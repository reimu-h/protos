#include "ipc.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <utility>

#include "check.h"
#include "spinlock.h"
#include "syscall_abi.h"
#include "task_stack.h"
#if !__STDC_HOSTED__
#include "smp.h"
#include "task.h"
#endif

namespace protos {
namespace {

static_assert(kIpcPortCount > 0 && kIpcPortCount <= 32);
static_assert(kIpcDisconnectRingCapacity >= kTaskStackSlotCount);

enum class TxState : uint8_t {
  kQueued = 0,
  kInService = 1,
  kDone = 2,
};

struct IpcTransaction {
  IpcTransaction* prev = nullptr;
  IpcTransaction* next = nullptr;
  uint64_t token = 0;
  uint64_t client_id = 0;
  Task* client_task = nullptr;
  int64_t req_size = 0;
  int64_t reply_cap = 0;
  int64_t reply_size = 0;
  int64_t status = kSyscallErrorNoDevice;
  bool was_popped = false;
  std::atomic<TxState> state{TxState::kQueued};
  // The request, then the reply. Only the first `req_size` (then
  // `reply_size`) bytes are ever read, so it is left uninitialized.
  uint8_t buf[kIpcMaxMessageSize];
};

struct TxList {
  IpcTransaction* head = nullptr;
  IpcTransaction* tail = nullptr;

  void PushBack(IpcTransaction* const tx) {
    DCHECK(tx != nullptr && tx->prev == nullptr && tx->next == nullptr);
    tx->prev = tail;
    tx->next = nullptr;
    if (tail != nullptr) {
      tail->next = tx;
    } else {
      head = tx;
    }
    tail = tx;
  }

  IpcTransaction* PopFront() {
    IpcTransaction* const tx = head;
    if (tx == nullptr) {
      return nullptr;
    }
    Remove(tx);
    return tx;
  }

  void Remove(IpcTransaction* const tx) {
    DCHECK(tx != nullptr);
    if (tx->prev != nullptr) {
      tx->prev->next = tx->next;
    } else {
      DCHECK(head == tx);
      head = tx->next;
    }
    if (tx->next != nullptr) {
      tx->next->prev = tx->prev;
    } else {
      DCHECK(tail == tx);
      tail = tx->prev;
    }
    tx->prev = nullptr;
    tx->next = nullptr;
  }

  IpcTransaction* FindByToken(const uint64_t token) const {
    for (IpcTransaction* cur = head; cur != nullptr; cur = cur->next) {
      if (cur->token == token) {
        return cur;
      }
    }
    return nullptr;
  }
};

struct ServerWaiter {
  Task* task = nullptr;
  ServerWaiter* prev = nullptr;
  ServerWaiter* next = nullptr;
  bool linked = false;

  ~ServerWaiter() { DCHECK(!linked); }
};

struct ServerWaiterList {
  ServerWaiter* head = nullptr;
  ServerWaiter* tail = nullptr;

  void PushBack(ServerWaiter* const w) {
    DCHECK(w != nullptr && !w->linked && w->prev == nullptr &&
           w->next == nullptr);
    w->prev = tail;
    w->next = nullptr;
    if (tail != nullptr) {
      tail->next = w;
    } else {
      head = w;
    }
    tail = w;
    w->linked = true;
  }

  ServerWaiter* PopFront() {
    ServerWaiter* const w = head;
    if (w == nullptr) {
      return nullptr;
    }
    Remove(w);
    return w;
  }

  void Remove(ServerWaiter* const w) {
    DCHECK(w != nullptr);
    if (!w->linked) {
      return;
    }
    if (w->prev != nullptr) {
      w->prev->next = w->next;
    } else {
      DCHECK(head == w);
      head = w->next;
    }
    if (w->next != nullptr) {
      w->next->prev = w->prev;
    } else {
      DCHECK(tail == w);
      tail = w->prev;
    }
    w->prev = nullptr;
    w->next = nullptr;
    w->linked = false;
  }
};

struct CanceledCall {
  uint64_t token = 0;
  int64_t reply_cap = 0;
};

std::shared_ptr<IpcEnvironment> g_ipc_env;

struct IpcPort {
  IrqSpinLock lock;
  uintptr_t owner = kIpcNoOwner;
  uint64_t next_token = 1;
  ServerWaiterList server_waiters;
  TxList queued_list;
  TxList in_service_list;

  uint64_t disconnect_ring[kIpcDisconnectRingCapacity] = {};
  int disconnect_head = 0;
  int disconnect_count = 0;

  CanceledCall canceled_calls[kIpcCanceledTokenRingCapacity] = {};
  int canceled_write_idx = 0;
  int canceled_count = 0;

  uint64_t AllocateTokenLocked() {
    uint64_t token = next_token++;
    if (token == 0) {
      token = next_token++;
    }
    return token;
  }

  void WakeOneServerLocked() {
    if (ServerWaiter* const w = server_waiters.PopFront()) {
      if (w->task != nullptr && g_ipc_env != nullptr) {
        g_ipc_env->Wake(w->task);
      }
    }
  }

  void WakeAllServersLocked() {
    while (ServerWaiter* const w = server_waiters.PopFront()) {
      if (w->task != nullptr && g_ipc_env != nullptr) {
        g_ipc_env->Wake(w->task);
      }
    }
  }

  // Queues a notification for `client_id` (`!= 0`) unless one is pending, and
  // returns false if the ring is full.
  bool PushDisconnectLocked(const uint64_t client_id) {
    for (int i = 0; i < disconnect_count; ++i) {
      const int idx = (disconnect_head + i) % kIpcDisconnectRingCapacity;
      if (disconnect_ring[idx] == client_id) {
        return true;
      }
    }
    if (disconnect_count == kIpcDisconnectRingCapacity) {
      return false;
    }
    const int tail_idx =
        (disconnect_head + disconnect_count) % kIpcDisconnectRingCapacity;
    disconnect_ring[tail_idx] = client_id;
    ++disconnect_count;
    return true;
  }

  uint64_t PopDisconnectLocked() {
    DCHECK(disconnect_count > 0);
    const uint64_t client_id = disconnect_ring[disconnect_head];
    disconnect_ring[disconnect_head] = 0;
    disconnect_head = (disconnect_head + 1) % kIpcDisconnectRingCapacity;
    --disconnect_count;
    return client_id;
  }

  void ClearDisconnectsLocked() {
    disconnect_head = 0;
    disconnect_count = 0;
  }

  void RecordCanceledCallLocked(const uint64_t token, const int64_t reply_cap) {
    if (token == 0) {
      return;
    }
    canceled_calls[canceled_write_idx] =
        CanceledCall{.token = token, .reply_cap = reply_cap};
    canceled_write_idx =
        (canceled_write_idx + 1) % kIpcCanceledTokenRingCapacity;
    if (canceled_count < kIpcCanceledTokenRingCapacity) {
      ++canceled_count;
    }
  }

  CanceledCall* FindCanceledCallLocked(const uint64_t token) {
    if (token == 0) {
      return nullptr;
    }
    for (int i = 0; i < canceled_count; ++i) {
      const int idx =
          (canceled_write_idx + kIpcCanceledTokenRingCapacity - 1 - i) %
          kIpcCanceledTokenRingCapacity;
      if (canceled_calls[idx].token == token) {
        return &canceled_calls[idx];
      }
    }
    return nullptr;
  }

  void ClearCanceledCallsLocked() {
    for (int i = 0; i < kIpcCanceledTokenRingCapacity; ++i) {
      canceled_calls[i] = CanceledCall{};
    }
    canceled_write_idx = 0;
    canceled_count = 0;
  }
};

IpcPort g_ports[kIpcPortCount];
// The next `IpcContext::OwnerToken()`: tokens are never reused, so a stale
// token never names a later owner.
std::atomic<uintptr_t> g_next_owner_token{kIpcNoOwner + 1};

static void ResetPortStateLocked(IpcPort& port) {
  port.owner = kIpcNoOwner;
  port.next_token = 1;
  port.WakeAllServersLocked();
  port.ClearDisconnectsLocked();
  port.ClearCanceledCallsLocked();
}

static void FailListLocked(TxList& list, const int64_t error_status) {
  while (IpcTransaction* const tx = list.PopFront()) {
    tx->status = error_status;
    tx->reply_size = 0;
    tx->state.store(TxState::kDone, std::memory_order_release);
    if (tx->client_task != nullptr && g_ipc_env != nullptr) {
      g_ipc_env->Wake(tx->client_task);
    }
  }
}

}  // namespace

void IpcInit(std::shared_ptr<IpcEnvironment> env) {
  CHECK(env != nullptr);
  for (int i = 0; i < kIpcPortCount; ++i) {
    const IrqSpinLockGuard guard(g_ports[i].lock);
    CHECK(g_ports[i].owner == kIpcNoOwner);
    CHECK(g_ports[i].queued_list.head == nullptr);
    CHECK(g_ports[i].in_service_list.head == nullptr);
    ResetPortStateLocked(g_ports[i]);
  }
  g_ipc_env = std::move(env);
}

#if !__STDC_HOSTED__
void IpcInit() { IpcInit(CreateKernelIpcEnvironment()); }
#endif

int IpcClaimedPortCount() {
  int count = 0;
  for (int i = 0; i < kIpcPortCount; ++i) {
    const IrqSpinLockGuard guard(g_ports[i].lock);
    if (g_ports[i].owner != kIpcNoOwner) {
      ++count;
    }
  }
  return count;
}

uintptr_t IpcPortOwner(const int64_t port) {
  if (port < 0 || port >= kIpcPortCount) {
    return kIpcNoOwner;
  }
  const IrqSpinLockGuard guard(g_ports[port].lock);
  return g_ports[port].owner;
}

void IpcNotifyClientDisconnect(const int64_t port,                         //
                               const uint64_t client_id,                   //
                               const uintptr_t expected_owner,             //
                               bool (*const should_stop)(void* stop_ctx),  //
                               void* const stop_ctx) {
  CHECK(g_ipc_env != nullptr);
  if (port < 0 || port >= kIpcPortCount || client_id == 0 ||
      expected_owner == kIpcNoOwner) {
    return;
  }
  IpcPort& p = g_ports[port];
  for (;;) {
    {
      const IrqSpinLockGuard guard(p.lock);
      if (p.owner != expected_owner) {
        return;
      }
      if (p.PushDisconnectLocked(client_id)) {
        p.WakeOneServerLocked();
        return;
      }
    }
    if (should_stop != nullptr && should_stop(stop_ctx)) {
      return;
    }
    // The ring is full: give the server, which takes one notification per
    // `Recv()`, a timer tick to drain it.
    g_ipc_env->BlockUntil(g_ipc_env->NowTicks() + 1);
  }
}

int64_t IpcCall(const int64_t port,                         //
                const uint64_t client_id,                   //
                const uintptr_t caller_owner_token,         //
                const uintptr_t expected_owner,             //
                const void* const req_buf,                  //
                const int64_t req_size,                     //
                void* const reply_buf,                      //
                const int64_t reply_cap,                    //
                int64_t* const out_reply_size,              //
                bool* const out_was_popped,                 //
                uintptr_t* const out_owner,                 //
                bool (*const should_stop)(void* stop_ctx),  //
                void* const stop_ctx) {
  CHECK(g_ipc_env != nullptr);
  if (out_reply_size != nullptr) {
    *out_reply_size = 0;
  }
  if (out_was_popped != nullptr) {
    *out_was_popped = false;
  }
  if (out_owner != nullptr) {
    *out_owner = kIpcNoOwner;
  }
  if (port < 0 || port >= kIpcPortCount || req_size < 0 ||
      req_size > kIpcMaxMessageSize || reply_cap < 0 ||
      reply_cap > kIpcMaxMessageSize || (req_size > 0 && req_buf == nullptr) ||
      (reply_cap > 0 && reply_buf == nullptr)) {
    return kSyscallErrorInvalid;
  }

  // Default-initialized: `buf` stays uninitialized.
  std::unique_ptr<IpcTransaction> tx(new (std::nothrow) IpcTransaction);
  if (tx == nullptr) {
    return kSyscallErrorNoMemory;
  }
  tx->client_id = client_id;
  tx->client_task = g_ipc_env->CurrentTask();
  tx->req_size = req_size;
  tx->reply_cap = reply_cap;
  if (req_size > 0) {
    __builtin_memcpy(tx->buf, req_buf, static_cast<size_t>(req_size));
  }

  IpcPort& p = g_ports[port];
  uintptr_t server = kIpcNoOwner;
  {
    const IrqSpinLockGuard guard(p.lock);
    if (p.owner == kIpcNoOwner) {
      return kSyscallErrorNoDevice;
    }
    if (caller_owner_token != kIpcNoOwner && p.owner == caller_owner_token) {
      return kSyscallErrorDeadlock;
    }
    if (expected_owner != kIpcNoOwner && p.owner != expected_owner) {
      return kSyscallErrorStale;
    }
    if (should_stop != nullptr && should_stop(stop_ctx)) {
      return kSyscallErrorInvalid;
    }
    server = p.owner;
    p.queued_list.PushBack(tx.get());
    p.WakeOneServerLocked();
  }
  if (out_owner != nullptr) {
    *out_owner = server;
  }

  for (;;) {
    if (tx->state.load(std::memory_order_acquire) == TxState::kDone) {
      break;
    }
    if (should_stop != nullptr && should_stop(stop_ctx)) {
      break;
    }
    g_ipc_env->Block();
  }

  int64_t final_status = kSyscallErrorInvalid;
  int64_t final_reply_size = 0;
  bool was_popped = false;
  {
    const IrqSpinLockGuard guard(p.lock);
    const TxState st = tx->state.load(std::memory_order_relaxed);
    if (st == TxState::kDone) {
      final_status = tx->status;
      final_reply_size = tx->reply_size;
      was_popped = tx->was_popped;
    } else if (st == TxState::kQueued) {
      p.queued_list.Remove(tx.get());
      was_popped = false;
    } else {
      DCHECK(st == TxState::kInService);
      p.in_service_list.Remove(tx.get());
      p.RecordCanceledCallLocked(tx->token, tx->reply_cap);
      was_popped = true;
    }
  }

  if (out_was_popped != nullptr) {
    *out_was_popped = was_popped;
  }
  if (final_status >= 0) {
    DCHECK(final_reply_size >= 0 && final_reply_size <= reply_cap);
    if (final_reply_size > 0) {
      __builtin_memcpy(reply_buf, tx->buf,
                       static_cast<size_t>(final_reply_size));
    }
    if (out_reply_size != nullptr) {
      *out_reply_size = final_reply_size;
    }
  }
  return final_status;
}

IpcContext::IpcContext()
    : owner_token_(g_next_owner_token.fetch_add(1, std::memory_order_relaxed)) {
}

IpcContext::~IpcContext() { (void)Release(); }

int64_t IpcContext::Listen(const int64_t port, const uint64_t capabilities) {
  CHECK(g_ipc_env != nullptr);
  if (port < 0 || port >= kIpcPortCount) {
    return kSyscallErrorInvalid;
  }
  if (port == kIpcPortFs && (capabilities & kUserCapDriver) == 0) {
    return kSyscallErrorPermission;
  }

  IpcPort& p = g_ports[port];
  const uintptr_t self_owner = OwnerToken();
  {
    const IrqSpinLockGuard guard(p.lock);
    if (p.owner == self_owner) {
      return 0;
    }
    if (p.owner != kIpcNoOwner) {
      return kSyscallErrorBusy;
    }
    DCHECK(p.queued_list.head == nullptr);
    DCHECK(p.in_service_list.head == nullptr);
    ResetPortStateLocked(p);
    p.owner = self_owner;
  }
  owned_ports_mask_.fetch_or(uint32_t{1} << port, std::memory_order_acq_rel);
  return 0;
}

int64_t IpcContext::Recv(const int64_t port,                         //
                         void* const out_buf,                        //
                         const int64_t buf_cap,                      //
                         IpcReceivedMessage* const out_msg,          //
                         const int64_t timeout_ticks,                //
                         bool (*const should_stop)(void* stop_ctx),  //
                         void* const stop_ctx) {
  CHECK(g_ipc_env != nullptr);
  if (port < 0 || port >= kIpcPortCount || buf_cap < 0 ||
      buf_cap > kIpcMaxMessageSize || out_msg == nullptr ||
      (buf_cap > 0 && out_buf == nullptr) ||
      timeout_ticks > kSyscallIpcTimeoutMaxTicks) {
    return kSyscallErrorInvalid;
  }

  IpcPort& p = g_ports[port];
  const uintptr_t self_owner = OwnerToken();
  Task* const self_task = g_ipc_env->CurrentTask();
  const bool has_deadline = timeout_ticks > 0;
  const int64_t deadline =
      has_deadline ? (g_ipc_env->NowTicks() + timeout_ticks) : 0;

  ServerWaiter waiter{.task = self_task};
  auto try_dequeue = [&](const bool allow_wait_registration,
                         bool* const out_stop) -> int64_t {
    *out_stop = (should_stop != nullptr && should_stop(stop_ctx));
    const IrqSpinLockGuard guard(p.lock);
    p.server_waiters.Remove(&waiter);
    if (p.owner != self_owner) {
      return kSyscallErrorAccess;
    }
    if (*out_stop) {
      if (p.disconnect_count > 0 || p.queued_list.head != nullptr) {
        p.WakeOneServerLocked();
      }
      return 0;
    }
    if (p.disconnect_count > 0) {
      const uint64_t cid = p.PopDisconnectLocked();
      __builtin_memset(static_cast<void*>(out_msg), 0, sizeof(*out_msg));
      out_msg->token = 0;
      out_msg->client_id = cid;
      out_msg->size = 0;
      out_msg->kind = IpcMessageKind::kClientDisconnect;
      return 1;
    }
    while (IpcTransaction* const tx = p.queued_list.PopFront()) {
      if (tx->req_size > buf_cap) {
        tx->status = kSyscallErrorInvalid;
        tx->reply_size = 0;
        tx->was_popped = false;
        tx->state.store(TxState::kDone, std::memory_order_release);
        if (tx->client_task != nullptr) {
          g_ipc_env->Wake(tx->client_task);
        }
        continue;
      }
      const uint64_t token = p.AllocateTokenLocked();
      tx->token = token;
      tx->was_popped = true;
      tx->state.store(TxState::kInService, std::memory_order_release);
      p.in_service_list.PushBack(tx);
      if (tx->req_size > 0) {
        __builtin_memcpy(out_buf, tx->buf, static_cast<size_t>(tx->req_size));
      }
      __builtin_memset(static_cast<void*>(out_msg), 0, sizeof(*out_msg));
      out_msg->token = token;
      out_msg->client_id = tx->client_id;
      out_msg->size = tx->req_size;
      out_msg->kind = IpcMessageKind::kCall;
      out_msg->reply_cap = static_cast<int32_t>(tx->reply_cap);
      return 1;
    }
    if (allow_wait_registration) {
      p.server_waiters.PushBack(&waiter);
    }
    return 0;
  };

  for (;;) {
    bool stop = false;
    const int64_t rc =
        try_dequeue(/*allow_wait_registration=*/timeout_ticks != 0, &stop);
    if (rc != 0 || timeout_ticks == 0 || stop) {
      return rc;
    }
    if (has_deadline && g_ipc_env->NowTicks() >= deadline) {
      return try_dequeue(/*allow_wait_registration=*/false, &stop);
    }
    if (has_deadline) {
      g_ipc_env->BlockUntil(deadline);
    } else {
      g_ipc_env->Block();
    }
  }
}

int64_t IpcContext::Reply(const int64_t port,           //
                          const uint64_t token,         //
                          const void* const reply_buf,  //
                          const int64_t reply_size,     //
                          const int64_t status) {
  CHECK(g_ipc_env != nullptr);
  if (port < 0 || port >= kIpcPortCount || token == 0 || reply_size < 0 ||
      reply_size > kIpcMaxMessageSize ||
      (reply_size > 0 && reply_buf == nullptr)) {
    return kSyscallErrorInvalid;
  }
  if (status < 0 && (!SyscallIsError(status) || reply_size != 0)) {
    return kSyscallErrorInvalid;
  }

  IpcPort& p = g_ports[port];
  const uintptr_t self_owner = OwnerToken();
  const IrqSpinLockGuard guard(p.lock);
  if (p.owner != self_owner) {
    return kSyscallErrorAccess;
  }

  IpcTransaction* const tx = p.in_service_list.FindByToken(token);
  if (tx != nullptr) {
    if (reply_size > tx->reply_cap) {
      // The client can't take the reply; the call stays in service.
      return kSyscallErrorInvalid;
    }
    p.in_service_list.Remove(tx);
    if (reply_size > 0) {
      __builtin_memcpy(tx->buf, reply_buf, static_cast<size_t>(reply_size));
    }
    tx->reply_size = reply_size;
    tx->status = status;
    tx->state.store(TxState::kDone, std::memory_order_release);
    if (tx->client_task != nullptr) {
      g_ipc_env->Wake(tx->client_task);
    }
    return 0;
  }

  CanceledCall* const canceled = p.FindCanceledCallLocked(token);
  if (canceled != nullptr) {
    if (reply_size > canceled->reply_cap) {
      return kSyscallErrorInvalid;
    }
    *canceled = CanceledCall{};
    return 0;
  }
  return kSyscallErrorInvalid;
}

int IpcContext::Release() {
  const uint32_t mask =
      owned_ports_mask_.exchange(0, std::memory_order_acq_rel);
  if (mask == 0) {
    return 0;
  }
  const uintptr_t self_owner = OwnerToken();
  int released = 0;
  for (int i = 0; i < kIpcPortCount; ++i) {
    if ((mask & (uint32_t{1} << i)) == 0) {
      continue;
    }
    IpcPort& p = g_ports[i];
    const IrqSpinLockGuard guard(p.lock);
    if (p.owner != self_owner) {
      continue;
    }
    FailListLocked(p.queued_list, kSyscallErrorNoDevice);
    FailListLocked(p.in_service_list, kSyscallErrorNoDevice);
    ResetPortStateLocked(p);
    ++released;
  }
  return released;
}

#if __STDC_HOSTED__
void IpcResetForTest() {
  for (int i = 0; i < kIpcPortCount; ++i) {
    const IrqSpinLockGuard guard(g_ports[i].lock);
    FailListLocked(g_ports[i].queued_list, kSyscallErrorNoDevice);
    FailListLocked(g_ports[i].in_service_list, kSyscallErrorNoDevice);
    ResetPortStateLocked(g_ports[i]);
  }
  g_ipc_env.reset();
}
#endif

#if !__STDC_HOSTED__
namespace {

class KernelIpcEnvironment final : public IpcEnvironment {
 public:
  int64_t NowTicks() override {
    const CpuLocal* const cpu0 = SmpGetCpuLocal(0);
    return cpu0 != nullptr ? cpu0->timer_ticks.load(std::memory_order_acquire)
                           : 0;
  }

  Task* CurrentTask() override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    return scheduler != nullptr ? scheduler->CurrentTask() : nullptr;
  }

  void Block() override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    CHECK(scheduler != nullptr);
    scheduler->Block();
  }

  void BlockUntil(const int64_t deadline_tick) override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    CHECK(scheduler != nullptr);
    scheduler->BlockUntil(deadline_tick);
  }

  void Wake(Task* const task) override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    if (scheduler != nullptr && task != nullptr) {
      scheduler->Wake(task);
    }
  }
};

}  // namespace

std::shared_ptr<IpcEnvironment> CreateKernelIpcEnvironment() {
  return std::make_shared<KernelIpcEnvironment>();
}
#endif

}  // namespace protos
