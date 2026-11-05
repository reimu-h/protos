#ifndef PROTOS_IPC_H_
#define PROTOS_IPC_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "syscall_abi.h"

namespace protos {

struct Task;

// Synchronous message-passing IPC between processes over the fixed kernel
// ports `0 .. kIpcPortCount - 1` (the `IpcListen()`, `IpcRecv()`, `IpcReply()`
// and `IpcCall()` system calls in syscall_abi.h), with client-disconnect
// notifications.
//
// A process claims a port with `IpcContext::Listen()` and owns it until its
// `IpcContext` releases it. Port `kIpcPortFs` belongs to the user-space
// filesystem server: only `kUserCapDriver` processes may claim it (callers of
// `IpcCall()` enforce who may call it). A client's `IpcCall()` queues a
// transaction on the port and blocks; the owner's `IpcContext::Recv()` moves
// it to the port's in-service list and `IpcContext::Reply()` completes it.
//
// Concurrency rules:
// - Every function here may block or allocate, so callers run on a task with
//   interrupts enabled and hold no spinlock. All buffers are kernel memory:
//   callers copy user memory in and out themselves.
// - Each port's state is guarded by its own `IrqSpinLock`, which may be held
//   while waking a task (lock order: port lock, then the scheduler's locks).
// - The client owns its transaction (heap memory freed when `IpcCall()`
//   returns); the server side touches it only under the port lock while it is
//   linked into one of the port's lists, and the client unlinks it under the
//   lock before freeing it.
// - `should_stop(stop_ctx)` callbacks may run with a port lock held, so they
//   must not block or take a port lock.

constexpr uintptr_t kIpcNoOwner = 0;

// The most client-disconnect notifications pending on one port. At least
// `kTaskStackSlotCount` (task_stack.h), so in the kernel it always has room:
// `IpcContext::Recv()` drains every pending notification before popping any
// new call, and at most `kTaskStackSlotCount - 1` live client tasks can hold a
// popped call from the current owner at once.
constexpr int kIpcDisconnectRingCapacity = 4096;

// The most canceled in-service calls whose tokens one port remembers, so that
// the server's late `IpcContext::Reply()` to a call whose client was killed
// returns 0 instead of failing.
constexpr int kIpcCanceledTokenRingCapacity = 64;

// Environment seam for host unit tests and the kernel implementation.
class IpcEnvironment {
 public:
  virtual ~IpcEnvironment() = default;

  virtual int64_t NowTicks() = 0;
  virtual Task* CurrentTask() = 0;
  // Blocks the current task until `Wake()` (also one that happened since the
  // task last blocked) or, for `BlockUntil()`, until tick `deadline_tick`. May
  // return spuriously; callers loop.
  virtual void Block() = 0;
  virtual void BlockUntil(int64_t deadline_tick) = 0;
  virtual void Wake(Task* task) = 0;
};

#if !__STDC_HOSTED__
std::shared_ptr<IpcEnvironment> CreateKernelIpcEnvironment();
#endif

// Initializes the IPC subsystem with `env` (non-null). In the kernel,
// `kernel.cpp` calls `IpcInit()` once at boot after `DriverInit()`.
void IpcInit(std::shared_ptr<IpcEnvironment> env);
#if !__STDC_HOSTED__
void IpcInit();
#endif

// Returns the number of claimed IPC ports, and the owner token of `port`
// (`kIpcNoOwner` if it is free or out of range).
int IpcClaimedPortCount();
uintptr_t IpcPortOwner(int64_t port);

// Posts an `IpcMessageKind::kClientDisconnect` notification for `client_id`
// (`!= 0`) to `port` if `expected_owner` (`!= kIpcNoOwner`) still owns `port`,
// waking a server task if one is blocked in `IpcContext::Recv()`. A
// notification for `client_id` that is still pending on `port` is not
// repeated. While `kIpcDisconnectRingCapacity` notifications are pending (in
// host tests that invent more client IDs than `kTaskStackSlotCount`), waits a
// timer tick at a time until the server receives one, `expected_owner` no
// longer owns `port`, or `should_stop(stop_ctx)` returns true.
void IpcNotifyClientDisconnect(int64_t port,                              //
                               uint64_t client_id,                        //
                               uintptr_t expected_owner,                  //
                               bool (*should_stop)(void* ctx) = nullptr,  //
                               void* stop_ctx = nullptr);

// Sends the request `[req_buf, req_buf + req_size)` on `port` on behalf of
// client `client_id`, and blocks until the port's owner replies, the owner
// releases the port (`kSyscallErrorNoDevice`), or `should_stop(stop_ctx)`
// returns true (the call is canceled; returns `kSyscallErrorInvalid`). Fails at
// once, without sending anything, with `kSyscallErrorNoDevice` if no process
// owns `port`, with `kSyscallErrorDeadlock` if the caller owns it
// (`caller_owner_token` is the caller's `IpcContext::OwnerToken()`), and with
// `kSyscallErrorStale` if `expected_owner` isn't `kIpcNoOwner` and another
// context owns it.
//
// Returns the reply's status. On a non-negative status, copies the reply
// (`0 <= reply_size <= reply_cap` bytes) to `reply_buf` and writes its size to
// `*out_reply_size`. Writes to `*out_was_popped` whether the owner received
// the call before it finished or was canceled, and to `*out_owner` the owner
// token of the port's owner when the call was sent (else `kIpcNoOwner`). Each
// `out_*` pointer may be null.
int64_t IpcCall(int64_t port,                    //
                uint64_t client_id,              //
                uintptr_t caller_owner_token,    //
                uintptr_t expected_owner,        //
                const void* req_buf,             //
                int64_t req_size,                //
                void* reply_buf,                 //
                int64_t reply_cap,               //
                int64_t* out_reply_size,         //
                bool* out_was_popped,            //
                uintptr_t* out_owner,            //
                bool (*should_stop)(void* ctx),  //
                void* stop_ctx);

// The IPC state of one process (`UserProcess::ipc_`): the ports it owns.
// Releasing them (on exit, fault or kill) fails all of their queued and
// in-service calls with `kSyscallErrorNoDevice`.
class IpcContext {
 public:
  IpcContext();
  ~IpcContext();

  IpcContext(const IpcContext&) = delete;
  IpcContext& operator=(const IpcContext&) = delete;

  // The token that identifies this context as the owner of ports
  // (`IpcPortOwner()`): unique among all contexts ever created, and never
  // `kIpcNoOwner`.
  uintptr_t OwnerToken() const { return owner_token_; }

  // The `IpcListen()`, `IpcRecv()` and `IpcReply()` system calls
  // (syscall_abi.h) on kernel buffers. `Recv()` returns early (0) once
  // `should_stop(stop_ctx)` returns true.
  int64_t Listen(int64_t port, uint64_t capabilities);

  int64_t Recv(int64_t port,                    //
               void* out_buf,                   //
               int64_t buf_cap,                 //
               IpcReceivedMessage* out_msg,     //
               int64_t timeout_ticks,           //
               bool (*should_stop)(void* ctx),  //
               void* stop_ctx);

  int64_t Reply(int64_t port,           //
                uint64_t token,         //
                const void* reply_buf,  //
                int64_t reply_size,     //
                int64_t status);

  // Releases every port that this context owns, failing their calls with
  // `kSyscallErrorNoDevice` and waking any tasks waiting in `Recv()` (which
  // then fail with `kSyscallErrorAccess`), and returns how many it released.
  // Idempotent.
  int Release();

 private:
  const uintptr_t owner_token_;
  std::atomic<uint32_t> owned_ports_mask_{0};
};

#if __STDC_HOSTED__
// Fails every pending call, wakes waiting servers, releases every port, and
// drops the environment. Callers join their threads first.
void IpcResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_IPC_H_
