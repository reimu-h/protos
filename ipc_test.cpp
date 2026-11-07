#include "ipc.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include "spinlock.h"
#include "syscall_abi.h"

namespace protos {

// A kernel task, played by a host thread (`FakeIpcEnvironment`).
struct Task {
  std::mutex mu;
  // Signaled when `wake_pending` is set.
  std::condition_variable wake_cv;
  // Signaled when `block_count` or `finished` changes.
  std::condition_variable state_cv;
  // Set by `Wake()` and cleared when a block ends, so that a `Wake()` before
  // the task blocks ends its next block at once.
  bool wake_pending = false;
  // How many times the task has started to block in `Block()` or
  // `BlockUntil()`.
  int block_count = 0;
  // How many of the task's next blocks in `Block()` end at once, without a
  // wake (`FakeIpcEnvironment::ReturnSpuriouslyFromNextBlocks()`).
  int spurious_block_returns = 0;
  // Whether the task's `TaskThreads` thread has returned from its function.
  bool finished = false;
};

namespace {

namespace t = ::testing;

// Bound on each wait for another thread: a lost wakeup must fail a test
// instead of hanging it.
constexpr std::chrono::seconds kWaitTimeout(30);

// What tests store in `*out_owner` before an `IpcCall()`, to check that the
// call overwrites it.
constexpr uintptr_t kUnwrittenOwner = ~uintptr_t{0};

// Waits until `done()` holds, with `task->mu` held, or until `timeout` has
// passed, and returns whether `done()` holds.
template <typename DoneFn>
static bool WaitForTask(Task* const task,                         //
                        const std::chrono::milliseconds timeout,  //
                        const DoneFn& done) {
  std::unique_lock<std::mutex> lock(task->mu);
  return task->state_cv.wait_for(lock, timeout, done);
}

// Waits until `task` has started to block `count` times since it was created,
// and returns whether it has: false if its thread returns from its function
// first, or if that takes longer than `kWaitTimeout`.
[[nodiscard]] static bool WaitUntilBlocked(Task* const task,
                                           const int count = 1) {
  std::unique_lock<std::mutex> lock(task->mu);
  return task->state_cv.wait_for(lock, kWaitTimeout, [task, count] {
    return task->block_count >= count || task->finished;
  }) && task->block_count >= count;
}

// How many times `task` has started to block.
static int BlockCount(Task* const task) {
  const std::lock_guard<std::mutex> lock(task->mu);
  return task->block_count;
}

// Waits until the thread that runs as `task` returns from its function or
// `task` starts to block, and returns whether the thread returned without
// blocking.
[[nodiscard]] static bool ReturnsWithoutBlocking(Task* const task) {
  std::unique_lock<std::mutex> lock(task->mu);
  return task->state_cv.wait_for(lock, kWaitTimeout, [task] {
    return task->finished || task->block_count > 0;
  }) && task->block_count == 0;
}

// Runs host threads as tasks, with a clock that advances only in
// `AdvanceTicks()`.
class FakeIpcEnvironment final : public IpcEnvironment {
 public:
  // Creates a task for a thread to run as (`RunAs()`).
  Task* NewTask() {
    auto task = std::make_unique<Task>();
    Task* const raw = task.get();
    const std::lock_guard<std::mutex> lock(tasks_mu_);
    tasks_.push_back(std::move(task));
    return raw;
  }

  // Makes the calling thread run as `task`.
  void RunAs(Task* const task) {
    current_task_ = task;
    current_env_id_ = id_;
  }

  int64_t NowTicks() override { return ticks_.load(std::memory_order_acquire); }

  // Advances the clock by `delta` ticks and wakes every task, as the timer
  // wakes tasks whose `BlockUntil()` deadline has come.
  void AdvanceTicks(const int64_t delta) {
    ticks_.fetch_add(delta, std::memory_order_acq_rel);
    const std::lock_guard<std::mutex> lock(tasks_mu_);
    for (const std::unique_ptr<Task>& task : tasks_) {
      Wake(task.get());
    }
  }

  // A thread that hasn't called `RunAs()` runs as a new task of its own.
  Task* CurrentTask() override {
    if (current_env_id_ != id_) {
      RunAs(NewTask());
    }
    return current_task_;
  }

  // Makes the calling task's next `count` blocks in `Block()` end at once,
  // without a wake, as `IpcEnvironment::Block()` may.
  void ReturnSpuriouslyFromNextBlocks(const int count) {
    Task* const self = CurrentTask();
    const std::lock_guard<std::mutex> lock(self->mu);
    self->spurious_block_returns = count;
  }

  void Block() override {
    Task* const self = CurrentTask();
    std::unique_lock<std::mutex> lock(self->mu);
    StartBlockLocked(*self);
    if (self->spurious_block_returns > 0) {
      --self->spurious_block_returns;
      return;
    }
    self->wake_cv.wait(lock, [self] { return self->wake_pending; });
    self->wake_pending = false;
  }

  void BlockUntil(const int64_t deadline_tick) override {
    Task* const self = CurrentTask();
    if (on_block_until_ != nullptr) {
      on_block_until_(this, deadline_tick);
    }
    std::unique_lock<std::mutex> lock(self->mu);
    StartBlockLocked(*self);
    self->wake_cv.wait(lock, [this, self, deadline_tick] {
      return self->wake_pending || NowTicks() >= deadline_tick;
    });
    self->wake_pending = false;
  }

  void Wake(Task* const task) override {
    if (task == nullptr) {
      return;
    }
    {
      const std::lock_guard<std::mutex> lock(task->mu);
      task->wake_pending = true;
    }
    task->wake_cv.notify_one();
  }

  // Called by `BlockUntil()`, on the blocking thread, before it blocks.
  void (*on_block_until_)(FakeIpcEnvironment* env, int64_t deadline) = nullptr;

 private:
  static void StartBlockLocked(Task& task) {
    ++task.block_count;
    task.state_cv.notify_all();
  }

  static inline std::atomic<int> next_id_{1};
  // The task that the calling thread runs as. It belongs to this environment
  // only if `current_env_id_ == id_`: the main thread also ran as tasks of the
  // environments of earlier tests.
  static inline thread_local Task* current_task_ = nullptr;
  static inline thread_local int current_env_id_ = 0;

  const int id_ = next_id_.fetch_add(1, std::memory_order_relaxed);
  std::atomic<int64_t> ticks_{100};
  std::mutex tasks_mu_;
  std::vector<std::unique_ptr<Task>> tasks_;
};

// Threads that run as tasks of a `FakeIpcEnvironment`. Declare it after the
// variables that its threads use. If the test returns while some of them
// still run (after a failed assertion), the destructor calls `unblock`, which
// must make them return (e.g. by releasing the port that they call or serve),
// wakes them until they do, and joins them.
class TaskThreads {
 public:
  TaskThreads(FakeIpcEnvironment& env, std::function<void()> unblock)
      : env_(env), unblock_(std::move(unblock)) {}

  TaskThreads(const TaskThreads&) = delete;
  TaskThreads& operator=(const TaskThreads&) = delete;

  ~TaskThreads() {
    bool running = false;
    for (const Thread& thread : threads_) {
      running = running || thread.thread.joinable();
    }
    if (!running) {
      return;
    }
    unblock_();
    const auto deadline = std::chrono::steady_clock::now() + kWaitTimeout;
    for (Thread& thread : threads_) {
      if (!thread.thread.joinable()) {
        continue;
      }
      Task* const task = thread.task;
      const auto finished = [task] { return task->finished; };
      while (!WaitForTask(task, std::chrono::milliseconds(1), finished)) {
        if (std::chrono::steady_clock::now() >= deadline) {
          // The thread uses variables of the test, so it can be neither
          // detached nor left running.
          std::fprintf(stderr, "A test thread still runs after `unblock`\n");
          std::abort();
        }
        env_.Wake(task);
      }
      thread.thread.join();
    }
  }

  // Starts a thread that runs `fn` as a new task, and returns the task.
  Task* Start(std::function<void()> fn) {
    Task* const task = env_.NewTask();
    FakeIpcEnvironment* const env = &env_;
    threads_.push_back({task, std::thread([env, task, fn = std::move(fn)] {
                          env->RunAs(task);
                          fn();
                          {
                            const std::lock_guard<std::mutex> lock(task->mu);
                            task->finished = true;
                          }
                          task->state_cv.notify_all();
                        })});
    return task;
  }

  // Waits until the thread that runs as `task` returns from its function, and
  // joins it. Returns false, and leaves the thread to the destructor, if that
  // takes longer than `kWaitTimeout`.
  [[nodiscard]] bool Join(Task* const task) {
    for (Thread& thread : threads_) {
      if (thread.task != task || !thread.thread.joinable()) {
        continue;
      }
      if (!WaitForTask(task, kWaitTimeout, [task] { return task->finished; })) {
        return false;
      }
      thread.thread.join();
      return true;
    }
    ADD_FAILURE() << "No unjoined thread runs as this task";
    return false;
  }

  // `Join()` for each thread that is not joined yet.
  [[nodiscard]] bool JoinAll() {
    for (Thread& thread : threads_) {
      if (thread.thread.joinable() && !Join(thread.task)) {
        return false;
      }
    }
    return true;
  }

 private:
  struct Thread {
    Task* task;
    std::thread thread;
  };

  FakeIpcEnvironment& env_;
  const std::function<void()> unblock_;
  std::vector<Thread> threads_;
};

static bool StopWhenFlagSet(void* const ctx) {
  return static_cast<std::atomic<bool>*>(ctx)->load(std::memory_order_acquire);
}

// A stop request that an action precedes (`RunActionThenStop()`).
struct StopWithAction {
  std::atomic<bool> armed{false};
  // May take a port lock.
  std::function<void()> action;
};

// The `should_stop` of a client with the `StopWithAction` at `ctx`. Returns
// false until it is armed, and when asked with a port lock held; otherwise runs
// its action the first time and returns true. `IpcCall()` asks without a port
// lock in its wait loop, after it has found its call unfinished and before it
// takes the port lock to end it, so arming it while the client waits makes the
// action happen in between.
static bool RunActionThenStop(void* const ctx) {
  StopWithAction* const stop = static_cast<StopWithAction*>(ctx);
  if (!stop->armed.load(std::memory_order_acquire) || HoldsAnyIrqSpinLock()) {
    return false;
  }
  if (stop->action != nullptr) {
    const std::function<void()> action = std::exchange(stop->action, nullptr);
    action();
  }
  return true;
}

class IpcTest : public t::Test {
 protected:
  void SetUp() override {
    IpcResetForTest();
    env_ = std::make_shared<FakeIpcEnvironment>();
    main_task_ = env_->CurrentTask();
    IpcInit(env_);
  }

  void TearDown() override { IpcResetForTest(); }

  std::shared_ptr<FakeIpcEnvironment> env_;
  // The task that the test's own thread runs as.
  Task* main_task_ = nullptr;
};

TEST_F(IpcTest, ListenValidatesPortAndCapabilities) {
  IpcContext unpriv;
  IpcContext priv;

  EXPECT_EQ(unpriv.Listen(-1, kUserCapNone), kSyscallErrorInvalid);
  EXPECT_EQ(unpriv.Listen(kIpcPortCount, kUserCapNone), kSyscallErrorInvalid);
  EXPECT_EQ(unpriv.Listen(kIpcPortFs, kUserCapNone), kSyscallErrorPermission);
  EXPECT_EQ(IpcClaimedPortCount(), 0);
  EXPECT_EQ(IpcPortOwner(kIpcPortFs), kIpcNoOwner);
  EXPECT_EQ(IpcPortOwner(-1), kIpcNoOwner);
  EXPECT_EQ(IpcPortOwner(kIpcPortCount), kIpcNoOwner);

  EXPECT_EQ(priv.Listen(kIpcPortFs, kUserCapDriver), 0);
  EXPECT_EQ(priv.Listen(kIpcPortFs, kUserCapDriver), 0);
  EXPECT_EQ(IpcClaimedPortCount(), 1);
  EXPECT_EQ(IpcPortOwner(kIpcPortFs), priv.OwnerToken());

  IpcContext priv2;
  EXPECT_EQ(priv2.Listen(kIpcPortFs, kUserCapDriver), kSyscallErrorBusy);

  for (int port = kIpcFirstUserPort; port < kIpcPortCount; ++port) {
    EXPECT_EQ(unpriv.Listen(port, kUserCapNone), 0);
    EXPECT_EQ(IpcPortOwner(port), unpriv.OwnerToken());
  }
  EXPECT_EQ(IpcClaimedPortCount(), kIpcPortCount);

  EXPECT_EQ(unpriv.Release(), kIpcPortCount - 1);
  EXPECT_EQ(IpcClaimedPortCount(), 1);
  EXPECT_EQ(priv.Release(), 1);
  EXPECT_EQ(IpcClaimedPortCount(), 0);
}

TEST_F(IpcTest, OwnerTokensAreUniqueAndNeverNoOwner) {
  std::vector<uintptr_t> tokens;
  // Each of these contexts ends before the next one starts, typically at the
  // same address.
  for (int i = 0; i < 4; ++i) {
    const IpcContext context;
    tokens.push_back(context.OwnerToken());
  }
  const IpcContext a;
  const IpcContext b;
  tokens.push_back(a.OwnerToken());
  tokens.push_back(b.OwnerToken());

  EXPECT_THAT(tokens, t::Each(t::Ne(kIpcNoOwner)));
  EXPECT_EQ(std::set<uintptr_t>(tokens.begin(), tokens.end()).size(),
            tokens.size());
}

TEST_F(IpcTest, CallWithInvalidArgumentsFailsWithoutBeingSent) {
  constexpr int kPort = kIpcFirstUserPort;
  constexpr int64_t kTooLarge = kIpcMaxMessageSize + 1;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  const uint8_t req[1] = {};
  uint8_t reply[1] = {};
  struct InvalidCall {
    const char* what;
    int64_t port;
    const void* req_buf;
    int64_t req_size;
    void* reply_buf;
    int64_t reply_cap;
  };
  const InvalidCall invalid_calls[] = {
      {"negative port", -1, req, 1, reply, 1},
      {"port past the last", kIpcPortCount, req, 1, reply, 1},
      {"negative req_size", kPort, req, -1, reply, 1},
      {"req_size too large", kPort, req, kTooLarge, reply, 1},
      {"null req_buf", kPort, nullptr, 1, reply, 1},
      {"negative reply_cap", kPort, req, 1, reply, -1},
      {"reply_cap too large", kPort, req, 1, reply, kTooLarge},
      {"null reply_buf", kPort, req, 1, nullptr, 1},
  };
  TaskThreads threads(*env_, [&] { server.Release(); });
  for (const InvalidCall& call : invalid_calls) {
    SCOPED_TRACE(call.what);
    int64_t reply_size = -1;
    bool was_popped = true;
    uintptr_t owner = kUnwrittenOwner;
    int64_t rc = 0;
    // If an invalid call were queued, it would block waiting for a reply.
    Task* const client = threads.Start([&] {
      rc = IpcCall(call.port,        //
                   /*client_id=*/1,  //
                   kIpcNoOwner,      //
                   kIpcNoOwner,      //
                   call.req_buf,     //
                   call.req_size,    //
                   call.reply_buf,   //
                   call.reply_cap,   //
                   &reply_size,      //
                   &was_popped,      //
                   &owner,           //
                   nullptr,          //
                   nullptr);
    });
    ASSERT_TRUE(ReturnsWithoutBlocking(client));
    ASSERT_TRUE(threads.Join(client));
    EXPECT_EQ(rc, kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    EXPECT_FALSE(was_popped);
    EXPECT_EQ(owner, kIpcNoOwner);
  }
  uint8_t srv_buf[1] = {};
  IpcReceivedMessage msg{};
  EXPECT_EQ(
      server.Recv(kPort, srv_buf, sizeof(srv_buf), &msg, 0, nullptr, nullptr),
      0);
}

TEST_F(IpcTest, CallUnclaimedAndSelfCallDeadlock) {
  const uint8_t req = 0x42;
  uint8_t reply = 0;
  int64_t reply_size = -1;
  bool was_popped = true;
  uintptr_t owner = kUnwrittenOwner;

  EXPECT_EQ(IpcCall(kIpcFirstUserPort,  //
                    /*client_id=*/1,    //
                    kIpcNoOwner,        //
                    kIpcNoOwner,        //
                    &req,               //
                    1,                  //
                    &reply,             //
                    1,                  //
                    &reply_size,        //
                    &was_popped,        //
                    &owner,             //
                    nullptr,            //
                    nullptr),
            kSyscallErrorNoDevice);
  EXPECT_EQ(reply_size, 0);
  EXPECT_FALSE(was_popped);
  EXPECT_EQ(owner, kIpcNoOwner);

  IpcContext server;
  ASSERT_EQ(server.Listen(kIpcFirstUserPort, kUserCapNone), 0);
  reply_size = -1;
  was_popped = true;
  owner = kUnwrittenOwner;
  int64_t rc = 0;
  TaskThreads threads(*env_, [&] { server.Release(); });
  // If the call were sent, it would wait for a reply.
  Task* const client = threads.Start([&] {
    rc = IpcCall(kIpcFirstUserPort,    //
                 /*client_id=*/1,      //
                 server.OwnerToken(),  //
                 kIpcNoOwner,          //
                 &req,                 //
                 1,                    //
                 &reply,               //
                 1,                    //
                 &reply_size,          //
                 &was_popped,          //
                 &owner,               //
                 nullptr,              //
                 nullptr);
  });
  ASSERT_TRUE(ReturnsWithoutBlocking(client));
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(rc, kSyscallErrorDeadlock);
  EXPECT_EQ(reply_size, 0);
  EXPECT_FALSE(was_popped);
  EXPECT_EQ(owner, kIpcNoOwner);

  uint8_t srv_buf[1] = {};
  IpcReceivedMessage msg{};
  EXPECT_EQ(server.Recv(kIpcFirstUserPort,  //
                        srv_buf,            //
                        sizeof(srv_buf),    //
                        &msg,               //
                        0,                  //
                        nullptr,            //
                        nullptr),
            0);
}

TEST_F(IpcTest, CallExpectingAnotherOwnerFailsStaleWithoutBeingSent) {
  constexpr int kPort = 7;
  // The port's first owner ends, and another context claims the port.
  IpcContext first_server;
  ASSERT_EQ(first_server.Listen(kPort, kUserCapNone), 0);
  const uintptr_t first_owner = first_server.OwnerToken();
  ASSERT_EQ(first_server.Release(), 1);
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  const uint8_t req = 1;
  int64_t reply_size = -1;
  bool was_popped = true;
  uintptr_t owner = kUnwrittenOwner;
  int64_t rc = 0;
  TaskThreads threads(*env_, [&] { server.Release(); });
  // If the call were sent, it would wait for a reply.
  Task* const client = threads.Start([&] {
    rc = IpcCall(kPort,                           //
                 /*client_id=*/1,                 //
                 kIpcNoOwner,                     //
                 /*expected_owner=*/first_owner,  //
                 &req,                            //
                 1,                               //
                 nullptr,                         //
                 0,                               //
                 &reply_size,                     //
                 &was_popped,                     //
                 &owner,                          //
                 nullptr,                         //
                 nullptr);
  });
  ASSERT_TRUE(ReturnsWithoutBlocking(client));
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(rc, kSyscallErrorStale);
  EXPECT_EQ(reply_size, 0);
  EXPECT_FALSE(was_popped);
  EXPECT_EQ(owner, kIpcNoOwner);
  uint8_t buf[1] = {};
  IpcReceivedMessage msg{};
  EXPECT_EQ(server.Recv(kPort, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 0);

  // Without an owner, a call fails with `kSyscallErrorNoDevice` whichever
  // owner it expects.
  ASSERT_EQ(server.Release(), 1);
  owner = kUnwrittenOwner;
  EXPECT_EQ(IpcCall(kPort,                                   //
                    /*client_id=*/1,                         //
                    kIpcNoOwner,                             //
                    /*expected_owner=*/server.OwnerToken(),  //
                    &req,                                    //
                    1,                                       //
                    nullptr,                                 //
                    0,                                       //
                    nullptr,                                 //
                    nullptr,                                 //
                    &owner,                                  //
                    nullptr,                                 //
                    nullptr),
            kSyscallErrorNoDevice);
  EXPECT_EQ(owner, kIpcNoOwner);
}

TEST_F(IpcTest, CallExpectingAnEndedOwnerIsStaleAlsoWhenTheCallerOwnsThePort) {
  constexpr int kPort = 8;
  // The owner that the call expects ends, and then the caller itself claims
  // the port.
  IpcContext first_server;
  ASSERT_EQ(first_server.Listen(kPort, kUserCapNone), 0);
  const uintptr_t first_owner = first_server.OwnerToken();
  ASSERT_EQ(first_server.Release(), 1);
  IpcContext caller;
  ASSERT_EQ(caller.Listen(kPort, kUserCapNone), 0);

  struct Case {
    const char* what;
    uintptr_t expected_owner;
    int64_t want;
  };
  const Case cases[] = {
      {"expects the owner that ended", first_owner, kSyscallErrorStale},
      {"expects itself", caller.OwnerToken(), kSyscallErrorDeadlock},
      {"expects any owner", kIpcNoOwner, kSyscallErrorDeadlock},
  };
  TaskThreads threads(*env_, [&] { caller.Release(); });
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    const uint8_t req = 1;
    int64_t reply_size = -1;
    bool was_popped = true;
    uintptr_t owner = kUnwrittenOwner;
    int64_t rc = 0;
    // If the call were sent, it would wait for a reply.
    Task* const client = threads.Start([&] {
      rc = IpcCall(kPort,                //
                   /*client_id=*/1,      //
                   caller.OwnerToken(),  //
                   c.expected_owner,     //
                   &req,                 //
                   1,                    //
                   nullptr,              //
                   0,                    //
                   &reply_size,          //
                   &was_popped,          //
                   &owner,               //
                   nullptr,              //
                   nullptr);
    });
    ASSERT_TRUE(ReturnsWithoutBlocking(client));
    ASSERT_TRUE(threads.Join(client));
    EXPECT_EQ(rc, c.want);
    EXPECT_EQ(reply_size, 0);
    EXPECT_FALSE(was_popped);
    EXPECT_EQ(owner, kIpcNoOwner);
  }
  uint8_t buf[1] = {};
  IpcReceivedMessage msg{};
  EXPECT_EQ(caller.Recv(kPort, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 0);
}

TEST_F(IpcTest, CallWithStopAlreadyRequestedIsNotSent) {
  constexpr int kPort = 14;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  int64_t recv_rc = 0;
  std::atomic<bool> stop{true};
  int64_t reply_size = -1;
  bool was_popped = true;
  uintptr_t owner = kUnwrittenOwner;
  int64_t rc = 0;
  TaskThreads threads(*env_, [&] { server.Release(); });
  // A call would wake the waiting server.
  Task* const receiver = threads.Start([&] {
    IpcReceivedMessage msg{};
    recv_rc = server.Recv(kPort, nullptr, 0, &msg, -1, nullptr, nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(receiver));
  Task* const client = threads.Start([&] {
    const uint8_t req = 0x71;
    rc = IpcCall(kPort,             //
                 /*client_id=*/71,  //
                 kIpcNoOwner,       //
                 kIpcNoOwner,       //
                 &req,              //
                 1,                 //
                 nullptr,           //
                 0,                 //
                 &reply_size,       //
                 &was_popped,       //
                 &owner,            //
                 &StopWhenFlagSet,  //
                 &stop);
  });
  ASSERT_TRUE(ReturnsWithoutBlocking(client));
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(rc, kSyscallErrorInvalid);
  EXPECT_EQ(reply_size, 0);
  EXPECT_FALSE(was_popped);
  // Only a call that was sent reports the owner that it was sent to.
  EXPECT_EQ(owner, kIpcNoOwner);

  // Nothing woke the server: only the release ends its wait.
  EXPECT_EQ(server.Release(), 1);
  ASSERT_TRUE(threads.Join(receiver));
  EXPECT_EQ(recv_rc, kSyscallErrorAccess);
  EXPECT_EQ(BlockCount(receiver), 1);
}

TEST_F(IpcTest, CallReportsTheOwnerThatReceivedIt) {
  constexpr int kPort = 8;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);
  const uintptr_t server_owner = server.OwnerToken();

  // One call names the owner that it expects, the other accepts any owner.
  int64_t expecting_rc = -1;
  uintptr_t expecting_owner = kUnwrittenOwner;
  int64_t any_rc = -1;
  uintptr_t any_owner = kUnwrittenOwner;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const expecting_client = threads.Start([&] {
    expecting_rc = IpcCall(kPort,                            //
                           /*client_id=*/1,                  //
                           kIpcNoOwner,                      //
                           /*expected_owner=*/server_owner,  //
                           nullptr,                          //
                           0,                                //
                           nullptr,                          //
                           0,                                //
                           nullptr,                          //
                           nullptr,                          //
                           &expecting_owner,                 //
                           nullptr,                          //
                           nullptr);
  });
  Task* const any_client = threads.Start([&] {
    any_rc = IpcCall(kPort,                           //
                     /*client_id=*/2,                 //
                     kIpcNoOwner,                     //
                     /*expected_owner=*/kIpcNoOwner,  //
                     nullptr,                         //
                     0,                               //
                     nullptr,                         //
                     0,                               //
                     nullptr,                         //
                     nullptr,                         //
                     &any_owner,                      //
                     nullptr,                         //
                     nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(expecting_client));
  ASSERT_TRUE(WaitUntilBlocked(any_client));

  // Both calls are queued. Each one returns its client's id.
  for (int i = 0; i < 2; ++i) {
    IpcReceivedMessage msg{};
    ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
    const int64_t status = static_cast<int64_t>(msg.client_id);
    EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, status), 0);
  }
  ASSERT_TRUE(threads.JoinAll());
  EXPECT_EQ(expecting_rc, 1);
  EXPECT_EQ(expecting_owner, server_owner);
  EXPECT_EQ(any_rc, 2);
  EXPECT_EQ(any_owner, server_owner);
}

TEST_F(IpcTest, RecvAndReplyValidateArgumentsAndOwnership) {
  IpcContext server;
  IpcContext other;
  ASSERT_EQ(server.Listen(1, kUserCapNone), 0);

  uint8_t buf[16] = {};
  IpcReceivedMessage msg{};

  EXPECT_EQ(other.Recv(1, buf, sizeof(buf), &msg, 0, nullptr, nullptr),
            kSyscallErrorAccess);
  EXPECT_EQ(server.Recv(-1, buf, sizeof(buf), &msg, 0, nullptr, nullptr),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Recv(1, nullptr, 4, &msg, 0, nullptr, nullptr),
            kSyscallErrorInvalid);
  EXPECT_EQ(
      server.Recv(1, buf, kIpcMaxMessageSize + 1, &msg, 0, nullptr, nullptr),
      kSyscallErrorInvalid);
  EXPECT_EQ(server.Recv(1, buf, sizeof(buf), nullptr, 0, nullptr, nullptr),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Recv(1, buf, sizeof(buf), &msg,
                        kSyscallIpcTimeoutMaxTicks + 1, nullptr, nullptr),
            kSyscallErrorInvalid);

  // Non-blocking poll returns 0 when empty.
  EXPECT_EQ(server.Recv(1, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 0);

  // Timed Recv wakes once ticks reach the deadline and returns 0.
  env_->on_block_until_ = [](FakeIpcEnvironment* const env,
                             const int64_t deadline) {
    env->AdvanceTicks(deadline - env->NowTicks());
  };
  EXPECT_EQ(server.Recv(1, buf, sizeof(buf), &msg, 5, nullptr, nullptr), 0);
  env_->on_block_until_ = nullptr;

  // Reply validation with no call in service.
  EXPECT_EQ(other.Reply(1, 1, nullptr, 0, 0), kSyscallErrorAccess);
  EXPECT_EQ(server.Reply(-1, 1, nullptr, 0, 0), kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(kIpcPortCount, 1, nullptr, 0, 0),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, 0, nullptr, 0, 0), kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, 999, nullptr, 0, 0), kSyscallErrorInvalid);

  // Reply validation against a live in-service token: invalid sizes, payloads,
  // statuses, and non-owner callers fail without consuming the call.
  int64_t client_rc = 0;
  int64_t client_reply_size = -1;
  uint8_t client_reply[16] = {};
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    const uint8_t req = 0x5A;
    client_rc = IpcCall(1,                     //
                        /*client_id=*/77,      //
                        kIpcNoOwner,           //
                        kIpcNoOwner,           //
                        &req,                  //
                        1,                     //
                        client_reply,          //
                        sizeof(client_reply),  //
                        &client_reply_size,    //
                        nullptr,               //
                        nullptr,               //
                        nullptr,               //
                        nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));
  ASSERT_EQ(server.Recv(1, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 1);
  EXPECT_EQ(other.Reply(1, msg.token, nullptr, 0, 0), kSyscallErrorAccess);
  EXPECT_EQ(server.Reply(1, msg.token, buf, -1, 0), kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, msg.token, buf, kIpcMaxMessageSize + 1, 0),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, msg.token, nullptr, 4, 0), kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, msg.token, buf, 4, kSyscallErrorNoEntry),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, msg.token, nullptr, 0, -4096),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, msg.token, nullptr, 0, -5000),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(1, msg.token, nullptr, 0, kSyscallErrorNoEntry), 0);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(client_rc, kSyscallErrorNoEntry);
  EXPECT_EQ(client_reply_size, 0);
}

TEST_F(IpcTest, SynchronousRendezvousMaxPayloadAndOutOfOrderReply) {
  constexpr int kPort = 2;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  std::vector<uint8_t> req_a(kIpcMaxMessageSize);
  for (int i = 0; i < kIpcMaxMessageSize; ++i) {
    req_a[i] = static_cast<uint8_t>((i * 7 + 3) & 0xFF);
  }
  std::vector<uint8_t> reply_a(kIpcMaxMessageSize, 0);
  int64_t status_a = -1;
  int64_t reply_size_a = -1;

  const char req_b[] = "second_client";
  char reply_b[32] = {};
  int64_t status_b = -1;
  int64_t reply_size_b = -1;

  std::vector<uint8_t> srv_buf_a(kIpcMaxMessageSize);
  IpcReceivedMessage msg_a{};
  int64_t recv_a_rc = -1;
  TaskThreads threads(*env_, [&] { server.Release(); });

  // The server waits in `Recv()` before client A calls.
  Task* const receiver = threads.Start([&] {
    recv_a_rc = server.Recv(kPort,               //
                            srv_buf_a.data(),    //
                            kIpcMaxMessageSize,  //
                            &msg_a,              //
                            -1,                  //
                            nullptr,             //
                            nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(receiver));
  Task* const client_a = threads.Start([&] {
    status_a = IpcCall(kPort,               //
                       /*client_id=*/101,   //
                       kIpcNoOwner,         //
                       kIpcNoOwner,         //
                       req_a.data(),        //
                       kIpcMaxMessageSize,  //
                       reply_a.data(),      //
                       kIpcMaxMessageSize,  //
                       &reply_size_a,       //
                       nullptr,             //
                       nullptr,             //
                       nullptr,             //
                       nullptr);
  });

  // Receive A first, then start B and reply to B before A (out-of-order).
  ASSERT_TRUE(threads.Join(receiver));
  ASSERT_EQ(recv_a_rc, 1);
  EXPECT_EQ(msg_a.kind, IpcMessageKind::kCall);
  EXPECT_EQ(msg_a.client_id, 101u);
  EXPECT_EQ(msg_a.size, kIpcMaxMessageSize);
  EXPECT_EQ(msg_a.reply_cap, kIpcMaxMessageSize);
  EXPECT_EQ(srv_buf_a, req_a);

  Task* const client_b = threads.Start([&] {
    status_b = IpcCall(kPort,              //
                       /*client_id=*/202,  //
                       kIpcNoOwner,        //
                       kIpcNoOwner,        //
                       req_b,              //
                       sizeof(req_b),      //
                       reply_b,            //
                       sizeof(reply_b),    //
                       &reply_size_b,      //
                       nullptr,            //
                       nullptr,            //
                       nullptr,            //
                       nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(client_b));

  char srv_buf_b[64] = {};
  IpcReceivedMessage msg_b{};
  ASSERT_EQ(server.Recv(kPort,              //
                        srv_buf_b,          //
                        sizeof(srv_buf_b),  //
                        &msg_b,             //
                        0,                  //
                        nullptr,            //
                        nullptr),
            1);
  EXPECT_EQ(msg_b.kind, IpcMessageKind::kCall);
  EXPECT_EQ(msg_b.client_id, 202u);
  EXPECT_EQ(msg_b.reply_cap, static_cast<int32_t>(sizeof(reply_b)));
  EXPECT_NE(msg_b.token, msg_a.token);
  EXPECT_STREQ(srv_buf_b, "second_client");

  // Reply to B first.
  const char kReplyB[] = "ack_b";
  ASSERT_EQ(server.Reply(kPort, msg_b.token, kReplyB, sizeof(kReplyB), 77), 0);
  ASSERT_TRUE(threads.Join(client_b));
  EXPECT_EQ(status_b, 77);
  EXPECT_EQ(reply_size_b, static_cast<int64_t>(sizeof(kReplyB)));
  EXPECT_STREQ(reply_b, "ack_b");

  // Now reply to A with a full `kIpcMaxMessageSize` payload.
  for (int i = 0; i < kIpcMaxMessageSize; ++i) {
    srv_buf_a[i] ^= 0xA5U;
  }
  ASSERT_EQ(server.Reply(kPort,               //
                         msg_a.token,         //
                         srv_buf_a.data(),    //
                         kIpcMaxMessageSize,  //
                         12345),
            0);
  ASSERT_TRUE(threads.Join(client_a));
  EXPECT_EQ(status_a, 12345);
  EXPECT_EQ(reply_size_a, kIpcMaxMessageSize);
  EXPECT_EQ(reply_a, srv_buf_a);
}

TEST_F(IpcTest, OversizedRequestFailsItsCallAndOversizedReplyIsRejected) {
  constexpr int kPort = 3;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  int64_t big_status = 0;
  bool big_was_popped = true;
  const uint8_t small_req[4] = {1, 2, 3, 4};
  uint8_t small_reply[2] = {};
  int64_t small_reply_size = -1;
  int64_t small_status = 0;
  TaskThreads threads(*env_, [&] { server.Release(); });

  // A call whose request doesn't fit in the server's receive buffer is queued
  // first, then one whose request fits.
  Task* const big_client = threads.Start([&] {
    const uint8_t big_req[64] = {};
    big_status = IpcCall(kPort,            //
                         /*client_id=*/1,  //
                         kIpcNoOwner,      //
                         kIpcNoOwner,      //
                         big_req,          //
                         sizeof(big_req),  //
                         nullptr,          //
                         0,                //
                         nullptr,          //
                         &big_was_popped,  //
                         nullptr,          //
                         nullptr,          //
                         nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(big_client));
  Task* const small_client = threads.Start([&] {
    small_status = IpcCall(kPort,                //
                           /*client_id=*/2,      //
                           kIpcNoOwner,          //
                           kIpcNoOwner,          //
                           small_req,            //
                           sizeof(small_req),    //
                           small_reply,          //
                           sizeof(small_reply),  //
                           &small_reply_size,    //
                           nullptr,              //
                           nullptr,              //
                           nullptr,              //
                           nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(small_client));

  // One `Recv()` with a 16-byte buffer fails the 64-byte request and receives
  // the 4-byte one.
  uint8_t srv_buf[16] = {};
  IpcReceivedMessage msg{};
  ASSERT_EQ(
      server.Recv(kPort, srv_buf, sizeof(srv_buf), &msg, 0, nullptr, nullptr),
      1);
  ASSERT_TRUE(threads.Join(big_client));
  EXPECT_EQ(big_status, kSyscallErrorInvalid);
  EXPECT_FALSE(big_was_popped);
  EXPECT_EQ(msg.kind, IpcMessageKind::kCall);
  EXPECT_EQ(msg.client_id, 2u);
  EXPECT_EQ(msg.size, static_cast<int64_t>(sizeof(small_req)));
  EXPECT_EQ(msg.reply_cap, static_cast<int32_t>(sizeof(small_reply)));
  EXPECT_THAT(std::vector<uint8_t>(srv_buf, srv_buf + sizeof(small_req)),
              t::ElementsAreArray(small_req));

  // A reply longer than the client's `reply_cap` fails, and the call stays in
  // service until a reply that fits.
  const uint8_t long_reply[3] = {9, 8, 7};
  EXPECT_EQ(server.Reply(kPort, msg.token, long_reply, sizeof(long_reply), 3),
            kSyscallErrorInvalid);
  const uint8_t fitting_reply[2] = {6, 5};
  EXPECT_EQ(
      server.Reply(kPort, msg.token, fitting_reply, sizeof(fitting_reply), 2),
      0);
  ASSERT_TRUE(threads.Join(small_client));
  EXPECT_EQ(small_status, 2);
  EXPECT_EQ(small_reply_size, 2);
  EXPECT_THAT(small_reply, t::ElementsAre(6, 5));
}

TEST_F(IpcTest, CallBlocksAgainAfterSpuriousWakeupsUntilItIsAnswered) {
  constexpr int kPort = 14;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  int64_t rc = -1;
  uint8_t reply[2] = {};
  int64_t reply_size = -1;
  bool was_popped = false;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    env_->ReturnSpuriouslyFromNextBlocks(2);
    const uint8_t req = 0x31;
    rc = IpcCall(kPort,             //
                 /*client_id=*/31,  //
                 kIpcNoOwner,       //
                 kIpcNoOwner,       //
                 &req,              //
                 1,                 //
                 reply,             //
                 sizeof(reply),     //
                 &reply_size,       //
                 &was_popped,       //
                 nullptr,           //
                 nullptr,           //
                 nullptr);
  });
  // The client's first two blocks end at once, and it blocks again after each.
  ASSERT_TRUE(WaitUntilBlocked(client, 3));
  uint8_t srv_buf[1] = {};
  IpcReceivedMessage msg{};
  ASSERT_EQ(
      server.Recv(kPort, srv_buf, sizeof(srv_buf), &msg, 0, nullptr, nullptr),
      1);
  EXPECT_EQ(srv_buf[0], 0x31);
  // A wake with neither a reply nor a stop request only makes it block again.
  env_->Wake(client);
  ASSERT_TRUE(WaitUntilBlocked(client, 4));

  const uint8_t kReply[2] = {0xAB, 0xCD};
  ASSERT_EQ(server.Reply(kPort, msg.token, kReply, sizeof(kReply), 5), 0);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(rc, 5);
  EXPECT_EQ(reply_size, 2);
  EXPECT_THAT(reply, t::ElementsAre(0xAB, 0xCD));
  EXPECT_TRUE(was_popped);
  EXPECT_EQ(BlockCount(client), 4);
}

TEST_F(IpcTest, ClientKilledWhileQueuedAndWhileInService) {
  constexpr int kPort = 4;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  std::atomic<bool> stop_queued{false};
  bool queued_was_popped = true;
  uintptr_t queued_owner = kUnwrittenOwner;
  int64_t queued_rc = 0;
  std::atomic<bool> stop_in_service{false};
  bool in_service_was_popped = false;
  int64_t in_service_rc = 0;
  TaskThreads threads(*env_, [&] { server.Release(); });

  // 1. Client killed while still queued (`was_popped == false`).
  Task* const queued_client = threads.Start([&] {
    const uint8_t b = 1;
    queued_rc = IpcCall(kPort,               //
                        /*client_id=*/10,    //
                        kIpcNoOwner,         //
                        kIpcNoOwner,         //
                        &b,                  //
                        1,                   //
                        nullptr,             //
                        0,                   //
                        nullptr,             //
                        &queued_was_popped,  //
                        &queued_owner,       //
                        &StopWhenFlagSet,    //
                        &stop_queued);
  });
  // Once the client blocks, its call is queued on the port: stop it without
  // calling `Recv`.
  ASSERT_TRUE(WaitUntilBlocked(queued_client));
  stop_queued.store(true, std::memory_order_release);
  env_->Wake(queued_client);
  ASSERT_TRUE(threads.Join(queued_client));
  EXPECT_EQ(queued_rc, kSyscallErrorInvalid);
  EXPECT_FALSE(queued_was_popped);
  EXPECT_EQ(queued_owner, server.OwnerToken());

  uint8_t srv_buf[8] = {};
  IpcReceivedMessage msg{};
  EXPECT_EQ(
      server.Recv(kPort, srv_buf, sizeof(srv_buf), &msg, 0, nullptr, nullptr),
      0);

  // 2. Client killed AFTER `Recv` popped its call (`was_popped == true`).
  Task* const in_service_client = threads.Start([&] {
    const uint8_t b = 0x77;
    in_service_rc = IpcCall(kPort,                   //
                            /*client_id=*/11,        //
                            kIpcNoOwner,             //
                            kIpcNoOwner,             //
                            &b,                      //
                            1,                       //
                            nullptr,                 //
                            0,                       //
                            nullptr,                 //
                            &in_service_was_popped,  //
                            nullptr,                 //
                            &StopWhenFlagSet,        //
                            &stop_in_service);
  });
  ASSERT_TRUE(WaitUntilBlocked(in_service_client));
  ASSERT_EQ(
      server.Recv(kPort, srv_buf, sizeof(srv_buf), &msg, 0, nullptr, nullptr),
      1);
  EXPECT_EQ(msg.client_id, 11u);
  EXPECT_EQ(srv_buf[0], 0x77);

  stop_in_service.store(true, std::memory_order_release);
  env_->Wake(in_service_client);
  ASSERT_TRUE(threads.Join(in_service_client));
  EXPECT_EQ(in_service_rc, kSyscallErrorInvalid);
  EXPECT_TRUE(in_service_was_popped);

  // Server replies late to the canceled token: first Reply succeeds with 0,
  // second duplicate Reply fails with `kSyscallErrorInvalid`.
  EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 0), 0);
  EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 0),
            kSyscallErrorInvalid);
}

TEST_F(IpcTest, StopRequestedAfterTheReplyIsStoredReturnsTheReply) {
  constexpr int kPort = 14;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  IpcReceivedMessage msg{};
  const uint8_t kReply[2] = {0x12, 0x34};
  int64_t reply_rc = -1;
  // The server answers the call when the client checks for a stop, after it
  // has found the call unanswered, and then the stop is requested.
  StopWithAction stop;
  stop.action = [&] {
    reply_rc = server.Reply(kPort, msg.token, kReply, sizeof(kReply), 9);
  };
  int64_t rc = -1;
  uint8_t reply[2] = {};
  int64_t reply_size = -1;
  bool was_popped = false;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    rc = IpcCall(kPort,               //
                 /*client_id=*/41,    //
                 kIpcNoOwner,         //
                 kIpcNoOwner,         //
                 nullptr,             //
                 0,                   //
                 reply,               //
                 sizeof(reply),       //
                 &reply_size,         //
                 &was_popped,         //
                 nullptr,             //
                 &RunActionThenStop,  //
                 &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));
  ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
  stop.armed.store(true, std::memory_order_release);
  env_->Wake(client);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(reply_rc, 0);
  EXPECT_EQ(rc, 9);
  EXPECT_EQ(reply_size, 2);
  EXPECT_THAT(reply, t::ElementsAre(0x12, 0x34));
  EXPECT_TRUE(was_popped);
  // The call ended with its reply, not canceled, so the port doesn't remember
  // its token as canceled: another reply fails instead of being dropped.
  EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 0),
            kSyscallErrorInvalid);
}

TEST_F(IpcTest, LateReplyToCanceledCallEnforcesReplyCap) {
  constexpr int kPort = 4;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  std::atomic<bool> stop{false};
  uint8_t reply_buf[2] = {};
  bool was_popped = false;
  int64_t rc = 0;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    const uint8_t req = 0x33;
    rc = IpcCall(kPort,              //
                 /*client_id=*/12,   //
                 kIpcNoOwner,        //
                 kIpcNoOwner,        //
                 &req,               //
                 1,                  //
                 reply_buf,          //
                 sizeof(reply_buf),  //
                 nullptr,            //
                 &was_popped,        //
                 nullptr,            //
                 &StopWhenFlagSet,   //
                 &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));

  uint8_t srv_buf[8] = {};
  IpcReceivedMessage msg{};
  ASSERT_EQ(
      server.Recv(kPort, srv_buf, sizeof(srv_buf), &msg, 0, nullptr, nullptr),
      1);
  EXPECT_EQ(msg.reply_cap, static_cast<int32_t>(sizeof(reply_buf)));

  stop.store(true, std::memory_order_release);
  env_->Wake(client);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(rc, kSyscallErrorInvalid);
  EXPECT_TRUE(was_popped);

  // Oversized late reply fails with `kSyscallErrorInvalid` without consuming
  // the canceled call; a fitting reply then succeeds once.
  const uint8_t too_big[3] = {1, 2, 3};
  EXPECT_EQ(server.Reply(kPort, msg.token, too_big, sizeof(too_big), 3),
            kSyscallErrorInvalid);
  const uint8_t fits[2] = {4, 5};
  EXPECT_EQ(server.Reply(kPort, msg.token, fits, sizeof(fits), 2), 0);
  EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 0),
            kSyscallErrorInvalid);
}

TEST_F(IpcTest, LateReplyFailsOnceTheCanceledCallIsForgotten) {
  constexpr int kPort = 10;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  // Cancels one more received call than the port remembers, one at a time.
  std::vector<uint64_t> tokens;
  for (int i = 0; i < kIpcCanceledTokenRingCapacity + 1; ++i) {
    SCOPED_TRACE(i);
    std::atomic<bool> stop{false};
    bool was_popped = false;
    int64_t rc = 0;
    TaskThreads threads(*env_, [&] { server.Release(); });
    Task* const client = threads.Start([&] {
      rc = IpcCall(kPort,                                       //
                   /*client_id=*/static_cast<uint64_t>(i + 1),  //
                   kIpcNoOwner,                                 //
                   kIpcNoOwner,                                 //
                   nullptr,                                     //
                   0,                                           //
                   nullptr,                                     //
                   0,                                           //
                   nullptr,                                     //
                   &was_popped,                                 //
                   nullptr,                                     //
                   &StopWhenFlagSet,                            //
                   &stop);
    });
    ASSERT_TRUE(WaitUntilBlocked(client));
    IpcReceivedMessage msg{};
    ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
    tokens.push_back(msg.token);
    stop.store(true, std::memory_order_release);
    env_->Wake(client);
    ASSERT_TRUE(threads.Join(client));
    EXPECT_EQ(rc, kSyscallErrorInvalid);
    EXPECT_TRUE(was_popped);
  }

  // The oldest token was evicted; the next oldest is still remembered.
  EXPECT_EQ(server.Reply(kPort, tokens.front(), nullptr, 0, 0),
            kSyscallErrorInvalid);
  EXPECT_EQ(server.Reply(kPort, tokens[1], nullptr, 0, 0), 0);
  EXPECT_EQ(server.Reply(kPort, tokens.back(), nullptr, 0, 0), 0);
}

TEST_F(IpcTest, ServerReleaseWakesQueuedAndInServiceClients) {
  auto server = std::make_unique<IpcContext>();
  ASSERT_EQ(server->Listen(5, kUserCapNone), 0);
  const uintptr_t server_owner = server->OwnerToken();

  int64_t rc_in_service = 0;
  bool popped_in_service = false;
  uintptr_t owner_in_service = kUnwrittenOwner;
  int64_t rc_queued = 0;
  bool popped_queued = true;
  // Set only if the test fails, to end calls that `Release()` didn't.
  std::atomic<bool> stop{false};
  TaskThreads threads(*env_, [&] {
    stop.store(true, std::memory_order_release);
    server->Release();
  });
  Task* const c1 = threads.Start([&] {
    const uint8_t req = 1;
    rc_in_service = IpcCall(5,                   //
                            /*client_id=*/21,    //
                            kIpcNoOwner,         //
                            kIpcNoOwner,         //
                            &req,                //
                            1,                   //
                            nullptr,             //
                            0,                   //
                            nullptr,             //
                            &popped_in_service,  //
                            &owner_in_service,   //
                            &StopWhenFlagSet,    //
                            &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(c1));

  uint8_t buf[8] = {};
  IpcReceivedMessage msg{};
  ASSERT_EQ(server->Recv(5, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 1);

  Task* const c2 = threads.Start([&] {
    const uint8_t req = 2;
    rc_queued = IpcCall(5,                 //
                        /*client_id=*/22,  //
                        kIpcNoOwner,       //
                        kIpcNoOwner,       //
                        &req,              //
                        1,                 //
                        nullptr,           //
                        0,                 //
                        nullptr,           //
                        &popped_queued,    //
                        nullptr,           //
                        &StopWhenFlagSet,  //
                        &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(c2));

  EXPECT_EQ(server->Release(), 1);
  ASSERT_TRUE(threads.JoinAll());
  EXPECT_EQ(rc_in_service, kSyscallErrorNoDevice);
  EXPECT_TRUE(popped_in_service);
  EXPECT_EQ(owner_in_service, server_owner);
  EXPECT_EQ(rc_queued, kSyscallErrorNoDevice);
  EXPECT_FALSE(popped_queued);
  EXPECT_EQ(IpcPortOwner(5), kIpcNoOwner);

  // Another server can immediately claim port 5.
  IpcContext replacement;
  EXPECT_EQ(replacement.Listen(5, kUserCapNone), 0);
}

TEST_F(IpcTest, ReleaseRacingAClientThatCancelsEndsTheCallOnce) {
  constexpr int kPort = 15;
  // 1. The server releases the port when the client checks for a stop, after
  // it has found its call unfinished and before it takes the port lock to
  // cancel the call: the release ends the call, queued or in service.
  for (const bool received : {false, true}) {
    SCOPED_TRACE(received ? "in service" : "queued");
    IpcContext server;
    ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);
    int released = -1;
    StopWithAction stop;
    stop.action = [&] { released = server.Release(); };
    int64_t rc = 0;
    bool was_popped = !received;
    uintptr_t owner = kUnwrittenOwner;
    TaskThreads threads(*env_, [&] { server.Release(); });
    Task* const client = threads.Start([&] {
      rc = IpcCall(kPort,               //
                   /*client_id=*/51,    //
                   kIpcNoOwner,         //
                   kIpcNoOwner,         //
                   nullptr,             //
                   0,                   //
                   nullptr,             //
                   0,                   //
                   nullptr,             //
                   &was_popped,         //
                   &owner,              //
                   &RunActionThenStop,  //
                   &stop);
    });
    ASSERT_TRUE(WaitUntilBlocked(client));
    if (received) {
      IpcReceivedMessage msg{};
      ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
    }
    stop.armed.store(true, std::memory_order_release);
    env_->Wake(client);
    ASSERT_TRUE(threads.Join(client));
    EXPECT_EQ(released, 1);
    EXPECT_EQ(rc, kSyscallErrorNoDevice);
    EXPECT_EQ(was_popped, received);
    EXPECT_EQ(owner, server.OwnerToken());
    EXPECT_EQ(IpcPortOwner(kPort), kIpcNoOwner);
  }

  // 2. The client cancels its call before the server releases the port, which
  // then has no call to end and forgets the canceled one.
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);
  std::atomic<bool> stop{false};
  int64_t rc = 0;
  bool was_popped = false;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    rc = IpcCall(kPort,             //
                 /*client_id=*/52,  //
                 kIpcNoOwner,       //
                 kIpcNoOwner,       //
                 nullptr,           //
                 0,                 //
                 nullptr,           //
                 0,                 //
                 nullptr,           //
                 &was_popped,       //
                 nullptr,           //
                 &StopWhenFlagSet,  //
                 &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));
  IpcReceivedMessage msg{};
  ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
  stop.store(true, std::memory_order_release);
  env_->Wake(client);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(rc, kSyscallErrorInvalid);
  EXPECT_TRUE(was_popped);
  EXPECT_EQ(server.Release(), 1);
  // The next owner's tokens start at 1 again, like the canceled call's, but a
  // reply to that token fails instead of being dropped.
  IpcContext replacement;
  ASSERT_EQ(replacement.Listen(kPort, kUserCapNone), 0);
  EXPECT_EQ(replacement.Reply(kPort, msg.token, nullptr, 0, 0),
            kSyscallErrorInvalid);
}

TEST_F(IpcTest, RecvRacingAClientThatCancelsCancelsTheCallInService) {
  constexpr int kPort = 15;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);
  // The server receives the call when the client checks for a stop, after it
  // has found its call unanswered and before it takes the port lock to cancel
  // the call: the client cancels the call in service, so it reports the call
  // received (the server may have acted on it), and the port drops one late
  // reply to it.
  uint8_t srv_buf[4] = {};
  IpcReceivedMessage msg{};
  int64_t recv_rc = -1;
  StopWithAction stop;
  stop.action = [&] {
    recv_rc = server.Recv(kPort,            //
                          srv_buf,          //
                          sizeof(srv_buf),  //
                          &msg,             //
                          0,                //
                          nullptr,          //
                          nullptr);
  };
  int64_t rc = 0;
  bool was_popped = false;
  uintptr_t owner = kUnwrittenOwner;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    const uint8_t req = 0x42;
    rc = IpcCall(kPort,               //
                 /*client_id=*/61,    //
                 kIpcNoOwner,         //
                 kIpcNoOwner,         //
                 &req,                //
                 1,                   //
                 nullptr,             //
                 0,                   //
                 nullptr,             //
                 &was_popped,         //
                 &owner,              //
                 &RunActionThenStop,  //
                 &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));
  stop.armed.store(true, std::memory_order_release);
  env_->Wake(client);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(recv_rc, 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kCall);
  EXPECT_EQ(msg.client_id, 61u);
  EXPECT_EQ(msg.size, 1);
  EXPECT_EQ(srv_buf[0], 0x42);
  EXPECT_EQ(rc, kSyscallErrorInvalid);
  EXPECT_TRUE(was_popped);
  EXPECT_EQ(owner, server.OwnerToken());
  EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 0), 0);
  EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 0),
            kSyscallErrorInvalid);
}

TEST_F(IpcTest, RecvAndReplyAfterReleaseFailWithAccess) {
  constexpr int kPort = 12;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  int64_t call_rc = 0;
  int64_t blocking_recv_rc = 0;
  // Set only if the test fails, to end calls that `Release()` didn't.
  std::atomic<bool> stop{false};
  TaskThreads threads(*env_, [&] {
    stop.store(true, std::memory_order_release);
    server.Release();
  });
  Task* const client = threads.Start([&] {
    call_rc = IpcCall(kPort,             //
                      /*client_id=*/1,   //
                      kIpcNoOwner,       //
                      kIpcNoOwner,       //
                      nullptr,           //
                      0,                 //
                      nullptr,           //
                      0,                 //
                      nullptr,           //
                      nullptr,           //
                      nullptr,           //
                      &StopWhenFlagSet,  //
                      &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));
  IpcReceivedMessage msg{};
  ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
  const uint64_t token = msg.token;
  EXPECT_EQ(server.Release(), 1);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(call_rc, kSyscallErrorNoDevice);

  // The call that was in service ended with the port.
  EXPECT_EQ(server.Reply(kPort, token, nullptr, 0, 0), kSyscallErrorAccess);
  EXPECT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr),
            kSyscallErrorAccess);
  // A `Recv()` that would wait fails at once too.
  Task* const receiver = threads.Start([&] {
    IpcReceivedMessage blocking_msg{};
    blocking_recv_rc = server.Recv(kPort,             //
                                   nullptr,           //
                                   0,                 //
                                   &blocking_msg,     //
                                   -1,                //
                                   &StopWhenFlagSet,  //
                                   &stop);
  });
  ASSERT_TRUE(ReturnsWithoutBlocking(receiver));
  ASSERT_TRUE(threads.Join(receiver));
  EXPECT_EQ(blocking_recv_rc, kSyscallErrorAccess);

  // Also once another context owns the port, which doesn't know the token.
  IpcContext replacement;
  ASSERT_EQ(replacement.Listen(kPort, kUserCapNone), 0);
  EXPECT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr),
            kSyscallErrorAccess);
  EXPECT_EQ(server.Reply(kPort, token, nullptr, 0, 0), kSyscallErrorAccess);
  EXPECT_EQ(replacement.Reply(kPort, token, nullptr, 0, 0),
            kSyscallErrorInvalid);
}

TEST_F(IpcTest, TimedRecvReturnsACallThatComesBeforeTheDeadline) {
  constexpr int kPort = 11;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  uint8_t srv_buf[2] = {};
  IpcReceivedMessage msg{};
  int64_t recv_rc = -1;
  int64_t call_rc = -1;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const receiver = threads.Start([&] {
    recv_rc = server.Recv(kPort,                   //
                          srv_buf,                 //
                          sizeof(srv_buf),         //
                          &msg,                    //
                          /*timeout_ticks=*/1000,  //
                          nullptr,                 //
                          nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(receiver));
  Task* const client = threads.Start([&] {
    const uint8_t req[2] = {0x61, 0x62};
    call_rc = IpcCall(kPort,             //
                      /*client_id=*/61,  //
                      kIpcNoOwner,       //
                      kIpcNoOwner,       //
                      req,               //
                      sizeof(req),       //
                      nullptr,           //
                      0,                 //
                      nullptr,           //
                      nullptr,           //
                      nullptr,           //
                      nullptr,           //
                      nullptr);
  });

  // No tick passes, so only the call ends the wait, which it ends at once.
  ASSERT_TRUE(threads.Join(receiver));
  ASSERT_EQ(recv_rc, 1);
  EXPECT_EQ(BlockCount(receiver), 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kCall);
  EXPECT_EQ(msg.client_id, 61u);
  EXPECT_EQ(msg.size, 2);
  EXPECT_THAT(srv_buf, t::ElementsAre(0x61, 0x62));
  ASSERT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 3), 0);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(call_rc, 3);
}

TEST_F(IpcTest, TimedRecvReturnsEarlyOnceShouldStopHolds) {
  constexpr int kPort = 11;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  std::atomic<bool> stop{false};
  int64_t rc = -1;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const receiver = threads.Start([&] {
    IpcReceivedMessage msg{};
    rc = server.Recv(kPort,                   //
                     nullptr,                 //
                     0,                       //
                     &msg,                    //
                     /*timeout_ticks=*/1000,  //
                     &StopWhenFlagSet,        //
                     &stop);
  });
  ASSERT_TRUE(WaitUntilBlocked(receiver));
  // A wake while `should_stop()` is false only makes it wait again.
  env_->Wake(receiver);
  ASSERT_TRUE(WaitUntilBlocked(receiver, 2));

  // No tick passes, so its deadline never comes.
  stop.store(true, std::memory_order_release);
  env_->Wake(receiver);
  ASSERT_TRUE(threads.Join(receiver));
  EXPECT_EQ(rc, 0);
}

TEST_F(IpcTest, NonBlockingRecvReturnsEarlyOnceShouldStopHolds) {
  constexpr int kPort = 11;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);
  const uintptr_t server_owner = server.OwnerToken();

  int64_t client_rc = -1;
  bool client_was_popped = false;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    const uint8_t req = 0x2A;
    client_rc = IpcCall(kPort,               //
                        /*client_id=*/42,    //
                        kIpcNoOwner,         //
                        kIpcNoOwner,         //
                        &req,                //
                        1,                   //
                        nullptr,             //
                        0,                   //
                        nullptr,             //
                        &client_was_popped,  //
                        nullptr,             //
                        nullptr,             //
                        nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));
  IpcNotifyClientDisconnect(kPort, 99, server_owner);

  // With `should_stop` already true, even a non-blocking poll (`timeout_ticks
  // == 0`) returns 0 without popping either the disconnect or the queued call.
  std::atomic<bool> stop{true};
  uint8_t buf[8] = {};
  IpcReceivedMessage msg{};
  EXPECT_EQ(server.Recv(kPort,                //
                        buf,                  //
                        sizeof(buf),          //
                        &msg,                 //
                        /*timeout_ticks=*/0,  //
                        &StopWhenFlagSet,     //
                        &stop),
            0);

  // Clearing `stop` lets subsequent non-blocking polls receive both intact.
  stop.store(false, std::memory_order_release);
  ASSERT_EQ(server.Recv(kPort,                //
                        buf,                  //
                        sizeof(buf),          //
                        &msg,                 //
                        /*timeout_ticks=*/0,  //
                        &StopWhenFlagSet,     //
                        &stop),
            1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kClientDisconnect);
  EXPECT_EQ(msg.client_id, 99u);

  ASSERT_EQ(server.Recv(kPort,                //
                        buf,                  //
                        sizeof(buf),          //
                        &msg,                 //
                        /*timeout_ticks=*/0,  //
                        &StopWhenFlagSet,     //
                        &stop),
            1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kCall);
  EXPECT_EQ(msg.client_id, 42u);
  EXPECT_EQ(buf[0], 0x2A);
  EXPECT_EQ(server.Reply(kPort, msg.token, nullptr, 0, 7), 0);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(client_rc, 7);
  EXPECT_TRUE(client_was_popped);
}

TEST_F(IpcTest, ConcurrentReceiversWakeIndependentlyAndPassBaton) {
  constexpr int kPort = 13;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);

  // Phase 1: Two concurrent `Recv()` waiters both wake when two clients call.
  IpcReceivedMessage msg1{};
  IpcReceivedMessage msg2{};
  int64_t r1_rc = -1;
  int64_t r2_rc = -1;
  int64_t c1_rc = -1;
  int64_t c2_rc = -1;
  {
    TaskThreads threads(*env_, [&] { server.Release(); });
    Task* const r1 = threads.Start([&] {
      r1_rc = server.Recv(kPort, nullptr, 0, &msg1, -1, nullptr, nullptr);
    });
    Task* const r2 = threads.Start([&] {
      r2_rc = server.Recv(kPort, nullptr, 0, &msg2, -1, nullptr, nullptr);
    });
    ASSERT_TRUE(WaitUntilBlocked(r1));
    ASSERT_TRUE(WaitUntilBlocked(r2));

    Task* const c1 = threads.Start([&] {
      c1_rc = IpcCall(kPort,        //
                      501,          //
                      kIpcNoOwner,  //
                      kIpcNoOwner,  //
                      nullptr,      //
                      0,            //
                      nullptr,      //
                      0,            //
                      nullptr,      //
                      nullptr,      //
                      nullptr,      //
                      nullptr,      //
                      nullptr);
    });
    Task* const c2 = threads.Start([&] {
      c2_rc = IpcCall(kPort,        //
                      502,          //
                      kIpcNoOwner,  //
                      kIpcNoOwner,  //
                      nullptr,      //
                      0,            //
                      nullptr,      //
                      0,            //
                      nullptr,      //
                      nullptr,      //
                      nullptr,      //
                      nullptr,      //
                      nullptr);
    });
    ASSERT_TRUE(threads.Join(r1));
    ASSERT_TRUE(threads.Join(r2));
    ASSERT_EQ(r1_rc, 1);
    ASSERT_EQ(r2_rc, 1);
    EXPECT_THAT((std::set<uint64_t>{msg1.client_id, msg2.client_id}),
                t::ElementsAre(501u, 502u));
    EXPECT_EQ(server.Reply(kPort, msg1.token, nullptr, 0, 10), 0);
    EXPECT_EQ(server.Reply(kPort, msg2.token, nullptr, 0, 20), 0);
    ASSERT_TRUE(threads.Join(c1));
    ASSERT_TRUE(threads.Join(c2));
    EXPECT_THAT((std::set<int64_t>{c1_rc, c2_rc}), t::ElementsAre(10, 20));
  }

  // Phase 2: If the first waiter wakes after its `should_stop` became true, it
  // passes the wakeup baton to the second waiter so a pending call isn't lost.
  {
    std::atomic<bool> stop1{false};
    std::atomic<bool> stop2{false};
    IpcReceivedMessage baton_msg1{};
    IpcReceivedMessage baton_msg2{};
    int64_t baton_r1_rc = -1;
    int64_t baton_r2_rc = -1;
    int64_t baton_c_rc = -1;
    TaskThreads threads(*env_, [&] {
      stop1.store(true, std::memory_order_release);
      stop2.store(true, std::memory_order_release);
      server.Release();
    });
    Task* const r1 = threads.Start([&] {
      baton_r1_rc = server.Recv(kPort,             //
                                nullptr,           //
                                0,                 //
                                &baton_msg1,       //
                                -1,                //
                                &StopWhenFlagSet,  //
                                &stop1);
    });
    ASSERT_TRUE(WaitUntilBlocked(r1));
    Task* const r2 = threads.Start([&] {
      baton_r2_rc = server.Recv(kPort,             //
                                nullptr,           //
                                0,                 //
                                &baton_msg2,       //
                                -1,                //
                                &StopWhenFlagSet,  //
                                &stop2);
    });
    ASSERT_TRUE(WaitUntilBlocked(r2));

    // Mark `r1` stopped without waking it; the incoming call wakes `r1` first,
    // which exits with 0 and wakes `r2` to serve the call.
    stop1.store(true, std::memory_order_release);
    Task* const c = threads.Start([&] {
      baton_c_rc = IpcCall(kPort,        //
                           503,          //
                           kIpcNoOwner,  //
                           kIpcNoOwner,  //
                           nullptr,      //
                           0,            //
                           nullptr,      //
                           0,            //
                           nullptr,      //
                           nullptr,      //
                           nullptr,      //
                           nullptr,      //
                           nullptr);
    });
    ASSERT_TRUE(threads.Join(r1));
    ASSERT_TRUE(threads.Join(r2));
    EXPECT_EQ(baton_r1_rc, 0);
    ASSERT_EQ(baton_r2_rc, 1);
    EXPECT_EQ(baton_msg2.client_id, 503u);
    EXPECT_EQ(server.Reply(kPort, baton_msg2.token, nullptr, 0, 30), 0);
    ASSERT_TRUE(threads.Join(c));
    EXPECT_EQ(baton_c_rc, 30);
  }

  // Phase 3: `Release()` wakes every concurrent waiter on the port.
  {
    int64_t rel_r1_rc = 0;
    int64_t rel_r2_rc = 0;
    TaskThreads threads(*env_, [&] { server.Release(); });
    Task* const r1 = threads.Start([&] {
      IpcReceivedMessage m{};
      rel_r1_rc = server.Recv(kPort, nullptr, 0, &m, -1, nullptr, nullptr);
    });
    Task* const r2 = threads.Start([&] {
      IpcReceivedMessage m{};
      rel_r2_rc = server.Recv(kPort, nullptr, 0, &m, -1, nullptr, nullptr);
    });
    ASSERT_TRUE(WaitUntilBlocked(r1));
    ASSERT_TRUE(WaitUntilBlocked(r2));
    EXPECT_EQ(server.Release(), 1);
    ASSERT_TRUE(threads.JoinAll());
    EXPECT_EQ(rel_r1_rc, kSyscallErrorAccess);
    EXPECT_EQ(rel_r2_rc, kSyscallErrorAccess);
  }
}

TEST_F(IpcTest, ClientDisconnectPrioritizedAndCoalesced) {
  IpcContext server;
  ASSERT_EQ(server.Listen(kIpcPortFs, kUserCapDriver), 0);
  const uintptr_t server_owner = server.OwnerToken();

  int64_t rc = -1;
  TaskThreads threads(*env_, [&] { server.Release(); });
  Task* const client = threads.Start([&] {
    const uint8_t req = 0x55;
    rc = IpcCall(kIpcPortFs,         //
                 /*client_id=*/300,  //
                 kIpcNoOwner,        //
                 kIpcNoOwner,        //
                 &req,               //
                 1,                  //
                 nullptr,            //
                 0,                  //
                 nullptr,            //
                 nullptr,            //
                 nullptr,            //
                 nullptr,            //
                 nullptr);
  });
  ASSERT_TRUE(WaitUntilBlocked(client));

  // Post duplicate disconnects for client 100 and one for client 200 while a
  // `kCall` from client 300 is already queued.
  IpcNotifyClientDisconnect(kIpcPortFs, 100, server_owner);
  IpcNotifyClientDisconnect(kIpcPortFs, 100, server_owner);
  IpcNotifyClientDisconnect(kIpcPortFs, 200, server_owner);

  uint8_t buf[8] = {};
  IpcReceivedMessage msg{};
  ASSERT_EQ(
      server.Recv(kIpcPortFs, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kClientDisconnect);
  EXPECT_EQ(msg.client_id, 100u);
  EXPECT_EQ(msg.token, 0u);
  EXPECT_EQ(msg.size, 0);
  EXPECT_EQ(msg.reply_cap, 0);

  ASSERT_EQ(
      server.Recv(kIpcPortFs, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kClientDisconnect);
  EXPECT_EQ(msg.client_id, 200u);

  ASSERT_EQ(
      server.Recv(kIpcPortFs, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kCall);
  EXPECT_EQ(msg.client_id, 300u);
  ASSERT_EQ(server.Reply(kIpcPortFs, msg.token, nullptr, 0, 0), 0);
  ASSERT_TRUE(threads.Join(client));
  EXPECT_EQ(rc, 0);
  EXPECT_EQ(
      server.Recv(kIpcPortFs, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 0);

  // Only a pending notification absorbs a duplicate.
  IpcNotifyClientDisconnect(kIpcPortFs, 100, server_owner);
  ASSERT_EQ(
      server.Recv(kIpcPortFs, buf, sizeof(buf), &msg, 0, nullptr, nullptr), 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kClientDisconnect);
  EXPECT_EQ(msg.client_id, 100u);
}

TEST_F(IpcTest, NotifyClientDisconnectWithoutMatchingOwnerReturnsAtOnce) {
  IpcContext other;
  // Nobody owns the port: the notification is dropped.
  IpcNotifyClientDisconnect(kIpcFirstUserPort, 5, other.OwnerToken());
  // Invalid ports and `kIpcNoOwner` are ignored.
  IpcNotifyClientDisconnect(-1, 5, other.OwnerToken());
  IpcNotifyClientDisconnect(kIpcPortCount, 5, other.OwnerToken());
  IpcNotifyClientDisconnect(kIpcFirstUserPort, 5, kIpcNoOwner);
  EXPECT_EQ(BlockCount(main_task_), 0);

  // Client id 0 and a mismatched owner token are ignored on an owned port.
  IpcContext server;
  ASSERT_EQ(server.Listen(kIpcFirstUserPort, kUserCapNone), 0);
  IpcNotifyClientDisconnect(kIpcFirstUserPort, 0, server.OwnerToken());
  IpcNotifyClientDisconnect(kIpcFirstUserPort, 5, kIpcNoOwner);
  IpcNotifyClientDisconnect(kIpcFirstUserPort, 5, other.OwnerToken());
  EXPECT_EQ(BlockCount(main_task_), 0);
  uint8_t srv_buf[1] = {};
  IpcReceivedMessage msg{};
  EXPECT_EQ(server.Recv(kIpcFirstUserPort,  //
                        srv_buf,            //
                        sizeof(srv_buf),    //
                        &msg,               //
                        0,                  //
                        nullptr,            //
                        nullptr),
            0);
}

TEST_F(IpcTest, NotifyClientDisconnectWaitsForRoomWhileRingIsFull) {
  constexpr int kPort = 9;
  constexpr uint64_t kLateClient = kIpcDisconnectRingCapacity + 1;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);
  const uintptr_t server_owner = server.OwnerToken();
  for (int i = 1; i <= kIpcDisconnectRingCapacity; ++i) {
    IpcNotifyClientDisconnect(kPort, static_cast<uint64_t>(i), server_owner);
  }

  TaskThreads threads(*env_, [&] { server.Release(); });
  // A notification for a client whose notification is pending doesn't need
  // room.
  Task* const duplicate_notifier =
      threads.Start([&] { IpcNotifyClientDisconnect(kPort, 1, server_owner); });
  ASSERT_TRUE(ReturnsWithoutBlocking(duplicate_notifier));
  ASSERT_TRUE(threads.Join(duplicate_notifier));

  // A full-ring wait stops early once `should_stop` holds.
  std::atomic<bool> stop_waiter{false};
  Task* const stopped_notifier = threads.Start([&] {
    IpcNotifyClientDisconnect(kPort,             //
                              kLateClient + 1,   //
                              server_owner,      //
                              &StopWhenFlagSet,  //
                              &stop_waiter);
  });
  ASSERT_TRUE(WaitUntilBlocked(stopped_notifier));
  stop_waiter.store(true, std::memory_order_release);
  env_->AdvanceTicks(1);
  ASSERT_TRUE(threads.Join(stopped_notifier));

  // A new client's notification waits a tick at a time.
  Task* const notifier = threads.Start(
      [&] { IpcNotifyClientDisconnect(kPort, kLateClient, server_owner); });
  ASSERT_TRUE(WaitUntilBlocked(notifier));
  env_->AdvanceTicks(1);
  ASSERT_TRUE(WaitUntilBlocked(notifier, 2));

  // After the server receives a notification, the next tick lets it in.
  IpcReceivedMessage msg{};
  ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kClientDisconnect);
  EXPECT_EQ(msg.client_id, 1u);
  env_->AdvanceTicks(1);
  ASSERT_TRUE(threads.Join(notifier));

  for (int i = 2; i <= kIpcDisconnectRingCapacity; ++i) {
    ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
    ASSERT_EQ(msg.client_id, static_cast<uint64_t>(i));
  }
  ASSERT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 1);
  EXPECT_EQ(msg.kind, IpcMessageKind::kClientDisconnect);
  EXPECT_EQ(msg.client_id, kLateClient);
  EXPECT_EQ(server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 0);
}

TEST_F(IpcTest, NotifyClientDisconnectStopsWaitingWhenOwnerChangesOrReleases) {
  constexpr int kPort = 9;
  IpcContext server;
  ASSERT_EQ(server.Listen(kPort, kUserCapNone), 0);
  const uintptr_t server_owner = server.OwnerToken();
  for (int i = 1; i <= kIpcDisconnectRingCapacity; ++i) {
    IpcNotifyClientDisconnect(kPort, static_cast<uint64_t>(i), server_owner);
  }

  IpcContext next_server;
  TaskThreads threads(*env_, [&] {
    server.Release();
    next_server.Release();
  });
  Task* const notifier = threads.Start([&] {
    IpcNotifyClientDisconnect(kPort,                           //
                              kIpcDisconnectRingCapacity + 1,  //
                              server_owner);
  });
  ASSERT_TRUE(WaitUntilBlocked(notifier));

  // Even when a replacement server claims the port before the waiting notifier
  // wakes up, the stale notification for `server_owner` is dropped instead of
  // being delivered to `next_server`.
  EXPECT_EQ(server.Release(), 1);
  ASSERT_EQ(next_server.Listen(kPort, kUserCapNone), 0);
  env_->AdvanceTicks(1);
  ASSERT_TRUE(threads.Join(notifier));

  IpcReceivedMessage msg{};
  EXPECT_EQ(next_server.Recv(kPort, nullptr, 0, &msg, 0, nullptr, nullptr), 0);
}

TEST_F(IpcTest, MultiThreadedConcurrentCallsAndDisconnects) {
  IpcContext server;
  ASSERT_EQ(server.Listen(6, kUserCapNone), 0);
  const uintptr_t server_owner = server.OwnerToken();

  constexpr int kClientCount = 6;
  constexpr int kCallsPerClient = 40;
  int disconnects_seen = 0;
  int calls_served = 0;
  TaskThreads threads(*env_, [&] { server.Release(); });

  // The server serves calls until every client has disconnected; each client
  // disconnects after its last call returns.
  threads.Start([&] {
    uint64_t payload = 0;
    IpcReceivedMessage msg{};
    while (disconnects_seen < kClientCount) {
      const int64_t rc =
          server.Recv(6, &payload, sizeof(payload), &msg, -1, nullptr, nullptr);
      if (rc != 1) {
        ADD_FAILURE() << "Recv() returned " << rc;
        return;
      }
      if (msg.kind == IpcMessageKind::kClientDisconnect) {
        ++disconnects_seen;
        continue;
      }
      EXPECT_EQ(msg.size, static_cast<int64_t>(sizeof(payload)));
      const uint64_t out = payload ^ 0xF0F0F0F0F0F0F0F0ULL;
      const int64_t status = static_cast<int64_t>(payload & 0x7FFF);
      EXPECT_EQ(server.Reply(6, msg.token, &out, sizeof(out), status), 0);
      ++calls_served;
    }
  });
  for (int c = 0; c < kClientCount; ++c) {
    threads.Start([c, server_owner] {
      const uint64_t cid = static_cast<uint64_t>(1000 + c);
      for (int i = 0; i < kCallsPerClient; ++i) {
        const uint64_t req = (cid << 16) | static_cast<uint64_t>(i);
        uint64_t reply = 0;
        int64_t rsize = 0;
        const int64_t rc = IpcCall(6,              //
                                   cid,            //
                                   kIpcNoOwner,    //
                                   kIpcNoOwner,    //
                                   &req,           //
                                   sizeof(req),    //
                                   &reply,         //
                                   sizeof(reply),  //
                                   &rsize,         //
                                   nullptr,        //
                                   nullptr,        //
                                   nullptr,        //
                                   nullptr);
        EXPECT_EQ(rc, static_cast<int64_t>(req & 0x7FFF));
        EXPECT_EQ(rsize, static_cast<int64_t>(sizeof(reply)));
        EXPECT_EQ(reply, req ^ 0xF0F0F0F0F0F0F0F0ULL);
      }
      IpcNotifyClientDisconnect(6, cid, server_owner);
    });
  }
  ASSERT_TRUE(threads.JoinAll());
  EXPECT_EQ(disconnects_seen, kClientCount);
  EXPECT_EQ(calls_served, kClientCount * kCallsPerClient);
}

}  // namespace
}  // namespace protos
