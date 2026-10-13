#include "join_lifecycle_scheduler.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>
#include <utility>
#if __STDC_HOSTED__
#include <thread>
#endif

#include "check.h"
#include "heap.h"
#include "rbtree.h"
#include "smp.h"
#include "spinlock.h"

namespace protos {
namespace {

// Spin-wait hint for the busy-waits in this file. Host unit tests run each
// simulated CPU on its own host thread, so the hosted build also yields the
// host thread to let the thread that the wait depends on make progress.
static void CpuRelax() {
  asm volatile("pause" : : : "memory");
#if __STDC_HOSTED__
  std::this_thread::yield();
#endif
}

class JoinLifecycleTaskScheduler;

// Join/detach bookkeeping for one task created through this layer. See
// `JoinLifecycleTaskScheduler` for the lifecycle and the invariants.
struct TaskJoinRecord {
  JoinLifecycleTaskScheduler* owner = nullptr;
  TaskFn entry = nullptr;
  void* arg = nullptr;
  // CPU that ran the inner CreateTask*() call, or -1 if no CPU was bound.
  int creator_cpu_id = -1;
  // Set (release) once `task` is valid and the record is in `task_tree_`.
  std::atomic<bool> finalized{false};

  // The fields below are guarded by the owner's `join_lock_`.
  Task* task = nullptr;
  // The task waiting in Join(), if any.
  Task* joiner = nullptr;
  // The entry function has returned.
  bool exited = false;
  // Detach() has been called; ReapZombies() reclaims the record once exited.
  bool detached = false;
  bool in_tree = false;
  RbNode tree_node{};
  // Links in either `live_` or `zombies_`.
  TaskJoinRecord* prev = nullptr;
  TaskJoinRecord* next = nullptr;
};

struct TaskJoinRecordRbTraits {
  static uintptr_t GetKey(const TaskJoinRecord& rec) {
    return reinterpret_cast<uintptr_t>(rec.task);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }
};

using TaskJoinRecordTree =
    RbTree<TaskJoinRecord, &TaskJoinRecord::tree_node, TaskJoinRecordRbTraits>;

// Intrusive doubly-linked list of records, linked through `prev`/`next`.
class TaskJoinRecordList {
 public:
  void PushBack(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    DCHECK(rec->prev == nullptr && rec->next == nullptr && head_ != rec);
    rec->prev = tail_;
    if (tail_ != nullptr) {
      tail_->next = rec;
    } else {
      head_ = rec;
    }
    tail_ = rec;
  }

  void Remove(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    if (rec->prev != nullptr) {
      rec->prev->next = rec->next;
    } else {
      DCHECK(head_ == rec);
      head_ = rec->next;
    }
    if (rec->next != nullptr) {
      rec->next->prev = rec->prev;
    } else {
      DCHECK(tail_ == rec);
      tail_ = rec->prev;
    }
    rec->prev = nullptr;
    rec->next = nullptr;
  }

  // Empties the list and returns its former head. The returned records stay
  // chained through `next`, in insertion order.
  TaskJoinRecord* TakeAll() {
    TaskJoinRecord* const head = head_;
    head_ = nullptr;
    tail_ = nullptr;
    return head;
  }

 private:
  TaskJoinRecord* head_ = nullptr;
  TaskJoinRecord* tail_ = nullptr;
};

// Join lifecycle layer: tracks every task created through it, so that the task
// can be joined (Join()) or detached and later reclaimed (Detach() and
// ReapZombies()). The inner layers provide the primitive "wait until the task
// has fully exited, then free its TCB" (inner Join()); this layer decides when
// that is safe to call.
//
// Each task gets a TaskJoinRecord, which goes through these steps:
//
// 1. CreateTask*() allocates the record and links it into `live_`, then asks
//    the inner layer to create the task with TaskEntryWrapper() as its entry
//    function and the record as its argument. The record must exist first
//    because the task may start running, and even return, on another CPU
//    before the inner CreateTask*() call returns.
// 2. FinalizeJoinRecord() stores the handle returned by the inner layer,
//    inserts the record into `task_tree_` (keyed by handle, so that Join() and
//    Detach() find it in O(log n)) and publishes `finalized`.
// 3. TaskEntryWrapper() runs the user entry function. A task that starts on a
//    CPU other than its creator's first waits for `finalized`, so that its
//    handle can already be joined or detached when user code runs (user code
//    may pass CurrentTask() to another task). The creator keeps interrupts
//    disabled from just before the inner CreateTask*() call until
//    FinalizeJoinRecord() returns, which keeps that wait short: if the creator
//    could be preempted in between, the new task would spin for as long as
//    the creator stays descheduled, and if the creator were then queued on the
//    spinning task's CPU it might never run again (a livelock). A task never
//    waits on its creator's CPU: it can only start there after the creator
//    has re-enabled interrupts, i.e. after finalization, unless an inner layer
//    runs it synchronously inside CreateTask*(), where waiting would deadlock.
// 4. When the user entry function returns, TaskEntryWrapper() marks the record
//    `exited`. The task has not fully exited yet: the inner layer still has to
//    switch away from it and free its stack, which is why inner Join() may
//    still wait briefly after `exited` is set.
// 5. The record is reclaimed exactly once, by the owner of the handle:
//    - Join() waits for `exited`, removes the record from `task_tree_` and
//      `live_`, frees it and calls inner Join() to free the TCB.
//    - Detach() marks the record `detached`. Once it is both detached and
//      exited, it moves from `live_` to `zombies_`, and the next ReapZombies()
//      removes it from `task_tree_`, frees it and calls inner Join().
//    Join() rejects detached records and ReapZombies() only sees detached
//    ones, so a reaper can never free a task that its owner is about to join.
//
// Invariants (all guarded by `join_lock_`):
// - From allocation until reclamation a record is on exactly one of `live_`
//   and `zombies_`; it is on `zombies_` iff it is both detached and exited.
//   Keeping zombies on their own list makes ReapZombies() proportional to the
//   number of zombies.
// - `in_tree` holds from finalization until reclamation, so `task_tree_` maps
//   every unreclaimed handle to its record. A record leaves the tree before
//   inner Join() frees the TCB, so a TCB address reused by a new task never
//   finds a stale record.
// - `joiner` is non-null only while Join() waits. A record is never both
//   detached and joined.
class JoinLifecycleTaskScheduler final : public TaskScheduler {
 public:
  explicit JoinLifecycleTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {
    DCHECK(inner_ != nullptr);
  }

  ~JoinLifecycleTaskScheduler() override { ClearAllRecords(); }

  void Init() override {
    ClearAllRecords();
    inner_->Init();
  }

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    return CreateWithJoinRecord(entry, arg, [&](TaskJoinRecord* const rec) {
      return inner_->CreateTask(&TaskEntryWrapper, rec, weight);
    });
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    return CreateWithJoinRecord(entry, arg, [&](TaskJoinRecord* const rec) {
      return inner_->CreateTaskOnCpu(&TaskEntryWrapper,  //
                                     rec,                //
                                     weight,             //
                                     target_cpu);
    });
  }

  void Yield() override { inner_->Yield(); }

  void Join(Task* const task) override {
    DCHECK(task != nullptr);
    Task* const self = inner_->CurrentTask();
    DCHECK(self != nullptr);
    DCHECK(task != self);
    DCHECK(!HoldsAnyIrqSpinLock());

    TaskJoinRecord* rec = nullptr;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      rec = FindRecordLocked(task);
      DCHECK(!rec->detached);
      DCHECK(rec->joiner == nullptr);
      rec->joiner = self;
    }

    // Busy-wait until the entry function has returned. Yielding through the
    // inner layers lets the task run on this CPU, and lets this CPU steal it
    // if it is stuck in another CPU's runqueue.
    while (!IsRecordExited(rec)) {
      inner_->Yield();
      CpuRelax();
    }

    {
      const IrqSpinLockGuard join_guard(join_lock_);
      DCHECK(rec->exited);
      DCHECK(rec->joiner == self);
      EraseFromTreeLocked(rec);
      live_.Remove(rec);
    }
    FreeJoinRecord(rec);
    // Waits for the task to finish switching away (step 4), then frees it.
    inner_->Join(task);
  }

  void Detach(Task* const task) override {
    DCHECK(task != nullptr);
    const IrqSpinLockGuard join_guard(join_lock_);
    TaskJoinRecord* const rec = FindRecordLocked(task);
    DCHECK(!rec->detached);
    DCHECK(rec->joiner == nullptr);
    rec->detached = true;
    if (rec->exited) {
      MoveToZombiesLocked(rec);
    }
  }

  int ReapZombies() override {
    DCHECK(!HoldsAnyIrqSpinLock());
    TaskJoinRecord* zombies = nullptr;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      zombies = zombies_.TakeAll();
      for (TaskJoinRecord* rec = zombies; rec != nullptr; rec = rec->next) {
        EraseFromTreeLocked(rec);
      }
    }

    int count = 0;
    TaskJoinRecord* rec = zombies;
    while (rec != nullptr) {
      TaskJoinRecord* const next = rec->next;
      Task* const task = rec->task;
      FreeJoinRecord(rec);
      inner_->Join(task);
      ++count;
      rec = next;
    }
    return count;
  }

  bool PollIdleCpu(const int cpu_index) override {
    return inner_->PollIdleCpu(cpu_index);
  }

  int RunqueueLoad(const int cpu_index) const override {
    return inner_->RunqueueLoad(cpu_index);
  }

  bool StealTask(const int dst_cpu, const int src_cpu) override {
    return inner_->StealTask(dst_cpu, src_cpu);
  }

  void SetPreemptEnabled(const bool enabled) override {
    inner_->SetPreemptEnabled(enabled);
  }

  bool IsPreemptEnabled() const override { return inner_->IsPreemptEnabled(); }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
    inner_->OnTimerInterrupt(frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    inner_->OnWakeupIpi(frame);
  }

  Task* CurrentTask() const override { return inner_->CurrentTask(); }

 private:
  static int CurrentCpuIdOrUnbound() {
    const CpuLocal* const cpu = CurrentCpuOrNull();
    return (cpu != nullptr) ? cpu->cpu_id : -1;
  }

  // Steps 1 and 2 of the class comment. `create_inner(rec)` creates the task
  // in the inner layer with TaskEntryWrapper() and `rec` as entry and argument.
  template <typename CreateInnerFn>
  Task* CreateWithJoinRecord(const TaskFn entry,  //
                             void* const arg,     //
                             const CreateInnerFn& create_inner) {
    DCHECK(entry != nullptr);
    TaskJoinRecord* const rec = AllocateJoinRecord(entry, arg);
    if (rec == nullptr) {
      return nullptr;
    }
    // Interrupts stay disabled until the record is finalized (step 3).
    const bool saved_irq = SaveAndDisableInterrupts();
    rec->creator_cpu_id = CurrentCpuIdOrUnbound();
    Task* const task = create_inner(rec);
    FinalizeJoinRecord(rec, task);
    RestoreInterrupts(saved_irq);
    return task;
  }

  // Steps 3 and 4 of the class comment.
  static void TaskEntryWrapper(void* const raw_arg) {
    TaskJoinRecord* const rec = static_cast<TaskJoinRecord*>(raw_arg);
    DCHECK(rec != nullptr);
    DCHECK(rec->owner != nullptr);
    DCHECK(rec->entry != nullptr);
    if (CurrentCpuIdOrUnbound() != rec->creator_cpu_id) {
      while (!rec->finalized.load(std::memory_order_acquire)) {
        CpuRelax();
      }
    }
    rec->entry(rec->arg);

    DCHECK(!HoldsAnyIrqSpinLock());

    rec->owner->MarkRecordExited(rec);
  }

  void MarkRecordExited(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    const IrqSpinLockGuard join_guard(join_lock_);
    DCHECK(!rec->exited);
    rec->exited = true;
    if (rec->detached) {
      MoveToZombiesLocked(rec);
    }
  }

  TaskJoinRecord* AllocateJoinRecord(const TaskFn entry, void* const arg) {
    void* const raw = Kmalloc(sizeof(TaskJoinRecord));
    if (raw == nullptr) {
      return nullptr;
    }
    TaskJoinRecord* const rec = new (raw) TaskJoinRecord();
    rec->owner = this;
    rec->entry = entry;
    rec->arg = arg;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      live_.PushBack(rec);
    }
    return rec;
  }

  // Publishes `task` as the handle of `rec` (step 2), or frees `rec` if the
  // inner layer failed to create the task.
  void FinalizeJoinRecord(TaskJoinRecord* const rec, Task* const task) {
    DCHECK(rec != nullptr);
    if (task == nullptr) {
      {
        const IrqSpinLockGuard join_guard(join_lock_);
        live_.Remove(rec);
      }
      FreeJoinRecord(rec);
      return;
    }
    const IrqSpinLockGuard join_guard(join_lock_);
    rec->task = task;
    const bool inserted = task_tree_.Insert(rec);
    DCHECK(inserted);
    rec->in_tree = true;
    rec->finalized.store(true, std::memory_order_release);
  }

  // Returns the record of the unreclaimed handle `task`. Fails if `task` was
  // not created by this layer or has already been reclaimed.
  TaskJoinRecord* FindRecordLocked(Task* const task) {
    TaskJoinRecord* const rec =
        task_tree_.Find(reinterpret_cast<uintptr_t>(task));
    DCHECK(rec != nullptr);
    DCHECK(rec->in_tree && rec->task == task);
    return rec;
  }

  void MoveToZombiesLocked(TaskJoinRecord* const rec) {
    DCHECK(rec->detached && rec->exited);
    live_.Remove(rec);
    zombies_.PushBack(rec);
  }

  static void FreeJoinRecord(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    rec->~TaskJoinRecord();
    Kfree(rec);
  }

  void EraseFromTreeLocked(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    if (rec->in_tree) {
      task_tree_.Erase(rec);
      rec->in_tree = false;
    }
  }

  // Frees a chain of records returned by TaskJoinRecordList::TakeAll().
  static void FreeJoinRecordChain(TaskJoinRecord* rec) {
    while (rec != nullptr) {
      TaskJoinRecord* const next = rec->next;
      FreeJoinRecord(rec);
      rec = next;
    }
  }

  // Frees every record without calling inner Join(): used by Init() and the
  // destructor, when the inner layers are being reset or destroyed as well.
  void ClearAllRecords() {
    TaskJoinRecord* live = nullptr;
    TaskJoinRecord* zombies = nullptr;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      live = live_.TakeAll();
      zombies = zombies_.TakeAll();
      task_tree_.Clear();
    }
    FreeJoinRecordChain(live);
    FreeJoinRecordChain(zombies);
  }

  bool IsRecordExited(const TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    const IrqSpinLockGuard join_guard(join_lock_);
    return rec->exited;
  }

  const std::shared_ptr<TaskScheduler> inner_;
  IrqSpinLock join_lock_;
  TaskJoinRecordTree task_tree_;
  // Records that are not on `zombies_`.
  TaskJoinRecordList live_;
  // Records that are detached and exited, waiting for ReapZombies().
  TaskJoinRecordList zombies_;
};

}  // namespace

std::shared_ptr<TaskScheduler> CreateJoinLifecycleScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<JoinLifecycleTaskScheduler>(std::move(inner));
}

}  // namespace protos
