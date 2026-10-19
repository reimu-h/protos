#ifndef PROTOS_SELFTEST_FPU_LIFECYCLE_LAYOUT_H_
#define PROTOS_SELFTEST_FPU_LIFECYCLE_LAYOUT_H_

#include <cstdint>

#include "check.h"
#include "smp.h"

namespace protos {

// Returns the initial CPU (`0 <= cpu < cpu_count`) selected by
// SelectLeastLoadedCpu() when StartProcesses() creates `2 * cpu_count`
// processes sequentially from CPU 0 while the bootstrap task keeps CPU 0 busy.
inline int FpuLifecyclePlacedCpu(const int process_index, const int cpu_count) {
  DCHECK(process_index >= 0);
  DCHECK(cpu_count >= 1);
  return ((process_index % cpu_count) + 1) % cpu_count;
}

// Returns the number of spin() calls (2, 3, or 4) for kFpuLongSyscall process
// `process_index` of `batch` in RunFpuLongSyscallBatches() on `cpu_count` CPUs.
//
// Because StartProcesses() places process `k` and process `k + cpu_count` on
// the same CPU (`FpuLifecyclePlacedCpu()`), indexing `{2, 4, 3}` by
// `(process_index % cpu_count) + batch` assigns the same call count to both
// processes on a given CPU. On every multi-CPU configuration (`cpu_count >= 2`)
// and in every batch, at least one CPU finishes both of its shorter processes
// while another CPU still has a switched-out mid-syscall process queued, so an
// idle CPU steals a mid-syscall process deterministically.
inline int64_t FpuLifecycleRounds(const int process_index,  //
                                  const int cpu_count,      //
                                  const int batch = 0) {
  DCHECK(process_index >= 0);
  DCHECK(cpu_count >= 1);
  DCHECK(batch >= 0);
  constexpr int64_t kRounds[3] = {2, 4, 3};
  return kRounds[((process_index % cpu_count) + batch) % 3];
}

// Simulates one batch of `2 * cpu_count` kFpuLongSyscall processes started from
// CPU 0 (where the bootstrap task is running) with `FpuLifecycleRounds(i,
// cpu_count, batch)` spin() calls of `syscall_ticks` task ticks each, and
// returns the number of processes stolen after they have started a call.
inline int SimulateFpuLifecycleBatchMigrations(const int cpu_count,  //
                                               const int batch,      //
                                               const int64_t syscall_ticks) {
  DCHECK(cpu_count >= 1);
  DCHECK(cpu_count <= kMaxSupportedCpus);
  DCHECK(batch >= 0);
  DCHECK(syscall_ticks >= 1);

  constexpr int kMaxProcesses = 2 * kMaxSupportedCpus;
  const int process_count = 2 * cpu_count;

  int load[kMaxSupportedCpus] = {};
  load[0] = 1;  // The bootstrap task keeps CPU 0 busy during StartProcesses().
  int queues[kMaxSupportedCpus][kMaxProcesses] = {};
  int queue_len[kMaxSupportedCpus] = {};

  for (int i = 0; i < process_count; ++i) {
    int best_cpu = 0;
    for (int c = 1; c < cpu_count; ++c) {
      if (load[c] < load[best_cpu]) {
        best_cpu = c;
      }
    }
    queues[best_cpu][queue_len[best_cpu]++] = i;
    ++load[best_cpu];
  }

  int64_t need[kMaxProcesses] = {};
  int64_t got[kMaxProcesses] = {};
  for (int i = 0; i < process_count; ++i) {
    need[i] = syscall_ticks * FpuLifecycleRounds(i, cpu_count, batch);
  }

  int migrations = 0;
  for (;;) {
    bool any_remaining = false;
    for (int c = 0; c < cpu_count; ++c) {
      if (queue_len[c] > 0) {
        any_remaining = true;
        break;
      }
    }
    if (!any_remaining) {
      break;
    }

    // CPUs with no runnable kFpuLongSyscall process steal a queued
    // (non-running) process from the most loaded donor CPU that holds at least
    // 2 processes.
    for (int c = 0; c < cpu_count; ++c) {
      if (queue_len[c] != 0) {
        continue;
      }
      int donor = -1;
      for (int d = 0; d < cpu_count; ++d) {
        if (queue_len[d] >= 2 &&
            (donor < 0 || queue_len[d] > queue_len[donor])) {
          donor = d;
        }
      }
      if (donor >= 0) {
        const int victim = queues[donor][1];
        for (int k = 1; k + 1 < queue_len[donor]; ++k) {
          queues[donor][k] = queues[donor][k + 1];
        }
        --queue_len[donor];
        if (got[victim] > 0) {
          ++migrations;
        }
        queues[c][queue_len[c]++] = victim;
      }
    }

    // Advance one timer tick on each CPU: the running process (index 0) gets
    // one task tick and rotates to the back of its CPU's queue if not done.
    for (int c = 0; c < cpu_count; ++c) {
      if (queue_len[c] == 0) {
        continue;
      }
      const int proc = queues[c][0];
      for (int k = 0; k + 1 < queue_len[c]; ++k) {
        queues[c][k] = queues[c][k + 1];
      }
      --queue_len[c];
      ++got[proc];
      if (got[proc] < need[proc]) {
        queues[c][queue_len[c]++] = proc;
      }
    }
  }
  return migrations;
}

}  // namespace protos

#endif  // PROTOS_SELFTEST_FPU_LIFECYCLE_LAYOUT_H_
