#!/usr/bin/env bash
# Boots the kernel ISO(s) headlessly in QEMU and verifies the boot self-tests.
#
# Passes (as many at once as BOOT_TEST_JOBS says, by default one at a time on
# hosts with fewer than 48 CPUs):
#   1. bios: legacy BIOS + Multiboot2 on CPU model qemu64: serial markers plus
#      the VGA text buffer at 0xB8000.
#   2. uefi: UEFI (OVMF) + Multiboot2 + x2APIC + 4 GiB RAM on qemu64: serial
#      markers plus the GOP linear framebuffer.
#   3. mb1: legacy BIOS + Multiboot1 on qemu64: serial markers plus the VGA
#      text buffer (only when the Multiboot1 ISO exists).
#   4. max: legacy BIOS + Multiboot2 on CPU model max: serial markers plus the
#      VGA text buffer.
#   5. smep: legacy BIOS + Multiboot2 on CPU model qemu64,+smep,+xsave,+xgetbv1:
#      serial markers plus the VGA text buffer. (QEMU 7.2's TCG hangs on a
#      write of CR4.OSXSAVE unless CPUID.(EAX=0DH,ECX=1):EAX names a feature,
#      such as XGETBV with ECX=1, which the kernel doesn't use.)
# The CPU model decides how the kernel keeps the x87/SSE/AVX state of user
# processes, which its boot report names: qemu64 lacks XSAVE, so the kernel
# must fall back to FXSAVE; max has XSAVE and AVX, so the kernel must enable
# both, and the FPU isolation self-test must also cover the upper halves of the
# YMM registers; the smep model has XSAVE without AVX (and without XSAVEOPT),
# so the kernel must enable XSAVE with the x87 and SSE state only. The CPU
# model also decides whether the kernel enables SMEP and SMAP, which the boot
# report names too: qemu64 has neither, max has both, and the smep model has
# SMEP only. Besides the markers that each pass requires, the markers that
# would contradict its CPU model (a protection, state component or test
# coverage that the CPU lacks, or a probe that it should run but skipped) must
# be absent, and each boot report line must not contradict itself.
#
# Usage: test_boot.sh [kernel.iso [kernel-mb1.iso]]
#
# Environment:
#   BIOS_TIMEOUT_SECS  Timeout of each legacy BIOS pass (default: 60).
#   UEFI_TIMEOUT_SECS  Timeout of the UEFI pass (default: 90).
#   MIN_STRESS_PASSES  Stress-test iterations that must pass (default: 3).
#   BOOT_TEST_JOBS     How many passes run at once (default: one per 24 host
#                      CPUs, at least one; see PASS_JOBS below). Values above
#                      the number of passes run them all at once.
#   OVMF               Combined OVMF image (passed with -bios), or the
#                      OVMF_CODE image when OVMF_VARS is also set.
#   OVMF_VARS          OVMF_VARS template; selects pflash mode with OVMF.
#   REQUIRE_UEFI=1     Fail instead of skipping when no OVMF image is found.
#   KVM=1              Run QEMU with KVM acceleration: -enable-kvm -cpu host in
#                      every pass (-cpu host,+x2apic in the UEFI pass), so the
#                      checks that depend on the CPU model are skipped, and so
#                      is the smep pass, whose CPU model only TCG provides.
#                      Needs read and write access to /dev/kvm.
set -euo pipefail

ORIG_PWD="${PWD}"
cd "$(dirname "${BASH_SOURCE[0]}")"

BUILD_DIR="build"

abs_path() {
  if [[ "$1" == /* ]]; then
    printf '%s\n' "$1"
  else
    printf '%s\n' "${ORIG_PWD}/$1"
  fi
}

if [[ $# -ge 1 ]]; then
  ISO_FILE="$(abs_path "$1")"
else
  ISO_FILE="${BUILD_DIR}/kernel.iso"
fi
if [[ $# -ge 2 ]]; then
  MB1_ISO_FILE="$(abs_path "$2")"
else
  MB1_ISO_FILE="${BUILD_DIR}/kernel-mb1.iso"
fi

if [[ ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
  if [[ "${USE_DOCKER:-0}" == "1" ]] || ! command -v qemu-system-x86_64 >/dev/null 2>&1; then
    exec make run-boot-test USE_DOCKER=1
  fi
  if [[ $# -eq 0 && ( ! -f "${ISO_FILE}" || ! -f "${MB1_ISO_FILE}" ) ]]; then
    make "${ISO_FILE}" "${MB1_ISO_FILE}"
  fi
fi

BIOS_TIMEOUT_SECS="${BIOS_TIMEOUT_SECS:-60}"
UEFI_TIMEOUT_SECS="${UEFI_TIMEOUT_SECS:-90}"
MIN_STRESS_PASSES="${MIN_STRESS_PASSES:-3}"
KVM="${KVM:-0}"
# The vCPUs of the QEMU of each pass.
VCPUS_PER_PASS=4
# How many passes run at once. The tick bounds of the boot self-tests (such as
# a kill within 10 timer ticks of CPU 0) also count time in which the host
# runs none of a vCPU's threads, so the passes must not oversubscribe the host
# themselves: their vCPUs take at most a sixth of the host's CPUs. A host whose
# CPUs all run another busy thread then has at most 7 busy threads for every 6
# CPUs, where five 4-vCPU passes at once on 24 CPUs would make 44 for 24.
# BOOT_TEST_JOBS overrides it.
DEFAULT_PASS_JOBS=$(( $(nproc 2>/dev/null || echo 1) / (6 * VCPUS_PER_PASS) ))
if [[ "${DEFAULT_PASS_JOBS}" -lt 1 ]]; then
  DEFAULT_PASS_JOBS=1
fi
PASS_JOBS="${BOOT_TEST_JOBS:-${DEFAULT_PASS_JOBS}}"
if [[ ! "${PASS_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "FAIL: BOOT_TEST_JOBS must be a positive integer, not '${PASS_JOBS}'." >&2
  exit 1
fi
# Every value from the number of passes up runs them all at once. Clamp longer
# values by their digit count, before any arithmetic, which overflows from
# 2^63 on (2^64 would make the batch loop below endless).
if [[ "${#PASS_JOBS}" -gt 3 ]]; then
  PASS_JOBS=999
fi

EXPECTED_STRING="Hello, x86-64 Kernel World!"
COMPLETION_MARKER="[TEST] ALL BOOT SELF-TESTS PASSED"
STRESS_MARKER="[STRESS] random_multicpu_stress: PASS"
# Any test failure line contains "FAIL"; kernel panics (failed CHECKs and
# unhandled CPU exceptions) print "[FATAL]".
FAILURE_PATTERN='FAIL|\[FATAL\]'

REQUIRED_MARKERS=(
  "[PMM] mmap entry:"
  "[PMM] Total RAM:"
  "[SMP] Discovered CPUs: 4"
  "[SMP] CPU 0 (APIC ID 0, BSP): online"
  "[SMP] CPU 1 (APIC ID 1, AP): online"
  "[SMP] CPU 2 (APIC ID 2, AP): online"
  "[SMP] CPU 3 (APIC ID 3, AP): online"
  "[SMP] Online CPUs: 4/4"
  "[INIT] hello from ring 3"
  "[INIT] exited with status 0"
  "[TEST] pmm_memory_map_init: PASS"
  "[TEST] pmm_alloc_and_bounds: PASS"
  "[TEST] pmm_free_and_reuse: PASS"
  "[TEST] heap_varied_sizes_and_alignment: PASS"
  "[TEST] heap_pattern_isolation: PASS"
  "[TEST] cpp_new_delete_lifecycle: PASS"
  "[TEST] heap_stress_reuse: PASS"
  "[TEST] edge_cases_and_oom: PASS"
  "[TEST] pmm_heap_exhaustion: PASS"
  "[TEST] paging_edge_cases: PASS"
  "[TEST] rbtree_edge_cases: PASS"
  "[TEST] multiboot_mmap_edge_cases: PASS"
  "[TEST] smp_discovery_and_ap_bringup: PASS"
  "[TEST] smp_multicpu_sync_stress: PASS"
  "[TEST] idt_percpu_and_lapic_timer: PASS"
  "[TEST] task_scheduler_smp: PASS"
  "[TEST] tlb_shootdown: PASS"
  "[TEST] task_stack_guard_pages: PASS"
  "[TEST] address_spaces: PASS"
  "[TEST] address_space_switching: PASS"
  "[TEST] user_mode_programs: PASS"
  "[TEST] user_mode_preemption: PASS"
  "[TEST] user_mode_preemptible_syscalls: PASS"
  "[TEST] user_mode_kill: PASS"
  "[TEST] user_mode_fpu_isolation: PASS"
  "[TEST] user_mode_fpu_initial_state: PASS"
  "[TEST] cpu_protection_features: PASS"
  "[TEST] smap_smep_protection: PASS"
  "[TEST] user_mode_kill_edge_cases: PASS"
  "[TEST] user_mode_syscall_buffer_checks: PASS"
  "[TEST] user_mode_fpu_lifecycle: PASS"
  "[TEST] user_mode_mixed_workload: PASS"
  "[TEST] user_mode_fpu_isolation: covered x87/MMX SSE MXCSR FCW"
  "[TEST] user_mode_fpu_initial_state: reuse_rounds="
  "[TEST] user_mode_fpu_initial_state: exit_reuse_rounds="
  "[TEST] cpu_protection_features: cpus=4 "
  "[TEST] smap_smep_protection: not-present page fault recovered on 4/4 CPUs (error code 0x0)"
  "[TEST] user_mode_kill_edge_cases: cases=8 "
  "[TEST] user_mode_fpu_lifecycle: switched="
  "[TEST] user_mode_mixed_workload: tenants=12 short=16 "
  "${COMPLETION_MARKER}"
  "${STRESS_MARKER}"
)

# Markers that depend on the CPU model of a pass: the kernel's report of how it
# keeps the x87/SSE/AVX state, whether it resets the x87 pointers (it must on
# every pass: under TCG, QEMU 7.2's qemu64 and max models, on which all passes
# are based, are AMD CPUs without XSaveErPtr; Intel models such as Nehalem
# would report "disabled") and whether it enabled SMEP and SMAP, whether the
# FPU isolation self-test covered the upper halves of the YMM registers, and
# which SMAP and SMEP probes the protection self-tests ran and which they
# skipped. With KVM=1 every pass runs on the host CPU, whose features are
# unknown, so only the start of each report and probe line, which does not
# depend on the CPU, is required.
FPU_AVX_NOT_COVERED="[TEST] user_mode_fpu_isolation: AVX not covered (CPU has no AVX state)"
X87_POINTER_RESET="[CPU] x87 pointer reset: enabled"
SMAP_PROBES_SKIPPED="[TEST] smap_smep_protection: smap probes skipped (SMAP disabled)"
SMEP_PROBE_SKIPPED="[TEST] smap_smep_protection: smep probe skipped (SMEP disabled)"
SMEP_PROBE_RAN="[TEST] smap_smep_protection: smep exec probe recovered on 4/4 CPUs (error code 0x11)"
AC_WINDOW_SKIPPED="[TEST] smap_smep_protection: interrupt in user-access window skipped (SMAP disabled)"
NMI_WINDOW_SKIPPED="[TEST] smap_smep_protection: nmi in user-access window skipped (SMAP disabled)"
BUFFER_CHECKS_SMAP_OFF="[TEST] user_mode_syscall_buffer_checks: checks=13 smap=off"
# The boot report lines, each of which must not contradict itself in a pass.
REPORT_PREFIXES=("[CPU] FPU state:" "[CPU] x87 pointer reset:" "[CPU] SMEP:")
if [[ "${KVM}" == "1" ]]; then
  KVM_MARKERS=(
    "[CPU] FPU state: "
    "[CPU] x87 pointer reset: "
    "[CPU] SMEP: "
    "[TEST] smap_smep_protection: smap "
    "[TEST] smap_smep_protection: smep "
    "[TEST] smap_smep_protection: nmi in user-access window "
    "[TEST] user_mode_syscall_buffer_checks: checks=13 smap="
    "[TEST] user_mode_mixed_workload: hardening probes "
  )
  QEMU64_MARKERS=("${KVM_MARKERS[@]}")
  MAX_MARKERS=("${KVM_MARKERS[@]}")
  SMEP_MARKERS=("${KVM_MARKERS[@]}")
  # Which markers would contradict the host CPU is unknown.
  QEMU64_ABSENT_MARKERS=()
  MAX_ABSENT_MARKERS=()
  SMEP_ABSENT_MARKERS=()
else
  QEMU64_MARKERS=(
    "[CPU] FPU state: FXSAVE, 512 bytes"
    "${FPU_AVX_NOT_COVERED}"
    "${X87_POINTER_RESET}"
    "[CPU] SMEP: disabled, SMAP: disabled"
    "[TEST] cpu_protection_features: cpus=4 smep=off smap=off xcr0=none"
    "${SMAP_PROBES_SKIPPED}"
    "${SMEP_PROBE_SKIPPED}"
    "${AC_WINDOW_SKIPPED}"
    "${NMI_WINDOW_SKIPPED}"
    "${BUFFER_CHECKS_SMAP_OFF}"
    "[TEST] user_mode_mixed_workload: hardening probes skipped (no SMAP or SMEP)"
  )
  MAX_MARKERS=(
    "[CPU] FPU state: XSAVE, XCR0=0x7 (x87 SSE AVX)"
    "[TEST] user_mode_fpu_isolation: covered AVX YMM upper halves"
    "${X87_POINTER_RESET}"
    "[CPU] SMEP: enabled, SMAP: enabled"
    "[TEST] cpu_protection_features: cpus=4 smep=on smap=on xcr0=0x7"
    "[TEST] smap_smep_protection: smap read probe recovered on 4/4 CPUs (error code 0x1)"
    "[TEST] smap_smep_protection: smap write probe recovered on 4/4 CPUs (error code 0x3)"
    "${SMEP_PROBE_RAN}"
    "[TEST] smap_smep_protection: interrupt in user-access window checked"
    "[TEST] smap_smep_protection: nmi in user-access window checked"
    "[TEST] user_mode_syscall_buffer_checks: checks=13 smap=on"
    "[TEST] user_mode_mixed_workload: hardening probes smap=on smep=on recovered on 4/4 CPUs"
  )
  SMEP_MARKERS=(
    "[CPU] FPU state: XSAVE, XCR0=0x3 (x87 SSE)"
    "${FPU_AVX_NOT_COVERED}"
    "${X87_POINTER_RESET}"
    "[CPU] SMEP: enabled, SMAP: disabled"
    "[TEST] cpu_protection_features: cpus=4 smep=on smap=off xcr0=0x3"
    "${SMEP_PROBE_RAN}"
    "${SMAP_PROBES_SKIPPED}"
    "${AC_WINDOW_SKIPPED}"
    "${NMI_WINDOW_SKIPPED}"
    "${BUFFER_CHECKS_SMAP_OFF}"
    "[TEST] user_mode_mixed_workload: hardening probes smap=off smep=on recovered on 4/4 CPUs"
  )
  # Markers that must be absent because they contradict the CPU model: a
  # protection or state component that it lacks (or lacking one that it has),
  # coverage of the AVX state without AVX, a probe that ran without the
  # protection, or one skipped although the CPU has it. Fixed strings that
  # can't match inside the expected lines (bare "XSAVE" would match "FXSAVE").
  QEMU64_ABSENT_MARKERS=(
    "SMEP: enabled"
    "SMAP: enabled"
    "smep=on"
    "smap=on"
    "[CPU] FPU state: XSAVE"
    "[TEST] user_mode_fpu_isolation: covered AVX YMM upper halves"
    "smap read probe recovered"
    "smap write probe recovered"
    "smep exec probe recovered"
    "interrupt in user-access window checked"
    "nmi in user-access window checked"
    "[TEST] user_mode_mixed_workload: hardening probes smap="
  )
  MAX_ABSENT_MARKERS=(
    "SMEP: disabled"
    "SMAP: disabled"
    "smep=off"
    "smap=off"
    "[CPU] FPU state: FXSAVE"
    "[TEST] user_mode_fpu_isolation: AVX not covered"
    "smap probes skipped"
    "smep probe skipped"
    "interrupt in user-access window skipped"
    "nmi in user-access window skipped"
    "hardening probes skipped"
  )
  SMEP_ABSENT_MARKERS=(
    "SMEP: disabled"
    "SMAP: enabled"
    "smep=off"
    "smap=on"
    "[CPU] FPU state: FXSAVE"
    "[TEST] user_mode_fpu_isolation: covered AVX YMM upper halves"
    "smap read probe recovered"
    "smap write probe recovered"
    "smep probe skipped"
    "interrupt in user-access window checked"
    "nmi in user-access window checked"
    "hardening probes skipped"
  )
fi

PASS_NAMES=(bios uefi mb1 max smep)
TEMP_FILES=()
for name in "${PASS_NAMES[@]}"; do
  TEMP_FILES+=(
    "${BUILD_DIR}/serial_${name}_output.log"
    "${BUILD_DIR}/qemu_${name}_monitor.sock"
    "${BUILD_DIR}/vga_${name}_dump.bin"
    "${BUILD_DIR}/${name}_pass_stdout.log"
    "${BUILD_DIR}/${name}_pass_result"
  )
done
TEMP_FILES+=("${BUILD_DIR}/ovmf_vars.fd")

cleanup() {
  rm -f "${TEMP_FILES[@]}"
}
trap cleanup EXIT

if [[ ! -f "${ISO_FILE}" ]]; then
  echo "Error: ISO file '${ISO_FILE}' not found." >&2
  exit 1
fi
if [[ "${KVM}" == "1" ]]; then
  if [[ ! -r /dev/kvm || ! -w /dev/kvm ]]; then
    echo "Error: KVM=1 needs read and write access to /dev/kvm, which is missing or not accessible to this user (uid $(id -u)); enable KVM, join the group that owns /dev/kvm, or unset KVM." >&2
    exit 1
  fi
  echo "Using KVM acceleration (/dev/kvm)."
fi

mkdir -p "${BUILD_DIR}"
cleanup

# Prints the number of lines in file $1 that contain the fixed string $2.
count_matches() {
  grep -Fc -- "$2" "$1" 2>/dev/null || true
}

# Waits until the serial log $1 of QEMU process $2 shows a completed boot (the
# greeting, the self-test completion marker, and MIN_STRESS_PASSES stress
# passes), a failure line, QEMU exiting, or $3 seconds elapsing. Returns 0 only
# for a completed boot.
wait_for_boot() {
  local log="$1" pid="$2" timeout_secs="$3"
  local deadline=$((SECONDS + timeout_secs))
  while ((SECONDS < deadline)); do
    if [[ -f "${log}" ]]; then
      if grep -Eq -- "${FAILURE_PATTERN}" "${log}"; then
        return 1
      fi
      if grep -Fq -- "${EXPECTED_STRING}" "${log}" &&
         grep -Fq -- "${COMPLETION_MARKER}" "${log}" &&
         (($(count_matches "${log}" "${STRESS_MARKER}") >= MIN_STRESS_PASSES)); then
        return 0
      fi
    fi
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 1
    fi
    sleep 0.1
  done
  return 1
}

# Checks the serial log $1 of QEMU process $2 after wait_for_boot succeeded:
# every required marker (the global list plus any extra arguments) is present,
# there is no failure line, QEMU did not triple-fault, and the kernel did not
# reboot.
verify_serial_log() {
  local log="$1" pid="$2"
  shift 2
  local marker
  for marker in "${REQUIRED_MARKERS[@]}" "$@"; do
    if ! grep -Fq -- "${marker}" "${log}"; then
      echo "FAIL: Missing required marker '${marker}' in serial output." >&2
      return 1
    fi
  done

  # Give a late panic or triple fault a moment to show up.
  sleep 0.5
  if grep -Eq -- "${FAILURE_PATTERN}" "${log}"; then
    echo "FAIL: Found a failure line in serial output:" >&2
    grep -E -- "${FAILURE_PATTERN}" "${log}" >&2
    return 1
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "FAIL: QEMU exited unexpectedly (triple fault or shutdown)." >&2
    return 1
  fi

  local greeting_count
  greeting_count="$(count_matches "${log}" "${EXPECTED_STRING}")"
  if [[ "${greeting_count}" -ne 1 ]]; then
    echo "FAIL: Expected the greeting once, found it ${greeting_count} times (reboot loop)." >&2
    return 1
  fi
}

# Checks that the serial log $1 contains none of the markers $2...: lines that
# would contradict the CPU model of the pass. Reports the first one found on
# stderr (the pass output, not the serial log).
verify_absent_markers() {
  local log="$1"
  shift
  local marker
  for marker in "$@"; do
    if grep -Fq -- "${marker}" "${log}"; then
      echo "FAIL: Unexpected marker '${marker}' in serial output." >&2
      return 1
    fi
  done
}

# Checks that in the serial log $1 all lines that contain a boot report prefix
# $2... are the same, so that no report contradicts itself (for example a
# second report line that names other features).
verify_consistent_reports() {
  local log="$1"
  shift
  local prefix count
  for prefix in "$@"; do
    count="$(grep -F -- "${prefix}" "${log}" | sort -u | wc -l || true)"
    if [[ "${count}" -ne 1 ]]; then
      echo "FAIL: Expected one distinct '${prefix}' line in serial output, found ${count}." >&2
      return 1
    fi
  done
}

# Checks the serial log $2 of pass $1 for consistent boot reports and for the
# markers that the CPU model of the pass forbids (none with KVM=1).
verify_pass_absent_markers() {
  local name="$1" log="$2"
  local absent=()
  case "${name}" in
    bios | uefi | mb1) absent=(${QEMU64_ABSENT_MARKERS[@]+"${QEMU64_ABSENT_MARKERS[@]}"}) ;;
    max) absent=(${MAX_ABSENT_MARKERS[@]+"${MAX_ABSENT_MARKERS[@]}"}) ;;
    smep) absent=(${SMEP_ABSENT_MARKERS[@]+"${SMEP_ABSENT_MARKERS[@]}"}) ;;
  esac
  verify_consistent_reports "${log}" "${REPORT_PREFIXES[@]}" &&
    verify_absent_markers "${log}" ${absent[@]+"${absent[@]}"}
}

# Sends the QEMU monitor command $2 through the UNIX socket $1 and waits up to
# two seconds for the file $3 to reach $4 bytes. Requires python3.
monitor_dump() {
  local sock="$1" command="$2" out="$3" size="$4"
  python3 - "${sock}" "${command}" <<'EOF' || return 1
import socket
import sys
import time

s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sys.argv[1])
s.sendall(sys.argv[2].encode() + b"\n")
time.sleep(0.2)
s.close()
EOF
  local i
  for ((i = 0; i < 20; i++)); do
    if [[ -f "${out}" && "$(wc -c <"${out}")" -ge "${size}" ]]; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

# Verifies that VGA text row 0 holds the greeting in white-on-black (0x0F).
verify_vga_text() {
  local sock="$1" dump="$2"
  if ! command -v python3 >/dev/null 2>&1; then
    echo "WARNING: python3 not found; skipping the VGA text-buffer check."
    return 0
  fi
  local nbytes=$((${#EXPECTED_STRING} * 2))
  if ! monitor_dump "${sock}" "pmemsave 0xB8000 ${nbytes} ${dump}" "${dump}" "${nbytes}"; then
    echo "FAIL: Could not dump the VGA text buffer through the QEMU monitor." >&2
    return 1
  fi
  local vga_text
  vga_text="$(python3 - "${dump}" <<'EOF'
import sys

data = open(sys.argv[1], "rb").read()
chars = "".join(chr(data[i]) for i in range(0, len(data), 2))
attrs = [data[i] for i in range(1, len(data), 2)]
print(chars if all(a == 0x0F for a in attrs) else "<bad attributes: %r>" % attrs)
EOF
)"
  if [[ "${vga_text}" != "${EXPECTED_STRING}" ]]; then
    echo "FAIL: VGA row 0 is '${vga_text}', expected '${EXPECTED_STRING}' (attribute 0x0F)." >&2
    return 1
  fi
  echo "Verified VGA text buffer at 0xB8000 shows '${EXPECTED_STRING}' with attribute 0x0F."
}

# Verifies that the GOP framebuffer reported in serial log $1 has lit pixels in
# its top rows.
verify_framebuffer() {
  local log="$1" sock="$2" dump="$3"
  local fb_addr
  fb_addr="$(sed -n 's/.*\[PMM\] Framebuffer: addr=\(0x[0-9A-Fa-f]*\).*/\1/p' "${log}" | head -n 1)"
  if [[ -z "${fb_addr}" ]]; then
    echo "FAIL: No '[PMM] Framebuffer:' line in UEFI serial output." >&2
    return 1
  fi
  if ! command -v python3 >/dev/null 2>&1; then
    echo "WARNING: python3 not found; skipping the GOP framebuffer check."
    return 0
  fi
  local nbytes=38400
  if ! monitor_dump "${sock}" "pmemsave ${fb_addr} ${nbytes} ${dump}" "${dump}" "${nbytes}"; then
    echo "FAIL: Could not dump the GOP framebuffer through the QEMU monitor." >&2
    return 1
  fi
  local lit
  lit="$(python3 -c 'import sys; print(sum(1 for b in open(sys.argv[1], "rb").read() if b == 0xFF))' "${dump}")"
  if [[ "${lit}" -le 0 ]]; then
    echo "FAIL: GOP framebuffer at ${fb_addr} has no rendered glyph pixels." >&2
    return 1
  fi
  echo "Verified GOP framebuffer at ${fb_addr} has rendered glyph pixels (${lit} lit subpixels in the top rows)."
}

# Selects the OVMF firmware. Sets OVMF_ARGS, or returns 1 if none is found.
find_ovmf() {
  local code="" vars=""
  if [[ -n "${OVMF:-}" ]]; then
    code="${OVMF}"
    vars="${OVMF_VARS:-}"
  else
    local f
    for f in /usr/share/ovmf/OVMF.fd /usr/share/OVMF/OVMF.fd /usr/share/qemu/OVMF.fd \
             /usr/share/edk2/x64/OVMF.fd /usr/share/edk2-ovmf/x64/OVMF.fd; do
      if [[ -f "${f}" ]]; then
        code="${f}"
        break
      fi
    done
    if [[ -z "${code}" ]]; then
      local brew_prefix=""
      if command -v brew >/dev/null 2>&1; then
        brew_prefix="$(brew --prefix)"
      fi
      local pair
      for pair in /usr/share/OVMF/OVMF_CODE_4M.fd:/usr/share/OVMF/OVMF_VARS_4M.fd \
                  /usr/share/OVMF/OVMF_CODE.fd:/usr/share/OVMF/OVMF_VARS.fd \
                  /usr/share/edk2/ovmf/OVMF_CODE.fd:/usr/share/edk2/ovmf/OVMF_VARS.fd \
                  /usr/share/edk2/x64/OVMF_CODE.4m.fd:/usr/share/edk2/x64/OVMF_VARS.4m.fd \
                  /usr/share/edk2-ovmf/x64/OVMF_CODE.fd:/usr/share/edk2-ovmf/x64/OVMF_VARS.fd \
                  "${brew_prefix}/share/qemu/edk2-x86_64-code.fd:${brew_prefix}/share/qemu/edk2-i386-vars.fd"; do
        if [[ -f "${pair%%:*}" && -f "${pair##*:}" ]]; then
          code="${pair%%:*}"
          vars="${pair##*:}"
          break
        fi
      done
    fi
  fi
  if [[ -z "${code}" || ! -f "${code}" ]]; then
    return 1
  fi
  if [[ -z "${vars}" ]]; then
    OVMF_ARGS=(-bios "${code}")
  else
    cp "${vars}" "${BUILD_DIR}/ovmf_vars.fd"
    OVMF_ARGS=(
      -drive "if=pflash,format=raw,unit=0,readonly=on,file=${code}"
      -drive "if=pflash,format=raw,unit=1,file=${BUILD_DIR}/ovmf_vars.fd"
    )
  fi
}

# Runs one QEMU boot pass named $1 with timeout $2 seconds; the remaining
# arguments are passed to QEMU. Leaves QEMU running (PID in QEMU_PID) so the
# caller can inspect the live machine; the caller's RETURN trap kills it.
start_qemu() {
  local name="$1"
  shift
  qemu-system-x86_64 \
    -smp "${VCPUS_PER_PASS}" \
    -display none \
    -serial "file:${BUILD_DIR}/serial_${name}_output.log" \
    -monitor "unix:${BUILD_DIR}/qemu_${name}_monitor.sock,server,nowait" \
    -no-reboot \
    "$@" \
    >/dev/null 2>&1 &
  QEMU_PID=$!
}

kill_qemu() {
  if [[ -n "${QEMU_PID:-}" ]] && kill -0 "${QEMU_PID}" 2>/dev/null; then
    kill -9 "${QEMU_PID}" 2>/dev/null || true
    wait "${QEMU_PID}" 2>/dev/null || true
  fi
}

report_failure() {
  local name="$1" log="$2"
  echo "FAIL: ${name} boot did not complete within its timeout (needs '${EXPECTED_STRING}', '${COMPLETION_MARKER}' and ${MIN_STRESS_PASSES}x '${STRESS_MARKER}'), or printed a failure." >&2
  if [[ -f "${log}" ]]; then
    echo "Captured ${name} serial output:" >&2
    cat "${log}" >&2
  fi
}

# Legacy BIOS boot of ISO $2 on QEMU CPU model $4; pass name $1, display label
# $3. The remaining arguments are serial markers that only this pass requires.
run_bios_pass() {
  local name="$1" iso="$2" label="$3" cpu="$4"
  shift 4
  local log="${BUILD_DIR}/serial_${name}_output.log"
  local QEMU_PID=""
  trap kill_qemu RETURN

  local cpu_args=(-cpu "${cpu}")
  if [[ "${KVM}" == "1" ]]; then
    cpu_args=(-enable-kvm -cpu host)
  fi
  echo "Running ${label} boot verification..."
  start_qemu "${name}" "${cpu_args[@]}" -cdrom "${iso}"
  if ! wait_for_boot "${log}" "${QEMU_PID}" "${BIOS_TIMEOUT_SECS}"; then
    report_failure "${label}" "${log}"
    return 1
  fi
  if ! verify_serial_log "${log}" "${QEMU_PID}" "$@" ||
     ! verify_pass_absent_markers "${name}" "${log}" ||
     ! verify_vga_text "${BUILD_DIR}/qemu_${name}_monitor.sock" "${BUILD_DIR}/vga_${name}_dump.bin"; then
    cat "${log}" >&2
    return 1
  fi
  echo "Captured ${label} serial output:"
  cat "${log}"
}

# UEFI boot of ISO $1 with 4 GiB of RAM and x2APIC.
run_uefi_pass() {
  local iso="$1"
  local name="uefi" label="UEFI/Multiboot2 (OVMF, x2APIC, 4 GiB)"
  local log="${BUILD_DIR}/serial_${name}_output.log"
  local QEMU_PID=""
  trap kill_qemu RETURN

  local OVMF_ARGS=()
  if ! find_ovmf; then
    if [[ "${REQUIRE_UEFI:-0}" == "1" ]]; then
      echo "FAIL: No OVMF firmware found and REQUIRE_UEFI=1 (set OVMF=/path/to/OVMF.fd)." >&2
      return 1
    fi
    echo "SKIPPED: ${label} pass (no OVMF firmware found; set OVMF=/path/to/OVMF.fd)."
    echo "skipped" >"${BUILD_DIR}/${name}_pass_result"
    return 0
  fi

  local cpu_args=(-cpu "qemu64,+x2apic")
  if [[ "${KVM}" == "1" ]]; then
    cpu_args=(-enable-kvm -cpu "host,+x2apic")
  fi
  echo "Running ${label} boot verification..."
  start_qemu "${name}" "${OVMF_ARGS[@]}" "${cpu_args[@]}" -m 4G -cdrom "${iso}"
  if ! wait_for_boot "${log}" "${QEMU_PID}" "${UEFI_TIMEOUT_SECS}"; then
    report_failure "${label}" "${log}"
    return 1
  fi
  if ! verify_serial_log "${log}" "${QEMU_PID}" \
         "[PMM] mmap entry: base=0x100000000" "[PMM] Framebuffer: addr=" \
         "${QEMU64_MARKERS[@]}" ||
     ! verify_pass_absent_markers "${name}" "${log}" ||
     ! verify_framebuffer "${log}" "${BUILD_DIR}/qemu_${name}_monitor.sock" "${BUILD_DIR}/vga_${name}_dump.bin"; then
    cat "${log}" >&2
    return 1
  fi
  echo "Captured ${label} serial output:"
  cat "${log}"
}

run_mb1_pass() {
  local iso="$1"
  if [[ ! -f "${iso}" ]]; then
    echo "SKIPPED: BIOS/Multiboot1 pass (ISO '${iso}' not found)."
    echo "skipped" >"${BUILD_DIR}/mb1_pass_result"
    return 0
  fi
  run_bios_pass mb1 "${iso}" "BIOS/Multiboot1" qemu64 "${QEMU64_MARKERS[@]}"
}

# Legacy BIOS boot of ISO $1 on a CPU with SMEP and XSAVE but without SMAP,
# AVX and XSAVEOPT, a CPU model that only TCG provides.
run_smep_pass() {
  local iso="$1"
  # +xgetbv1 only because QEMU 7.2's TCG treats CR4.OSXSAVE as reserved (and
  # hangs on writing it) unless CPUID.(EAX=0DH,ECX=1):EAX is not zero.
  local cpu="qemu64,+smep,+xsave,+xgetbv1"
  local label="BIOS/Multiboot2 (-cpu ${cpu})"
  if [[ "${KVM}" == "1" ]]; then
    echo "SKIPPED: ${label} pass (TCG-only CPU model; KVM=1)."
    echo "skipped" >"${BUILD_DIR}/smep_pass_result"
    return 0
  fi
  run_bios_pass smep "${iso}" "${label}" "${cpu}" "${SMEP_MARKERS[@]}"
}

run_pass() {
  case "$1" in
    bios) run_bios_pass bios "${ISO_FILE}" "BIOS/Multiboot2" qemu64 "${QEMU64_MARKERS[@]}" ;;
    uefi) run_uefi_pass "${ISO_FILE}" ;;
    mb1) run_mb1_pass "${MB1_ISO_FILE}" ;;
    max) run_bios_pass max "${ISO_FILE}" "BIOS/Multiboot2 (-cpu max)" max "${MAX_MARKERS[@]}" ;;
    smep) run_smep_pass "${ISO_FILE}" ;;
  esac
}

# The passes run in batches of PASS_JOBS, each batch after the one before has
# ended; PASS_RCS[i] is the exit status of the job that ran PASS_NAMES[i].
PASS_RCS=()
batch_start=0
while [[ "${batch_start}" -lt "${#PASS_NAMES[@]}" ]]; do
  batch_end=$((batch_start + PASS_JOBS))
  if [[ "${batch_end}" -gt "${#PASS_NAMES[@]}" ]]; then
    batch_end="${#PASS_NAMES[@]}"
  fi
  batch_pids=()
  for ((i = batch_start; i < batch_end; ++i)); do
    name="${PASS_NAMES[i]}"
    run_pass "${name}" >"${BUILD_DIR}/${name}_pass_stdout.log" 2>&1 &
    batch_pids+=("$!")
  done
  for j in "${!batch_pids[@]}"; do
    rc=0
    wait "${batch_pids[j]}" || rc=$?
    PASS_RCS[batch_start + j]="${rc}"
  done
  batch_start="${batch_end}"
done

overall_rc=0
summary=()
for i in "${!PASS_NAMES[@]}"; do
  name="${PASS_NAMES[i]}"
  rc="${PASS_RCS[i]}"
  pass_stdout="${BUILD_DIR}/${name}_pass_stdout.log"
  cat "${pass_stdout}"
  # The pass output ends with the serial log, whose last line is cut off if
  # QEMU was stopped in the middle of it.
  if [[ -n "$(tail -c 1 "${pass_stdout}")" ]]; then
    echo
  fi
  if [[ "${rc}" -ne 0 ]]; then
    overall_rc=1
    summary+=("${name}: FAIL")
  elif [[ -f "${BUILD_DIR}/${name}_pass_result" ]]; then
    summary+=("${name}: SKIPPED")
  else
    summary+=("${name}: PASS")
  fi
done

if [[ "${overall_rc}" -ne 0 ]]; then
  echo "FAILURE: boot verification failed (${summary[*]})." >&2
  exit 1
fi
echo "SUCCESS: boot verification passed (${summary[*]}): greeting, all boot self-tests, ${MIN_STRESS_PASSES}+ stress-test passes, and no panic, triple fault, or reboot."
