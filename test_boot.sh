#!/usr/bin/env bash
# Boots the kernel ISO(s) headlessly in QEMU and verifies the boot self-tests.
#
# Passes (as many at once as BOOT_TEST_JOBS says, by default one at a time on
# hosts with fewer than 48 CPUs):
#   1. bios: legacy BIOS + Multiboot2 on CPU model qemu64: serial markers plus
#      the VGA text buffer at 0xB8000; with the USB devices of `make run` (an
#      xHCI controller with a USB keyboard), the USB keyboard scenarios.
#   2. uefi: UEFI (OVMF) + Multiboot2 + x2APIC + 4 GiB RAM on qemu64: serial
#      markers plus the GOP linear framebuffer; with those USB devices plus an
#      MSI-only xHCI controller behind a PCI bridge with a full-speed keyboard,
#      the USB keyboard scenarios too.
#   3. mb1: legacy BIOS + Multiboot1 on qemu64: serial markers plus the VGA
#      text buffer (only when the Multiboot1 ISO exists).
#   4. max: legacy BIOS + Multiboot2 on CPU model max: serial markers plus the
#      VGA text buffer.
#   5. smep: legacy BIOS + Multiboot2 on CPU model qemu64,+smep,+xsave,+xgetbv1:
#      serial markers plus the VGA text buffer. (QEMU 7.2's TCG hangs on a
#      write of CR4.OSXSAVE unless CPUID.(EAX=0DH,ECX=1):EAX names a feature,
#      such as XGETBV with ECX=1, which the kernel doesn't use.)
# The USB keyboard scenarios (BOOT_TEST_USB=0 skips them; see run_usb_session)
# ask the kernel for its interactive boot session, type into the shell through
# the USB keyboards with QMP key events, hot-plug and unplug keyboards, and
# check what the shell prints on the serial console, which they can type into
# too.
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
# be absent, and each boot report line must not contradict itself. Whether the
# kernel resets the x87 pointers, which the boot report names as well, must
# follow the vendor and XSAVEERPTR feature of the vCPUs, which QEMU reports
# over QMP in every pass (see verify_x87_pointer_reset).
#
# Usage: test_boot.sh [--passes LIST] [--artifact-dir DIR] [--drain-on-fail]
#                     [kernel.iso [kernel-mb1.iso]]
#
# Options (for deflake.py; make passes them only on from the Docker fallback
# below):
#   --passes LIST      Run only these passes (names from: bios uefi mb1 max
#                      smep, separated by spaces or commas), in the usual
#                      order, each with all of its checks (default: all).
#   --artifact-dir DIR Also save, for each pass that runs, its QEMU command
#                      line (qemu_argv, NUL-terminated arguments; qemu_cwd,
#                      the directory it runs in; and qemu_run_dir, the
#                      directory of the run's files that it names, which the
#                      script deletes at its end), its complete serial log
#                      (serial.log), QEMU's error output (qemu_stderr.log),
#                      its output (output.log) and its result (result: PASS,
#                      FAIL or SKIPPED) in <dir>/<pass>/.
#   --drain-on-fail    When the serial log of a pass shows a failure, keep
#                      QEMU running until the self-tests end, the kernel
#                      panics, QEMU exits or the timeout expires, so that the
#                      log also has the results of the later tests. The pass
#                      fails as before.
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
#   BOOT_TEST_USB=0    Skip the USB keyboard scenarios: the bios and uefi
#                      passes then run without USB devices, without the
#                      interactive boot session and with the serial output
#                      going to a file, like the other passes (default: 1).
#   QEMU_USB_FLAGS     The USB devices of the bios pass, which must be those of
#                      `make run` (the Makefile's QEMU_USB_FLAGS must default
#                      to the same; it passes a QEMU_USB_FLAGS that is set on
#                      into Docker): an xHCI controller with
#                      id=xhci0 and a usb-kbd with id=kbd0 on bus xhci0.0,
#                      whose ports 2 and 4 the scenarios hot-plug keyboards on
#                      (default: QEMU_USB_FLAGS_DEFAULT below). The uefi pass
#                      adds its second controller to them.
#   USB_SESSION_TIMEOUT_SECS  Time that the USB keyboard scenarios add to the
#                      timeout of the bios and uefi passes (default: 60).
#   KVM=1              Run QEMU with KVM acceleration: -enable-kvm -cpu host in
#                      every pass (-cpu host,+x2apic in the UEFI pass), so the
#                      checks that depend on the CPU model are skipped (except
#                      the x87 pointer reset check, which reads the vendor and
#                      XSAVEERPTR of the vCPUs from QEMU), and so is the smep
#                      pass, whose CPU model only TCG provides.
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

PASSES_SPEC=""
PASSES_SET=0
ARTIFACT_DIR=""
DRAIN_ON_FAIL=0
while [[ $# -gt 0 && "$1" == --* ]]; do
  case "$1" in
    --passes=*)
      PASSES_SPEC="${1#*=}"
      PASSES_SET=1
      ;;
    --passes)
      if [[ $# -lt 2 ]]; then
        echo "FAIL: --passes requires a pass list (passes: bios uefi mb1 max smep)." >&2
        exit 1
      fi
      PASSES_SPEC="$2"
      PASSES_SET=1
      shift
      ;;
    --artifact-dir=*)
      if [[ -z "${1#*=}" ]]; then
        echo "FAIL: --artifact-dir requires a non-empty directory path." >&2
        exit 1
      fi
      ARTIFACT_DIR="$(abs_path "${1#*=}")"
      ;;
    --artifact-dir)
      if [[ $# -lt 2 || -z "$2" ]]; then
        echo "FAIL: --artifact-dir requires a non-empty directory path." >&2
        exit 1
      fi
      ARTIFACT_DIR="$(abs_path "$2")"
      shift
      ;;
    --drain-on-fail)
      DRAIN_ON_FAIL=1
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "FAIL: Unknown option '$1' (options: --passes LIST, --artifact-dir DIR, --drain-on-fail)." >&2
      exit 1
      ;;
  esac
  shift
done

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

# Prints the path of file or directory $1 relative to this directory, the
# repository; fails if $1 is outside it (or the directory of a file $1 does
# not exist).
repo_relative_path() {
  local dir="$1" base=""
  if [[ ! -d "${dir}" ]]; then
    dir="$(dirname -- "$1")"
    base="/$(basename -- "$1")"
  fi
  local repo
  dir="$(CDPATH='' cd -- "${dir}" 2>/dev/null && pwd -P)" || return 1
  repo="$(pwd -P)"
  if [[ "${dir}" == "${repo}" ]]; then
    base="${base#/}"
    printf '%s\n' "${base:-.}"
  elif [[ "${dir}" == "${repo}/"* ]]; then
    printf '%s\n' "${dir#"${repo}/"}${base}"
  else
    return 1
  fi
}

# Prints its arguments as the value of a make variable that a recipe passes to
# the shell: each argument single-quoted for the shell, with $ doubled for
# make.
make_shell_words() {
  local word words=() quote="'\\''"
  for word in "$@"; do
    word="${word//\'/${quote}}"
    words+=("'${word//\$/\$\$}'")
  done
  printf '%s' "${words[*]-}"
}

# Outside a container, without QEMU on the host or with USE_DOCKER=1, the boot
# test runs in the Docker image: `make run-boot-test` runs this script there
# with the same options and ISO files (without ISO files, the ISOs of the
# working tree, which it builds first). The container sees only the
# repository, as its working directory, so the artifact directory and the ISO
# files must be in it (ISO files that don't exist are passed on as they are:
# the script in the container reports them like on the host).
if [[ ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
  if [[ "${USE_DOCKER:-0}" == "1" ]] || ! command -v qemu-system-x86_64 >/dev/null 2>&1; then
    docker_opts=()
    if [[ "${PASSES_SET}" -eq 1 ]]; then
      docker_opts+=("--passes=${PASSES_SPEC}")
    fi
    if [[ -n "${ARTIFACT_DIR}" ]]; then
      if ! mkdir -p "${ARTIFACT_DIR}" ||
         ! docker_path="$(repo_relative_path "${ARTIFACT_DIR}")"; then
        echo "FAIL: The boot test runs in Docker, which sees only the repository '${PWD}': --artifact-dir must be inside it, not '${ARTIFACT_DIR}'." >&2
        exit 1
      fi
      docker_opts+=("--artifact-dir=${docker_path}")
    fi
    if [[ "${DRAIN_ON_FAIL}" -eq 1 ]]; then
      docker_opts+=(--drain-on-fail)
    fi
    iso_args=()
    if [[ $# -ge 1 ]]; then
      iso_args+=("${ISO_FILE}")
    fi
    if [[ $# -ge 2 ]]; then
      iso_args+=("${MB1_ISO_FILE}")
    fi
    docker_isos=()
    for iso in ${iso_args[@]+"${iso_args[@]}"}; do
      if docker_path="$(repo_relative_path "${iso}")"; then
        docker_isos+=("${docker_path}")
      elif [[ -e "${iso}" ]]; then
        echo "FAIL: The boot test runs in Docker, which sees only the repository '${PWD}': the ISO files must be inside it, not '${iso}'." >&2
        exit 1
      else
        docker_isos+=("${iso}")
      fi
    done
    exec make run-boot-test USE_DOCKER=1 \
      "BOOT_TEST_OPTS=$(make_shell_words ${docker_opts[@]+"${docker_opts[@]}"})" \
      "BOOT_TEST_ISOS=$(make_shell_words ${docker_isos[@]+"${docker_isos[@]}"})"
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

# The USB keyboard scenarios of the bios and uefi passes (run_usb_session).
BOOT_TEST_USB="${BOOT_TEST_USB:-1}"
if [[ "${BOOT_TEST_USB}" != "0" && "${BOOT_TEST_USB}" != "1" ]]; then
  echo "FAIL: BOOT_TEST_USB must be 0 or 1, not '${BOOT_TEST_USB}'." >&2
  exit 1
fi
# The USB devices of `make run`, which the bios pass verifies: the default of
# QEMU_USB_FLAGS in the Makefile must be the same (deflake_test.py checks it).
QEMU_USB_FLAGS_DEFAULT="-device qemu-xhci,id=xhci0 -device usb-kbd,bus=xhci0.0,port=1,id=kbd0"
QEMU_USB_FLAGS="${QEMU_USB_FLAGS:-${QEMU_USB_FLAGS_DEFAULT}}"
USB_SESSION_TIMEOUT_SECS="${USB_SESSION_TIMEOUT_SECS:-60}"
QEMU_USB_ARGS=()
USB_FLAGS_ARE_DEFAULT=0
if [[ "${BOOT_TEST_USB}" == "1" ]]; then
  if [[ ! "${USB_SESSION_TIMEOUT_SECS}" =~ ^[1-9][0-9]{0,3}$ ]]; then
    echo "FAIL: USB_SESSION_TIMEOUT_SECS must be a number of seconds from 1 to 9999, not '${USB_SESSION_TIMEOUT_SECS}'." >&2
    exit 1
  fi
  read -r -a QEMU_USB_ARGS <<<"${QEMU_USB_FLAGS}"
  usb_has_xhci0=0
  usb_has_kbd0=0
  for usb_arg in ${QEMU_USB_ARGS[@]+"${QEMU_USB_ARGS[@]}"}; do
    if [[ ",${usb_arg}," == *",id=xhci0,"* ]]; then
      usb_has_xhci0=1
    fi
    if [[ ",${usb_arg}," == ",usb-kbd,"* && ",${usb_arg}," == *",bus=xhci0.0,"* &&
          ",${usb_arg}," == *",id=kbd0,"* ]]; then
      usb_has_kbd0=1
    fi
  done
  if [[ "${usb_has_xhci0}" != "1" || "${usb_has_kbd0}" != "1" ]]; then
    echo "FAIL: QEMU_USB_FLAGS must have an xHCI controller with id=xhci0 and a usb-kbd with bus=xhci0.0 and id=kbd0 (the USB keyboard scenarios use them; BOOT_TEST_USB=0 skips the scenarios), not '${QEMU_USB_FLAGS}'." >&2
    exit 1
  fi
  if [[ "${QEMU_USB_ARGS[*]}" == "${QEMU_USB_FLAGS_DEFAULT}" ]]; then
    USB_FLAGS_ARE_DEFAULT=1
  fi
fi

EXPECTED_STRING="Hello, x86-64 Kernel World!"
COMPLETION_MARKER="[TEST] ALL BOOT SELF-TESTS PASSED"
STRESS_MARKER="[STRESS] random_multicpu_stress: PASS"
# Any test failure line contains "FAIL"; kernel panics (failed CHECKs and
# unhandled CPU exceptions) print "[FATAL]".
FAILURE_PATTERN='FAIL|\[FATAL\]'
# The end of the self-tests (either result) or a kernel panic.
SUITE_END_OR_PANIC_PATTERN='\[TEST\] (ALL BOOT SELF-TESTS PASSED|BOOT SELF-TESTS FAILED)|\[FATAL\]'

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

# The QEMU arguments that only the USB passes add. The fw_cfg file asks the
# kernel for the interactive boot session (boot self-tests first, then an init
# session that waits for the harness to type 'exit 0'). The uefi pass adds an
# xHCI controller that has MSI but no MSI-X (nec-usb-xhci: QEMU 7.2's
# qemu-xhci can't turn MSI-X off) behind a PCI bridge, with a full-speed
# keyboard, the speed of most real keyboards.
USB_FW_CFG_ARGS=(-fw_cfg name=opt/protos/boot_session,string=interactive)
USB_MULTI_ARGS=(
  -device pci-bridge,id=br1,chassis_nr=1
  -device nec-usb-xhci,id=xhci1,bus=br1,addr=1,msi=on,msix=off
  -device usb-hub,bus=xhci1.0,port=2,id=hub0
  -device usb-kbd,bus=xhci1.0,port=2.1,id=hkbd0,usb_version=1
  -device usb-kbd,bus=xhci1.0,port=1,id=kbd1,usb_version=1
)
# Markers that the USB passes require besides REQUIRED_MARKERS: the driver's
# (which run_usb_session checks in detail) and the kernel's verdict that the
# driver released everything after the init session.
USB_REQUIRED_MARKERS=(
  "[USBKBD] controller "
  "[USBKBD] keyboard attached: "
  "[TEST] driver_session_cleanup: PASS"
)

# Markers that depend on the CPU model of a pass: the kernel's report of how it
# keeps the x87/SSE/AVX state and whether it enabled SMEP and SMAP, whether the
# FPU isolation self-test covered the upper halves of the YMM registers, and
# which SMAP and SMEP probes the protection self-tests ran and which they
# skipped. With KVM=1 every pass runs on the host CPU, whose features are
# unknown, so only the start of each report and probe line, which does not
# depend on the CPU, is required. Whether the kernel resets the x87 pointers
# depends on the vendor and XSAVEERPTR of the vCPUs, which can differ between
# QEMU versions for the same model (QEMU 7.2's max lacks XSAVEERPTR, 11.1's has
# it), so verify_x87_pointer_reset checks that report in every pass, with KVM=1
# too, against the vCPUs as QEMU reports them.
FPU_AVX_NOT_COVERED="[TEST] user_mode_fpu_isolation: AVX not covered (CPU has no AVX state)"
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
if [[ "${PASSES_SET}" -eq 1 ]]; then
  IFS=', ' read -r -a wanted_passes <<<"${PASSES_SPEC}"
  for wanted in ${wanted_passes[@]+"${wanted_passes[@]}"}; do
    if [[ -n "${wanted}" && " ${PASS_NAMES[*]} " != *" ${wanted} "* ]]; then
      echo "FAIL: --passes names an unknown pass '${wanted}' (passes: ${PASS_NAMES[*]})." >&2
      exit 1
    fi
  done
  selected_passes=()
  for name in "${PASS_NAMES[@]}"; do
    if [[ " ${wanted_passes[*]-} " == *" ${name} "* ]]; then
      selected_passes+=("${name}")
    fi
  done
  if [[ "${#selected_passes[@]}" -eq 0 ]]; then
    echo "FAIL: --passes='${PASSES_SPEC}' selects no pass (passes: ${PASS_NAMES[*]})." >&2
    exit 1
  fi
  PASS_NAMES=("${selected_passes[@]}")
  echo "Running only the passes that --passes selects: ${PASS_NAMES[*]}."
fi

# The files of this run: for each pass, QEMU's serial log, monitor, QMP and
# serial console sockets, error output and memory dump, and the pass's output
# and result. They go in a directory of the run's own under build/ (created
# below, deleted at exit), so that runs at the same time in one checkout (make
# test during a deflake.py session, for example) don't touch each other's
# files.
RUN_DIR=""
PASSES_STARTED=0
cleanup() {
  if [[ -n "${ARTIFACT_DIR}" && "${PASSES_STARTED}" -eq 1 ]]; then
    local name
    for name in "${PASS_NAMES[@]}"; do
      if [[ ! -f "${ARTIFACT_DIR}/${name}/result" ]]; then
        { mkdir -p "${ARTIFACT_DIR}/${name}" && copy_pass_logs "${name}"; } ||
          echo "WARNING: Could not save the artifacts of the ${name} pass in '${ARTIFACT_DIR}'." >&2
      fi
    done
  fi
  if [[ -n "${RUN_DIR}" ]]; then
    rm -rf -- "${RUN_DIR}"
  fi
}
trap cleanup EXIT
# A signal ends the run through the EXIT trap too. (Bash runs EXIT traps on
# fatal signals by itself, but as PID 1 of a container, where deflake.py stops
# it with SIGTERM, the script gets only the signals that it traps.)
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

# With --artifact-dir, copies the serial log, QEMU's error output and the
# output of pass $1 (those that exist so far) to its artifact directory.
copy_pass_logs() {
  local name="$1"
  local dir="${ARTIFACT_DIR}/${name}"
  { [[ ! -f "${RUN_DIR}/serial_${name}_output.log" ]] ||
    cp "${RUN_DIR}/serial_${name}_output.log" "${dir}/serial.log"; } &&
    { [[ ! -f "${RUN_DIR}/qemu_${name}_stderr.log" ]] ||
      cp "${RUN_DIR}/qemu_${name}_stderr.log" "${dir}/qemu_stderr.log"; } &&
    { [[ ! -f "${RUN_DIR}/${name}_pass_stdout.log" ]] ||
      cp "${RUN_DIR}/${name}_pass_stdout.log" "${dir}/output.log"; }
}

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
RUN_DIR="$(mktemp -d "${BUILD_DIR}/test_boot.XXXXXX")"

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
        drain_after_failure "${log}" "${pid}" "${deadline}"
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

# With --drain-on-fail, waits after a failure line in the serial log $1 of QEMU
# process $2 until the self-tests ended, the kernel panicked, QEMU exited or
# SECONDS reached $3, then one more second for trailing output, so that the log
# also has the results of the later tests. The pass fails either way; this only
# delays stopping QEMU.
drain_after_failure() {
  local log="$1" pid="$2" deadline="$3"
  if [[ "${DRAIN_ON_FAIL}" != "1" ]]; then
    return 0
  fi
  while ((SECONDS < deadline)) && kill -0 "${pid}" 2>/dev/null; do
    if grep -Eq -- "${SUITE_END_OR_PANIC_PATTERN}" "${log}"; then
      sleep 1
      return 0
    fi
    sleep 0.1
  done
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

# Prints "<xsaveerptr> <vendor>" of the vCPUs of the QEMU whose QMP socket is
# $1, for example "false AuthenticAMD": the QOM properties xsaveerptr and
# vendor of each vCPU, which are what its CPUID tells the guest. Fails, within
# 10 seconds, unless QEMU reports exactly $2 vCPUs and all of them agree.
# Requires python3.
qmp_vcpu_features() {
  python3 - "$1" "$2" <<'EOF'
import json
import socket
import sys
import time

sock_path, cpu_count = sys.argv[1], int(sys.argv[2])
deadline = time.monotonic() + 10
conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
received = b""
last_id = 0


def receive():
    """Returns the next QMP message (one JSON object per line)."""
    global received
    while b"\n" not in received:
        conn.settimeout(max(deadline - time.monotonic(), 0.001))
        data = conn.recv(65536)
        if not data:
            raise EOFError("QEMU closed the QMP connection")
        received += data
    line, received = received.split(b"\n", 1)
    return json.loads(line)


def execute(command, **arguments):
    """Runs QMP command `command` and returns its result, skipping events."""
    global last_id
    last_id += 1
    request = {"execute": command, "id": last_id}
    if arguments:
        request["arguments"] = arguments
    conn.sendall(json.dumps(request).encode() + b"\n")
    reply = receive()
    while "event" in reply:
        reply = receive()
    if reply.get("id") != last_id or "return" not in reply:
        raise RuntimeError("%s: unexpected reply %r" % (command, reply))
    return reply["return"]


try:
    conn.settimeout(10)
    conn.connect(sock_path)
    if "QMP" not in receive():
        raise RuntimeError("no QMP greeting")
    execute("qmp_capabilities")
    paths = sorted({cpu["qom-path"] for cpu in execute("query-cpus-fast")})
    if len(paths) != cpu_count:
        raise RuntimeError("expected %d vCPUs, found %r" % (cpu_count, paths))
    features = {}
    for path in paths:
        xsaveerptr = execute("qom-get", path=path, property="xsaveerptr")
        vendor = execute("qom-get", path=path, property="vendor")
        if (not isinstance(xsaveerptr, bool) or not isinstance(vendor, str) or
                not vendor.strip()):
            raise RuntimeError(
                "%s: xsaveerptr=%r vendor=%r" % (path, xsaveerptr, vendor))
        features[path] = (xsaveerptr, vendor)
    if len(set(features.values())) != 1:
        raise RuntimeError("the vCPUs differ: %r" % features)
except Exception as e:  # Whatever went wrong, the check must fail.
    print("QMP query of the vCPUs failed: %s" % e, file=sys.stderr)
    sys.exit(1)
xsaveerptr, vendor = features[paths[0]]
print("true" if xsaveerptr else "false", vendor)
EOF
}

# Checks the x87 pointer reset report in serial log $1 against the vCPUs of the
# QEMU whose QMP socket is $2. The kernel resets the x87 pointers exactly on AMD
# CPUs without XSAVEERPTR, which save and restore them only while an x87
# exception is pending (CpuChooseConfig in cpu.cpp), so the report must say
# "enabled" if the vCPUs are AuthenticAMD without XSAVEERPTR and "disabled"
# otherwise, and the opposite report must be absent. QEMU, not the kernel,
# tells which; the check fails, never skips, if QEMU can't tell.
verify_x87_pointer_reset() {
  local log="$1" sock="$2"
  if ! command -v python3 >/dev/null 2>&1; then
    echo "FAIL: python3 not found; the x87 pointer reset check needs it to ask QEMU for the vendor and XSAVEERPTR of the vCPUs." >&2
    return 1
  fi
  local features
  if ! features="$(qmp_vcpu_features "${sock}" "${VCPUS_PER_PASS}")"; then
    echo "FAIL: Could not read the vendor and XSAVEERPTR of the vCPUs through QMP, which decide the expected '[CPU] x87 pointer reset:' report." >&2
    return 1
  fi
  local xsaveerptr vendor
  read -r xsaveerptr vendor <<<"${features}"
  local expected="disabled" unexpected="enabled"
  if [[ "${vendor}" == "AuthenticAMD" && "${xsaveerptr}" == "false" ]]; then
    expected="enabled"
    unexpected="disabled"
  fi
  local cpus="QEMU reports ${VCPUS_PER_PASS} vCPUs ${vendor} with XSAVEERPTR=${xsaveerptr}"
  local reason="${cpus}, so the kernel must report '[CPU] x87 pointer reset: ${expected}' (it resets the x87 pointers exactly on AMD CPUs without XSAVEERPTR)"
  if grep -Fq -- "[CPU] x87 pointer reset: ${unexpected}" "${log}"; then
    echo "FAIL: Unexpected marker '[CPU] x87 pointer reset: ${unexpected}' in serial output: ${reason}." >&2
    return 1
  fi
  if ! grep -Fq -- "[CPU] x87 pointer reset: ${expected}" "${log}"; then
    echo "FAIL: Missing required marker '[CPU] x87 pointer reset: ${expected}' in serial output: ${reason}." >&2
    return 1
  fi
  echo "Verified '[CPU] x87 pointer reset: ${expected}' in serial output: ${cpus}."
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

# Selects the OVMF firmware. Sets OVMF_ARGS, or returns 1 if none is found (2
# after reporting an OVMF_VARS template that QEMU can't read).
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
    if [[ ! -f "${vars}" || ! -r "${vars}" ]]; then
      echo "FAIL: The OVMF_VARS template '${vars}' is not a readable file." >&2
      return 2
    fi
    # With snapshot=on, QEMU keeps the firmware's writes to its variable store
    # in a temporary file, so every boot starts from the template, and the
    # QEMU command line (which deflake.py's rerun.sh runs again) can name it.
    OVMF_ARGS=(
      -drive "if=pflash,format=raw,unit=0,readonly=on,file=${code}"
      -drive "if=pflash,format=raw,unit=1,snapshot=on,file=${vars}"
    )
  fi
}

# Starts the QEMU of the boot pass named $1; the remaining arguments are passed
# to QEMU. Leaves QEMU running (PID in QEMU_PID) so the caller can inspect the
# live machine; the caller's RETURN trap kills it. In a USB pass (the caller's
# USB_SESSION=1) the serial console is a socket that the harness can type into,
# which also logs its output to the same file. QEMU's error output goes to
# qemu_<pass>_stderr.log, which failure reports show.
start_qemu() {
  local name="$1"
  shift
  local serial_args=(-serial "file:${RUN_DIR}/serial_${name}_output.log")
  if [[ "${USB_SESSION:-0}" == "1" ]]; then
    serial_args=(
      -chardev "socket,id=ser0,path=${RUN_DIR}/qemu_${name}_serial.sock,server=on,wait=off,logfile=${RUN_DIR}/serial_${name}_output.log"
      -serial chardev:ser0
    )
  fi
  local qemu_argv=(
    qemu-system-x86_64
    -smp "${VCPUS_PER_PASS}"
    -display none
    "${serial_args[@]}"
    -monitor "unix:${RUN_DIR}/qemu_${name}_monitor.sock,server,nowait"
    -qmp "unix:${RUN_DIR}/qemu_${name}_qmp.sock,server,nowait"
    -no-reboot
    "$@"
  )
  save_qemu_command "${name}" "${qemu_argv[@]}"
  "${qemu_argv[@]}" >/dev/null 2>"${RUN_DIR}/qemu_${name}_stderr.log" &
  QEMU_PID=$!
}

# With --artifact-dir, saves the QEMU command line $2... of pass $1, the
# directory it runs in and the directory of the run's files.
save_qemu_command() {
  local name="$1"
  shift
  if [[ -z "${ARTIFACT_DIR}" ]]; then
    return 0
  fi
  { mkdir -p "${ARTIFACT_DIR}/${name}" &&
    printf '%s\0' "$@" >"${ARTIFACT_DIR}/${name}/qemu_argv" &&
    printf '%s\n' "${PWD}" >"${ARTIFACT_DIR}/${name}/qemu_cwd" &&
    printf '%s\n' "${RUN_DIR}" >"${ARTIFACT_DIR}/${name}/qemu_run_dir"; } ||
    echo "WARNING: Could not save the QEMU command line of the ${name} pass in '${ARTIFACT_DIR}'." >&2
}

# With --artifact-dir, saves the complete serial log, QEMU's error output, the
# output and the result $2 (PASS, FAIL or SKIPPED) of pass $1, whose QEMU has
# exited.
save_pass_artifacts() {
  local name="$1" result="$2"
  if [[ -z "${ARTIFACT_DIR}" ]]; then
    return 0
  fi
  local dir="${ARTIFACT_DIR}/${name}"
  { mkdir -p "${dir}" &&
    copy_pass_logs "${name}" &&
    printf '%s\n' "${result}" >"${dir}/result"; } ||
    echo "WARNING: Could not save the artifacts of the ${name} pass in '${ARTIFACT_DIR}'." >&2
}

kill_qemu() {
  if [[ -n "${QEMU_PID:-}" ]] && kill -0 "${QEMU_PID}" 2>/dev/null; then
    kill -9 "${QEMU_PID}" 2>/dev/null || true
    wait "${QEMU_PID}" 2>/dev/null || true
  fi
  # Once reaped, the PID may name another process.
  QEMU_PID=""
}

# Prints the error output of the QEMU of pass $1, if any, on stderr.
print_qemu_stderr() {
  local file="${RUN_DIR}/qemu_$1_stderr.log"
  if [[ -s "${file}" ]]; then
    echo "QEMU error output of the $1 pass:" >&2
    cat "${file}" >&2
  fi
}

# Reports that the boot of pass $3 (label $1, serial log $2) did not complete:
# that QEMU exited (a triple fault, a shutdown or a QEMU error, such as an
# option that it doesn't support), or that the boot timed out or printed a
# failure; then QEMU's error output and the serial log.
report_failure() {
  local name="$1" log="$2" pass="$3"
  if [[ -n "${QEMU_PID:-}" ]] && ! kill -0 "${QEMU_PID}" 2>/dev/null; then
    echo "FAIL: ${name}: QEMU exited unexpectedly before the boot completed (triple fault, shutdown or QEMU error)." >&2
  else
    echo "FAIL: ${name} boot did not complete within its timeout (needs '${EXPECTED_STRING}', '${COMPLETION_MARKER}' and ${MIN_STRESS_PASSES}x '${STRESS_MARKER}'), or printed a failure." >&2
  fi
  print_qemu_stderr "${pass}"
  if [[ -f "${log}" ]]; then
    echo "Captured ${name} serial output:" >&2
    cat "${log}" >&2
  fi
}

# Ends the QEMU of QEMU_PID after the checks of a completed boot and checks its
# complete serial log $1 once more. QEMU keeps running the stress test during
# the checks after verify_serial_log (seconds of QMP and monitor queries), so
# a failure or panic printed meanwhile, which the saved serial log shows, must
# still fail the pass.
stop_qemu_and_recheck_log() {
  local log="$1"
  if ! kill -0 "${QEMU_PID}" 2>/dev/null; then
    echo "FAIL: QEMU exited unexpectedly (triple fault or shutdown)." >&2
    return 1
  fi
  kill_qemu
  if grep -Eq -- "${FAILURE_PATTERN}" "${log}"; then
    echo "FAIL: Found a failure line in serial output:" >&2
    grep -E -- "${FAILURE_PATTERN}" "${log}" >&2
    return 1
  fi
}

# The USB keyboard scenarios of USB pass $1 (serial log $2; topology $3:
# "single" for the bios pass, with the USB devices of `make run`, or "multi"
# for the uefi pass, which adds an MSI-only controller behind a bridge),
# against the QEMU of QEMU_PID, within $4 seconds. Keys go only through QMP
# input-send-event to the newest USB keyboard (QEMU's routing), so a claim
# about one keyboard is made only while it is the only one; serial input goes
# through the serial console socket. Every line's random parts are generated
# right before it is typed, and the shell's output must match the edited line
# exactly. Order (the kernel's interactive boot session): the boot self-tests,
# the init session's scripted commands and the driver's markers, a serial
# echo command, the scenarios of the pass, then 'exit 0' typed over USB, after
# which init must exit with status 0 and the kernel report that the driver
# released everything ('[TEST] driver_session_cleanup: PASS').
# bios: a random token; Shift (both keys) with Backspace corrections; an empty
# line, Backspace on an empty line and Caps Lock (with Shift); a line typed
# partly over USB and partly into the serial console; the keyboard unplugged
# and keys sent with no USB keyboard (which must not reach the shell); a
# full-speed keyboard hot-plugged on the last root port (symbols, rollover),
# unplugged and attached again (Tab, spaces, digits); a second, high-speed
# keyboard; a line of exactly 128 characters plus two dropped ones; help, an
# unknown command and usertest 0.
# uefi: a random token and Shift with Backspace corrections on the full-speed
# keyboard of the MSI-only controller; that keyboard unplugged, then symbols
# and Caps Lock (Tab separator) on the root controller's only keyboard; serial
# input; the bridged keyboard attached again at high speed and the root one
# unplugged, then a 128-character line on it; a full-speed keyboard hot-plugged
# on another bridged port; help, an unknown command and usertest 0.
# Returns 0 if all of it passed; 2 if the serial log shows a failure line,
# which the caller reports like a failed boot; otherwise prints
# 'FAIL: usb_<scenario>: <detail>' (details on the lines after it) on stderr
# and returns 1. Requires python3.
run_usb_session() {
  if ! command -v python3 >/dev/null 2>&1; then
    echo "FAIL: usb_session: python3 not found; the USB keyboard scenarios need it to type through QMP." >&2
    return 1
  fi
  python3 - "$1" "$2" "${RUN_DIR}/qemu_$1_qmp.sock" "${RUN_DIR}/qemu_$1_serial.sock" \
    "${QEMU_PID}" "$4" "$3" "${USB_FLAGS_ARE_DEFAULT}" <<'USB_SESSION_PY'
import json
import os
import re
import secrets
import socket
import sys
import time
import traceback

(PASS_NAME, LOG_PATH, QMP_PATH, SERIAL_PATH, QEMU_PID, REMAINING, TOPOLOGY,
 DEFAULT_FLAGS) = sys.argv[1:9]
QEMU_PID = int(QEMU_PID)
T0 = time.monotonic()
DEADLINE = T0 + max(float(REMAINING), 1.0)
MULTI = TOPOLOGY == "multi"
# With the default QEMU_USB_FLAGS the root controller is a qemu-xhci (MSI-X)
# and its cold-plugged keyboard is high-speed on QEMU port 1. Other flags
# may differ there, so then only the bdf of those markers is checked.
EXACT_ROOT = DEFAULT_FLAGS == "1"

# Bounds of the waits, in seconds; every wait also ends at the pass deadline.
SESSION_READY_TIMEOUT = 30.0  # init session start -> scripted commands, markers
ECHO_TIMEOUT = 10.0  # the shell's echo of one key
OUTPUT_TIMEOUT = 15.0  # a command's output and the next prompt
MARKER_TIMEOUT = 15.0  # a keyboard attached/detached marker after hot-plug
EVENT_TIMEOUT = 10.0  # QEMU's DEVICE_DELETED event
NEGATIVE_WINDOW = 2.0  # keys typed without a USB keyboard must not arrive
KEY_GAP = 0.02  # between two input-send-event commands
POLL = 0.01
# Key events that may wait for the guest at once (QEMU's USB keyboard queues
# 16 and drops the rest): the harness waits for the shell's echo after every
# key that types a character.
MAX_UNPOLLED = 12
LINE_MAX = 128  # the shell's line limit (kShellMaxLineLength in user/sh/sh_lib.h)
ECHO_MAX_TOKENS = 16  # the shell's token limit (kShellMaxTokens)

COMPLETION = "[TEST] ALL BOOT SELF-TESTS PASSED"
INIT_HELLO = "[INIT] hello from ring 3"
INIT_EXITED = "[INIT] exited with status"
INIT_EXITED_OK = "[INIT] exited with status 0"
CLEANUP_PASS = "[TEST] driver_session_cleanup: PASS"
NO_CONTROLLERS = "[USBKBD] no xHCI controllers found"
STRESS = "[STRESS]"
PROMPT = "sh> "
HELP_LINES = ("built-in commands: help echo cpu programs run exit",
              "user programs: init sh usertest")
SCRIPTED_ECHO = "boot_sh_ok"
USERTEST_RE = re.compile(r"sh: usertest exited with status -?[0-9]+")
FAILURE_RE = re.compile(r"FAIL|\[FATAL\]")
BDF = r"[0-9a-f]{2}:[0-9a-f]{2}\.[0-7]"
PORT_PATH = r"[0-9]+(?:\.[0-9]+)*"
READY_RE = re.compile(r"\[USBKBD\] controller (%s) ready \((msix|msi)\)$" % BDF)
ATTACHED_RE = re.compile(r"\[USBKBD\] keyboard attached: (%s) port (%s) "
                         r"\((ls|fs|hs|ss)\)$" % (BDF, PORT_PATH))
DETACHED_RE = re.compile(r"\[USBKBD\] keyboard detached: (%s) port (%s)$" %
                         (BDF, PORT_PATH))
# A kernel or driver log line. Typed text never has "[" before a letter.
LOG_LINE_RE = re.compile(r"\[[A-Z][A-Za-z0-9_]*\][^\n]*\n")
# Text that typed lines must not contain: what test_boot.sh and deflake.py
# read as failures, test results, tags, seeds, CPU reports or the greeting,
# and the harness's own sync words.
FORBIDDEN_RE = re.compile(
    r"FAIL|FATAL|PASS|TEST|BOOT|INIT|STRESS|USBKBD|Hello|XSAVE|SMEP|SMAP|"
    r"smep|smap|seed=|sh>|ser_|boot_sh_ok|\[[A-Za-z_]")

LETTERS = "abcdefghijklmnopqrstuvwxyz"
DIGITS = "0123456789"
ALNUM = LETTERS + DIGITS
UPPER_1 = "ABCDEFGHIJKLM"
UPPER_2 = "NOPQRSTUVWXYZ"
SYMBOLS_1 = "~!@#$%^&*()"  # shifted
SYMBOLS_2 = "_+{}|:\"<>?"  # shifted
PLAIN_SYMBOLS = "`-=[]\\;',./"  # unshifted
# The US layout: QMP qcode -> (character, character with Shift).
KEYMAP = {d: (d, s) for d, s in zip(DIGITS, ")!@#$%^&*(")}
KEYMAP.update({
    "grave_accent": ("`", "~"), "minus": ("-", "_"), "equal": ("=", "+"),
    "bracket_left": ("[", "{"), "bracket_right": ("]", "}"),
    "backslash": ("\\", "|"), "semicolon": (";", ":"),
    "apostrophe": ("'", "\""), "comma": (",", "<"), "dot": (".", ">"),
    "slash": ("/", "?"), "spc": (" ", " "), "tab": ("\t", "\t"),
    "ret": ("\r", "\r"), "backspace": ("\b", "\b"),
})
MODIFIERS = ("shift", "shift_r")
CHAR_KEY = {}  # character -> (qcode, needs Shift)
for _q, (_plain, _shifted) in KEYMAP.items():
    CHAR_KEY.setdefault(_plain, (_q, False))
    CHAR_KEY.setdefault(_shifted, (_q, True))
for _c in LETTERS:
    CHAR_KEY[_c] = (_c, False)
    CHAR_KEY[_c.upper()] = (_c, True)
RNG = secrets.SystemRandom()


class Failure(Exception):
    """A scenario failed: reported as 'FAIL: <scenario>: <detail>'."""

    def __init__(self, scenario, detail, info=()):
        Exception.__init__(self, detail)
        self.scenario, self.detail, self.info = scenario, detail, list(info)


class SerialFailureLine(Exception):
    """The serial log shows a failure line (test_boot.sh reports it)."""


def say(message):
    print(message, flush=True)


def rand(n, alphabet=ALNUM):
    return "".join(secrets.choice(alphabet) for _ in range(n))


def shuffled(text):
    chars = list(text)
    RNG.shuffle(chars)
    return "".join(chars)


def tap(qcode, mod=None):
    """One key pressed and released (with modifier `mod` held), one chunk."""
    events = [(qcode, True), (qcode, False)]
    if mod:
        events = [(mod, True)] + events + [(mod, False)]
    return [events]


def keys(text, shift="shift"):
    """The keys that type `text`, each in its own chunk; characters that need
    Shift get the modifier `shift` around their key."""
    chunks = []
    for c in text:
        qcode, shifted = CHAR_KEY[c]
        chunks += tap(qcode, shift if shifted else None)
    return chunks


def held(mod, text):
    """The keys of the characters of `text` with modifier `mod` held down
    over all of them."""
    chunks = [[(q, True), (q, False)] for q in (CHAR_KEY[c][0] for c in text)]
    chunks[0].insert(0, (mod, True))
    chunks[-1].append((mod, False))
    return chunks


def rollover(a, b):
    """Key b pressed before key a is released."""
    return [[(a, True), (b, True), (a, False), (b, False)]]


CAPS = tap("caps_lock")
ENTER = tap("ret")
BACKSPACE = tap("backspace")


class KeyboardModel:
    """What a keyboard driver with the US layout delivers for key events: a
    character on every press of a character key, the shifted character or
    other letter case while either Shift key is held, and Caps Lock toggling
    the case of letters only (Shift with Caps Lock gives lower case)."""

    def __init__(self, other=None):
        self.held = set(other.held) if other else set()
        self.caps = other.caps if other else False

    def feed(self, qcode, down):
        if qcode in MODIFIERS:
            if down:
                self.held.add(qcode)
            else:
                self.held.discard(qcode)
            return ""
        if qcode == "caps_lock":
            if down:
                self.caps = not self.caps
            return ""
        if not down:
            return ""
        shift = bool(self.held)
        if qcode in LETTERS:
            return qcode.upper() if shift != self.caps else qcode
        return KEYMAP[qcode][1 if shift else 0]


class ShellModel:
    """The line editor of sh (user/sh/sh_lib.h): printable characters are echoed
    up to the line limit and later ones dropped silently, Tab becomes a space,
    Backspace (0x08 or 0x7F) erases one character with "\\b \\b" (nothing on an
    empty line), CR or LF submits the line (LF right after CR is ignored), and
    the serial console turns the echoed LF into CR LF."""

    def __init__(self, other=None):
        self.buf = list(other.buf) if other else []
        self.last_cr = other.last_cr if other else False

    def feed(self, ch):
        """Returns (echo, the submitted line or None)."""
        if ch == "\n" and self.last_cr:
            self.last_cr = False
            return "", None
        self.last_cr = ch == "\r"
        if ch in "\r\n":
            line = "".join(self.buf)
            self.buf = []
            return "\r\n", line
        if ch in "\b\x7f":
            if self.buf:
                self.buf.pop()
                return "\b \b", None
            return "", None
        if ch == "\t":
            ch = " "
        if " " <= ch <= "~" and len(self.buf) < LINE_MAX:
            self.buf.append(ch)
            return ch, None
        return "", None


def echo_output(line):
    """What sh prints for `line` if it is an echo command, else None."""
    tokens = [t for t in line.split(" ") if t]
    if not tokens or tokens[0] != "echo":
        return None
    return " ".join(tokens[1:ECHO_MAX_TOKENS])


class Plan:
    """Key event chunks run through the models: per chunk, the shell's echo
    and how many of its events follow the last one that the echo proves the
    guest has read (None: no echo); the characters delivered and the lines
    submitted."""

    def __init__(self, chunks, kbd, shell):
        self.chunks = chunks
        self.echoes, self.pending, self.lines, typed = [], [], [], []
        for chunk in chunks:
            echo, last = [], -1
            for k, (qcode, down) in enumerate(chunk):
                for ch in kbd.feed(qcode, down):
                    if ch not in "\b\r\n":
                        typed.append(" " if ch == "\t" else ch)
                    text, line = shell.feed(ch)
                    if text:
                        echo.append(text)
                        last = k
                    if line is not None:
                        self.lines.append(line)
            self.echoes.append("".join(echo))
            self.pending.append(len(chunk) - 1 - last if last >= 0 else None)
        self.typed = "".join(typed)
        self.events = sum(len(c) for c in chunks)


def describe(chunks):
    """A short, readable form of key events for failure reports."""
    names = {"spc": " ", "tab": "<tab>", "ret": "<ret>", "backspace": "<bs>",
             "caps_lock": "<caps>"}
    out = []
    for chunk in chunks:
        for qcode, down in chunk:
            if qcode in MODIFIERS:
                out.append("<%s%s>" % ("" if down else "/", qcode))
            elif down:
                out.append(names.get(qcode) or KEYMAP.get(qcode, (qcode,))[0])
    return "".join(out)


def acceptable(*texts):
    return not any(FORBIDDEN_RE.search(t) for t in texts)


def complete_lines(text, start):
    """The complete lines of `text` after offset `start`, without CR LF."""
    segment = text[start:]
    end = segment.rfind("\n")
    if end < 0:
        return []
    return [l[:-1] if l.endswith("\r") else l for l in segment[:end].split("\n")]


def qemu_alive():
    try:
        with open("/proc/%d/stat" % QEMU_PID) as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except OSError:
        pass
    try:
        os.kill(QEMU_PID, 0)
        return True
    except OSError:
        return False


def connect_unix(path, timeout):
    end = min(time.monotonic() + timeout, DEADLINE)
    while True:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(path)
            return sock
        except OSError:
            sock.close()
            if time.monotonic() >= end:
                raise
            time.sleep(0.05)


class QmpError(Exception):
    pass


class Qmp:
    """A QMP client that keeps the events it reads (QEMU may send an event
    before or after the reply to the command that caused it)."""

    def __init__(self, path):
        self.sock = connect_unix(path, 10.0)
        self.buf = b""
        self.events = []
        self.last_id = 0
        if "QMP" not in self.recv(10.0):
            raise QmpError("no QMP greeting")
        self.execute("qmp_capabilities")

    def recv(self, timeout):
        end = min(time.monotonic() + timeout, DEADLINE)
        while b"\n" not in self.buf:
            remaining = end - time.monotonic()
            if remaining <= 0:
                raise socket.timeout("no QMP message within %g s" % timeout)
            self.sock.settimeout(remaining)
            data = self.sock.recv(65536)
            if not data:
                raise QmpError("QEMU closed the QMP connection")
            self.buf += data
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8")) if line.strip() else {}

    def execute(self, command, **arguments):
        self.last_id += 1
        request = {"execute": command, "id": self.last_id}
        if arguments:
            request["arguments"] = arguments
        self.sock.sendall(json.dumps(request).encode() + b"\n")
        while True:
            reply = self.recv(10.0)
            if "event" in reply:
                self.events.append(reply)
            elif reply.get("id") == self.last_id:
                if "error" in reply:
                    raise QmpError("%s: %s" % (command, reply["error"].get(
                        "desc", reply["error"])))
                return reply.get("return")

    def wait_event(self, matches, timeout):
        end = min(time.monotonic() + timeout, DEADLINE)
        while True:
            for i, event in enumerate(self.events):
                if matches(event):
                    return self.events.pop(i)
            remaining = end - time.monotonic()
            if remaining <= 0:
                return None
            try:
                message = self.recv(remaining)
            except socket.timeout:
                return None
            if "event" in message:
                self.events.append(message)

    def close(self):
        self.sock.close()


class Session:
    """The USB scenarios of one pass, against the QEMU of that pass."""

    def __init__(self):
        self.log = ""
        self.failure_scan = 0
        self.session_scan = 0
        self.scenario = "usb_session"
        self.s0 = None  # where the init session's part of the log starts
        self.session_live = False
        self.exit_typed = False
        self.qmp = None
        self.serial_sock = None
        self.pressed = set()
        self.next_key = 0.0
        self.kbd = KeyboardModel()
        self.shell = ShellModel()
        self.cursor = 0
        self.bdf = {}
        self.keyboards = []  # attached USB keyboards (QMP ids), newest last
        self.ports = {}  # QMP id -> (bdf, xHCI port)
        self.hubs = {}  # QMP hub id -> (controller, QEMU port)
        self.initial_markers = []
        self.hotplug_markers = []
        self.journal = []  # everything typed into the serial console
        self.usb_outputs = []  # outputs of the lines typed only over USB
        self.lines_typed = 0

    # -- The serial log ----------------------------------------------------

    def text(self):
        try:
            with open(LOG_PATH, "rb") as f:
                f.seek(len(self.log))
                data = f.read()
        except OSError:
            data = b""
        if data:
            self.log += data.decode("latin-1")
        return self.log

    def check(self):
        """Fails on what ends the scenarios early: a failure line in the
        serial log, the init session ending or the stress test starting
        before the harness typed 'exit 0', the driver finding no controller,
        or QEMU exiting."""
        t = self.text()
        if FAILURE_RE.search(t, max(0, self.failure_scan - 6)):
            raise SerialFailureLine()
        self.failure_scan = len(t)
        if self.session_live and not self.exit_typed:
            start = max(self.s0, self.session_scan - 40)
            if t.find(INIT_EXITED, start) >= 0:
                raise Failure("usb_session", "the init session ended before "
                              "the harness typed 'exit 0'")
            if t.find(STRESS, start) >= 0:
                raise Failure("usb_session", "the stress test started while "
                              "the init session should still run")
            if t.find(NO_CONTROLLERS, start) >= 0:
                raise Failure("usb_ready", "the driver reported that it "
                              "found no xHCI controller")
            self.session_scan = len(t)
        if not qemu_alive():
            raise Failure(self.scenario, "QEMU exited unexpectedly")

    def idle(self):
        if self.serial_sock is not None:
            try:
                while self.serial_sock.recv(65536, socket.MSG_DONTWAIT):
                    pass
            except OSError:
                pass
        time.sleep(POLL)

    def wait_for(self, what, timeout, predicate):
        """Waits until predicate(log) returns a value other than None and
        returns it."""
        end = min(time.monotonic() + timeout, DEADLINE)
        while True:
            self.check()
            result = predicate(self.log)
            if result is not None:
                return result
            if time.monotonic() >= end:
                raise Failure(self.scenario, "timed out waiting for " + what,
                              ["(%s)" % self.timeout_reason(timeout)])
            self.idle()

    def timeout_reason(self, timeout):
        if time.monotonic() >= DEADLINE:
            return "the pass deadline expired"
        return "after %g s" % timeout

    def log_line(self, line):
        """A kernel or driver log line in the middle of shell output: an
        unexpected keyboard or controller marker fails, others are noted."""
        s = line.rstrip("\r\n")
        if (READY_RE.search(s) or ATTACHED_RE.search(s) or
                DETACHED_RE.search(s) or NO_CONTROLLERS in s):
            raise Failure(self.scenario, "unexpected driver marker in the "
                          "serial log", ["line: %r" % s])
        say("USB note: log line during the session: %r" % s)

    def consume(self, cursor, expected, timeout, what):
        """Waits until the serial log continues at `cursor` with exactly
        `expected` (complete kernel or driver log lines may come in between)
        and returns the offset after it."""
        end = min(time.monotonic() + timeout, DEADLINE)
        i, j = cursor, 0
        while True:
            t = self.text()
            while j < len(expected) and i < len(t):
                if t[i] == "[":
                    if i + 1 < len(t) and t[i + 1].isupper():
                        nl = t.find("\n", i)
                        if nl < 0:
                            break
                        self.log_line(t[i:nl + 1])
                        i = nl + 1
                        continue
                    if i + 1 >= len(t) and expected[j] != "[":
                        break  # Maybe the start of a log line.
                if t[i] != expected[j]:
                    raise Failure(self.scenario, "unexpected serial output "
                                  "instead of " + what, [
                                      "expected: %r" % expected[j:j + 120],
                                      "found:    %r" % t[i:i + 120],
                                      "after:    %r" % t[max(0, i - 80):i]])
                i += 1
                j += 1
            if j >= len(expected):
                return i
            self.check()
            if time.monotonic() >= end:
                raise Failure(self.scenario, "timed out waiting for " + what, [
                    "(%s)" % self.timeout_reason(timeout),
                    "expected: %r" % expected[j:j + 120],
                    "log after the cursor: %r" % self.log[cursor:][-240:]])
            self.idle()

    def expect_marker(self, cursor, pattern, timeout, what):
        """Waits for a driver marker line that matches `pattern` at `cursor`
        (complete kernel log lines may come first) and returns the offset
        after it and the match."""
        end = min(time.monotonic() + timeout, DEADLINE)
        i = cursor
        while True:
            t = self.text()
            while True:
                nl = t.find("\n", i)
                if nl < 0:
                    break
                line = t[i:nl + 1]
                match = pattern.fullmatch(line.rstrip("\r\n"))
                if match:
                    return nl + 1, match
                if not LOG_LINE_RE.fullmatch(line):
                    break
                self.log_line(line)
                i = nl + 1
            if i < len(t) and t[i] != "[":
                raise Failure(self.scenario, "unexpected serial output "
                              "instead of " + what, ["found: %r" % t[i:i + 120]])
            self.check()
            if time.monotonic() >= end:
                raise Failure(self.scenario, "timed out waiting for " + what, [
                    "(%s)" % self.timeout_reason(timeout),
                    "log after the cursor: %r" % self.log[cursor:][-240:]])
            self.idle()

    # -- Input ---------------------------------------------------------------

    def send_keys(self, chunk):
        delay = self.next_key - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        try:
            self.qmp.execute("input-send-event", events=[
                {"type": "key", "data": {
                    "down": down, "key": {"type": "qcode", "data": qcode}}}
                for qcode, down in chunk])
        except (QmpError, OSError) as e:
            raise Failure(self.scenario, "QMP input-send-event failed",
                          [str(e)])
        for qcode, down in chunk:
            if down:
                self.pressed.add(qcode)
            else:
                self.pressed.discard(qcode)
        self.next_key = time.monotonic() + KEY_GAP

    def type_plan(self, plan, what):
        """Types the chunks of `plan` over USB, one input-send-event command
        per chunk, and waits for the shell's echo of each chunk that has one
        (at the cursor)."""
        unpolled = 0
        for k, chunk in enumerate(plan.chunks):
            if unpolled + len(chunk) > MAX_UNPOLLED:
                raise RuntimeError("too many key events without an echo")
            self.send_keys(chunk)
            if plan.echoes[k]:
                self.cursor = self.consume(
                    self.cursor, plan.echoes[k], ECHO_TIMEOUT,
                    "the shell's echo of key %d of %s" % (k + 1, what))
                unpolled = plan.pending[k]
            else:
                unpolled += len(chunk)

    def plan(self, build):
        """Builds the keys of a line with fresh random parts (until neither
        the typed characters nor the edited line nor the expected output have
        reserved text, and the output is not in the log yet) and checks the
        builder's expected output against the models."""
        for _ in range(200):
            chunks, expected = build()
            kbd, shell = KeyboardModel(self.kbd), ShellModel(self.shell)
            plan = Plan(chunks, kbd, shell)
            if len(plan.lines) != 1:
                raise RuntimeError("a planned line must end with one Enter")
            output = echo_output(plan.lines[0])
            if output != expected:
                raise RuntimeError("the models give %r for a line that should "
                                   "print %r" % (output, expected))
            if (acceptable(plan.typed, plan.lines[0], output) and
                    output not in self.text()):
                return plan, kbd, shell, output
        raise RuntimeError("could not generate a line without reserved text")

    def echo_line(self, scenario, build, what):
        """Types an echo command with random parts over USB and checks that
        the shell prints exactly the edited arguments."""
        self.scenario = scenario
        plan, kbd, shell, output = self.plan(build)
        started = time.monotonic()
        try:
            self.type_plan(plan, what)
            self.kbd, self.shell = kbd, shell
            self.cursor = self.consume(
                self.cursor, output + "\r\n" + PROMPT, OUTPUT_TIMEOUT,
                "the output of the echo command typed over USB (%s) and the "
                "next prompt" % what)
        except Failure as e:
            e.info += ["keys typed: %s" % describe(plan.chunks),
                       "edited line: %r" % plan.lines[0],
                       "expected output: %r" % output,
                       "USB keyboards attached: %s" % " ".join(self.keyboards)]
            raise
        self.usb_outputs.append(output)
        self.lines_typed += 1
        say("USB %s: typed %r (%d key events) over %s; output %r (%.1f s)." % (
            scenario, plan.lines[0], plan.events, self.keyboards[-1], output,
            time.monotonic() - started))

    def command(self, line, output, what):
        kbd, shell = KeyboardModel(self.kbd), ShellModel(self.shell)
        plan = Plan(keys(line) + ENTER, kbd, shell)
        self.type_plan(plan, what)
        self.kbd, self.shell = kbd, shell
        self.cursor = self.consume(self.cursor, output + "\r\n" + PROMPT,
                                   OUTPUT_TIMEOUT,
                                   "the output of %s and the next prompt" % what)
        self.lines_typed += 1

    def serial_type(self, data, cursor, echo, what):
        """Types `data` into the serial console and returns the offset after
        its echo. QEMU drops the bytes that the guest has not read yet when
        the client disconnects, so the connection stays open until then."""
        self.journal.append(data)
        try:
            sock = connect_unix(SERIAL_PATH, 10.0)
        except OSError as e:
            raise Failure(self.scenario, "could not connect to the serial "
                          "console socket", [str(e)])
        self.serial_sock = sock
        try:
            sock.sendall(data.encode("latin-1"))
            return self.consume(cursor, echo, OUTPUT_TIMEOUT, what)
        finally:
            self.serial_sock = None
            sock.close()

    def serial_sync(self, scenario):
        """An echo command typed into the serial console: its output line
        proves that the shell reads the serial console and waits at a
        prompt."""
        self.scenario = scenario
        token = "ser_" + rand(12)
        for ch in "echo " + token + "\r":
            self.shell.feed(ch)
        self.cursor = self.serial_type("echo %s\r" % token, self.cursor,
                                       "echo %s\r\n" % token,
                                       "the echo of the serial command")
        self.cursor = self.consume(
            self.cursor, token + "\r\n" + PROMPT, OUTPUT_TIMEOUT,
            "the output of the echo command typed into the serial console")
        say("USB %s: the serial console typed 'echo %s'; output OK." % (
            scenario, token))

    # -- Devices -------------------------------------------------------------

    def qmp_execute(self, command, **arguments):
        try:
            return self.qmp.execute(command, **arguments)
        except (QmpError, OSError) as e:
            raise Failure(self.scenario, "QMP %s failed" % command, [str(e)])

    def expected_port(self, controller, qemu_port):
        """The xHCI port path of QEMU port `qemu_port` (USB 2 root ports
        follow the four USB 3 root ports); None if QEMU_USB_FLAGS replaced
        the root controller."""
        if controller == "xhci0" and not EXACT_ROOT:
            return None
        parts = str(qemu_port).split(".")
        parts[0] = str(int(parts[0]) + 4)
        return ".".join(parts)

    def plug(self, scenario, controller, qemu_port, kid, full_speed):
        """Hot-plugs a USB keyboard and waits for its 'keyboard attached'
        marker, before which no key may be typed on it."""
        self.scenario = scenario
        cursor = self.cursor
        args = {"driver": "usb-kbd", "bus": controller + ".0",
                "port": str(qemu_port), "id": kid}
        if full_speed:
            args["usb_version"] = 1
        self.qmp_execute("device_add", **args)
        bdf = self.bdf[controller]
        port = self.expected_port(controller, qemu_port)
        speed = "fs" if (full_speed or "." in str(qemu_port)) else "hs"
        if port is None:
            pattern = re.compile(r"\[USBKBD\] keyboard attached: %s port "
                                 r"(%s) \((ls|fs|hs|ss)\)" % (
                                     re.escape(bdf), PORT_PATH))
        else:
            root_port = port.split(".")[0]
            port_pat = (re.escape(port) if port == root_port else
                        "(?:%s|%s)" % (re.escape(port), re.escape(root_port)))
            pattern = re.compile(r"\[USBKBD\] keyboard attached: %s port "
                                 r"(%s) \(%s\)" % (
                                     re.escape(bdf), port_pat, re.escape(speed)))
        self.cursor, match = self.expect_marker(
            cursor, pattern, MARKER_TIMEOUT,
            "the 'keyboard attached' marker of the keyboard hot-plugged on "
            "%s port %s" % (controller, qemu_port))
        port = match.group(1)
        self.hotplug_markers.append(match.group(0))
        self.keyboards.append(kid)
        self.ports[kid] = (bdf, port)
        say("USB %s: hot-plugged %s (%s-speed) on %s port %s: %r." % (
            scenario, kid, "full" if speed == "fs" else "high", controller,
            qemu_port, match.group(0)))

    def plug_hub(self, scenario, controller, qemu_port, hid):
        """Hot-plugs a USB hub on `controller` port `qemu_port`."""
        self.scenario = scenario
        self.qmp_execute("device_add", driver="usb-hub",
                         bus=controller + ".0", port=str(qemu_port), id=hid)
        self.hubs[hid] = (controller, str(qemu_port))
        say("USB %s: hot-plugged hub %s on %s port %s." % (
            scenario, hid, controller, qemu_port))

    def unplug_hub(self, scenario, hid, kids=(), sub_hubs=()):
        """Unplugs a USB hub: QEMU cascade-deletes its child devices and emits
        DEVICE_DELETED for the hub and each child, and the driver emits
        'keyboard detached' for every attached downstream keyboard."""
        self.scenario = scenario
        cursor = self.cursor
        self.qmp_execute("device_del", id=hid)
        for dev_id in [hid] + list(sub_hubs) + list(kids):
            try:
                event = self.qmp.wait_event(
                    lambda e, d=dev_id: (e.get("event") == "DEVICE_DELETED" and
                                         e.get("data", {}).get("device") == d),
                    EVENT_TIMEOUT)
            except (QmpError, OSError) as e:
                raise Failure(scenario, "QMP failed while waiting for "
                              "DEVICE_DELETED", [str(e)])
            if event is None:
                raise Failure(scenario, "QEMU reported no DEVICE_DELETED event "
                              "for %s" % dev_id)
        self.hubs.pop(hid, None)
        for sub_id in sub_hubs:
            self.hubs.pop(sub_id, None)
        remaining = []
        for kid in kids:
            bdf, port = self.ports.pop(kid)
            self.keyboards.remove(kid)
            remaining.append("[USBKBD] keyboard detached: %s port %s" % (
                bdf, port))
        while remaining:
            pattern = re.compile("|".join(re.escape(m) for m in remaining))
            cursor, match = self.expect_marker(
                cursor, pattern, MARKER_TIMEOUT,
                "the 'keyboard detached' marker of a keyboard behind %s" % hid)
            remaining.remove(match.group(0))
            self.hotplug_markers.append(match.group(0))
        self.cursor = cursor
        say("USB %s: unplugged hub %s (sub_hubs=%r, kids=%r)." % (
            scenario, hid, list(sub_hubs), list(kids)))

    def unplug(self, scenario, kid):
        """Unplugs a USB keyboard: QEMU's DEVICE_DELETED event and the
        driver's 'keyboard detached' marker."""
        self.scenario = scenario
        cursor = self.cursor
        self.qmp_execute("device_del", id=kid)
        try:
            event = self.qmp.wait_event(
                lambda e: (e.get("event") == "DEVICE_DELETED" and
                           e.get("data", {}).get("device") == kid),
                EVENT_TIMEOUT)
        except (QmpError, OSError) as e:
            raise Failure(scenario, "QMP failed while waiting for "
                          "DEVICE_DELETED", [str(e)])
        if event is None:
            raise Failure(scenario, "QEMU reported no DEVICE_DELETED event "
                          "for the unplugged keyboard")
        bdf, port = self.ports.pop(kid)
        self.keyboards.remove(kid)
        marker = "[USBKBD] keyboard detached: %s port %s" % (bdf, port)
        self.cursor, match = self.expect_marker(
            cursor, re.compile(re.escape(marker)), MARKER_TIMEOUT,
            "the 'keyboard detached' marker of the unplugged keyboard")
        self.hotplug_markers.append(marker)
        say("USB %s: unplugged %s: %r." % (scenario, kid, marker))

    def require_only(self, kid):
        """A claim about one keyboard holds only while it is the only USB
        keyboard: QEMU sends the keys to the newest one."""
        if self.keyboards != [kid]:
            raise RuntimeError("%s is not the only USB keyboard: %r" % (
                kid, self.keyboards))

    # -- Session -------------------------------------------------------------

    def wait_session(self):
        """Waits for the interactive boot session: the boot self-tests
        first, then the init session with its scripted commands (help,
        echo boot_sh_ok, usertest 0) and the driver's markers."""
        self.scenario = "usb_session"

        def started(t):
            done = t.find(COMPLETION)
            ended = t.find(INIT_EXITED)
            if ended >= 0 and (done < 0 or ended < done):
                raise Failure("usb_session", "the init session ended before "
                              "the boot self-tests passed, so the kernel did "
                              "not run the interactive boot session that "
                              "fw_cfg opt/protos/boot_session=interactive "
                              "requests")
            nl = t.find("\n", done) if done >= 0 else -1
            return nl + 1 if nl >= 0 else None

        self.s0 = self.wait_for(
            "'%s' (the interactive boot session runs the self-tests first)" %
            COMPLETION, DEADLINE - time.monotonic(), started)
        self.session_live = True
        say("USB session (%s pass, %s topology): the boot self-tests passed "
            "after %.1f s." % (PASS_NAME, TOPOLOGY, time.monotonic() - T0))
        self.wait_for("'%s' after the boot self-tests" % INIT_HELLO,
                      SESSION_READY_TIMEOUT,
                      lambda t: True if INIT_HELLO in t[self.s0:] else None)

        def scripted(t):
            lines = complete_lines(t, self.s0)
            if SCRIPTED_ECHO not in lines or any(h not in lines
                                                 for h in HELP_LINES):
                return None
            for line in lines:
                if USERTEST_RE.fullmatch(line):
                    after = t.find(line, self.s0) + len(line)
                    return line if t.find(PROMPT, after) >= 0 else None
            return None

        self.usertest_line = self.wait_for(
            "the output of the init session's scripted commands (help, echo "
            "boot_sh_ok, usertest 0) and the next prompt", SESSION_READY_TIMEOUT,
            scripted)

    def check_ready(self):
        """Checks the driver's markers of the init session against the
        controllers and cold-plugged keyboards of the pass."""
        self.scenario = "usb_ready"
        ready_count = 2 if MULTI else 1
        kbd_count = 3 if MULTI else 1

        def markers(t):
            lines = complete_lines(t, self.s0)
            ready = [m for m in map(READY_RE.search, lines) if m]
            attached = [m for m in map(ATTACHED_RE.search, lines) if m]
            if len(ready) >= ready_count and len(attached) >= kbd_count:
                return ready, attached
            return None

        ready, attached = self.wait_for(
            "the driver's 'controller ... ready' and 'keyboard attached' "
            "markers in the init session", SESSION_READY_TIMEOUT, markers)
        try:
            self.qmp = Qmp(QMP_PATH)
            pci = self.qmp.execute("query-pci")
        except (QmpError, OSError) as e:
            raise Failure("usb_ready", "could not query the PCI devices "
                          "through QMP", [str(e)])

        def walk(devices):
            for d in devices:
                if d.get("qdev_id"):
                    self.bdf[d["qdev_id"]] = "%02x:%02x.%x" % (
                        d["bus"], d["slot"], d["function"])
                walk((d.get("pci_bridge") or {}).get("devices", []))

        for bus in pci:
            walk(bus.get("devices", []))
        controllers = ["xhci0", "xhci1"] if MULTI else ["xhci0"]
        if any(c not in self.bdf for c in controllers):
            raise Failure("usb_ready", "QEMU's query-pci does not list the "
                          "xHCI controllers of the pass", [repr(self.bdf)])
        want_ready = {"xhci0": "msix" if EXACT_ROOT else None, "xhci1": "msi"}
        got_ready = sorted(m.groups() for m in ready)
        got_attached = [m.groups() for m in attached]
        ok = len(got_ready) == ready_count and len(got_attached) == kbd_count
        if ok:
            bdf0 = self.bdf["xhci0"]
            r0 = [g for g in got_ready if g[0] == bdf0]
            a0 = [g for g in got_attached if g[0] == bdf0]
            want_p0 = "5" if EXACT_ROOT else None
            want_s0 = "hs" if EXACT_ROOT else None
            if (len(r0) != 1 or len(a0) != 1 or
                    want_ready["xhci0"] not in (None, r0[0][1]) or
                    want_p0 not in (None, a0[0][1]) or
                    want_s0 not in (None, a0[0][2])):
                ok = False
            else:
                self.keyboards.append("kbd0")
                self.ports["kbd0"] = (bdf0, a0[0][1])
        if ok and MULTI:
            bdf1 = self.bdf["xhci1"]
            r1 = [g for g in got_ready if g[0] == bdf1]
            a1 = [g for g in got_attached if g[0] == bdf1]
            root_a1 = [g for g in a1 if g[1] == "5" and g[2] == "fs"]
            hub_a1 = [g for g in a1 if g[1] in ("6.1", "6") and g[2] == "fs"]
            if (len(r1) != 1 or r1[0][1] != "msi" or len(root_a1) != 1 or
                    len(hub_a1) != 1):
                ok = False
            else:
                self.hubs["hub0"] = ("xhci1", "2")
                self.keyboards.append("hkbd0")
                self.ports["hkbd0"] = (bdf1, hub_a1[0][1])
                self.keyboards.append("kbd1")
                self.ports["kbd1"] = (bdf1, root_a1[0][1])
        if not ok:
            raise Failure("usb_ready", "the driver's markers do not match the "
                          "xHCI controllers and keyboards of the pass", [
                              "controllers (query-pci): %r" % {
                                  c: self.bdf[c] for c in controllers},
                              "ready markers: %r" % [m.group(0) for m in ready],
                              "attached markers: %r" % [m.group(0)
                                                        for m in attached]])
        self.initial_markers = ([m.group(0) for m in ready] +
                                [m.group(0) for m in attached])
        say("USB session: %s." % "; ".join(self.initial_markers))

    def settle(self):
        """Waits until the serial log ends with the prompt or a complete line
        (no output in progress) and returns its end."""
        return self.wait_for(
            "the shell to wait for input", OUTPUT_TIMEOUT,
            lambda t: len(t) if t.endswith(PROMPT) or t.endswith("\n") else None)

    def negative_control(self, scenario):
        """With no USB keyboard attached, keys sent through QMP go to the
        PS/2 keyboard, which the kernel does not drive: none may reach the
        shell. This shows that the shell got the other lines through the USB
        keyboard."""
        self.scenario = scenario
        if self.keyboards:
            raise RuntimeError("USB keyboards are attached: %r" % self.keyboards)
        token = rand(12, LETTERS)
        cursor = self.cursor
        for chunk in keys("echo " + token) + ENTER:
            self.send_keys(chunk)
        end = time.monotonic() + NEGATIVE_WINDOW
        while time.monotonic() < end:
            self.check()
            self.idle()
        t = self.text()
        i = cursor
        while True:
            match = LOG_LINE_RE.match(t, i)
            if not match:
                break
            self.log_line(match.group(0))
            i = match.end()
        if i < len(t):
            raise Failure(scenario, "keys typed while no USB keyboard was "
                          "attached reached the shell", [
                              "serial output: %r" % t[i:i + 120]])
        say("USB %s: %d key events without a USB keyboard; no shell output "
            "within %g s." % (scenario, 2 * len("echo " + token) + 2,
                              NEGATIVE_WINDOW))

    def exit_session(self):
        """Ends the init session with 'exit 0' typed over USB: init must exit
        with status 0, and the kernel must find every driver resource
        released."""
        self.scenario = "usb_exit"
        kbd, shell = KeyboardModel(self.kbd), ShellModel(self.shell)
        plan = Plan(keys("exit 0") + ENTER, kbd, shell)
        self.exit_typed = True
        self.type_plan(plan, "the exit command")
        self.kbd, self.shell = kbd, shell
        start = self.cursor

        def exited(t):
            for line in complete_lines(t, start):
                if line == INIT_EXITED_OK:
                    return t.find(INIT_EXITED_OK, start)
                if line.startswith(INIT_EXITED):
                    raise Failure("usb_exit", "init exited with a non-zero "
                                  "status after 'exit 0'", [repr(line)])
            return None

        end_of_session = self.wait_for("'%s' after 'exit 0'" % INIT_EXITED_OK,
                                       OUTPUT_TIMEOUT, exited)
        self.wait_for(
            "'%s' after the init session" % CLEANUP_PASS, OUTPUT_TIMEOUT,
            lambda t: (True if CLEANUP_PASS in complete_lines(t, end_of_session)
                       else None))
        self.session_live = False
        self.lines_typed += 1
        markers = []
        for line in complete_lines(self.log[:end_of_session], self.s0):
            match = (READY_RE.search(line) or ATTACHED_RE.search(line) or
                     DETACHED_RE.search(line))
            if match:
                markers.append(match.group(0))
        n = len(self.initial_markers)
        if (sorted(markers[:n]) != sorted(self.initial_markers) or
                markers[n:] != self.hotplug_markers):
            raise Failure("usb_exit", "the driver's markers in the init session "
                          "do not match its controllers and keyboard events", [
                              "found:    %r" % markers,
                              "expected: %r (in any order), then %r" % (
                                  self.initial_markers, self.hotplug_markers)])

    def check_journal(self):
        """Nothing that a USB-only line printed was typed into the serial
        console."""
        typed = "".join(self.journal)
        leaked = [o for o in self.usb_outputs if o in typed]
        if leaked:
            raise Failure("usb_session", "text of a line typed over USB also "
                          "went to the serial console", [repr(leaked)])

    # -- Lines -------------------------------------------------------------

    @staticmethod
    def token_line(n):
        token = rand(n)
        return keys("echo " + token) + ENTER, token

    @staticmethod
    def shift_backspace_line():
        """Capital letters (left Shift) and shifted symbols (right Shift),
        with three corrections at random places: a wrong capital erased, a
        letter and a shifted symbol erased, and Shift held down over a wrong
        letter and the Backspace that erases it."""
        base = rand(4) + shuffled(UPPER_1) + rand(3) + shuffled(SYMBOLS_1) + rand(2)
        units = [keys(c, "shift" if c in UPPER_1 else "shift_r") for c in base]
        wrong = secrets.choice(LETTERS)
        fixes = [
            keys(secrets.choice(LETTERS).upper()) + BACKSPACE,
            (keys(secrets.choice(LETTERS)) +
             keys(secrets.choice(SYMBOLS_1 + SYMBOLS_2), "shift_r") +
             BACKSPACE + BACKSPACE),
            [[("shift", True), (wrong, True), (wrong, False)],
             [("backspace", True), ("backspace", False), ("shift", False)]],
        ]
        places = sorted(RNG.sample(range(1, len(units)), len(fixes)),
                        reverse=True)
        for place, fix in zip(places, fixes):
            units.insert(place, fix)
        return keys("echo ") + [c for u in units for c in u] + ENTER, base

    @staticmethod
    def symbols_line(variant):
        """Capitals N-Z, the unshifted and the other shifted symbols, two
        overlapping keys (rollover) and a doubled letter."""
        head, tail, double = rand(4), rand(6), secrets.choice(LETTERS) * 2
        a, b = RNG.sample(LETTERS, 2)
        upper, plain, sym = shuffled(UPPER_2), shuffled(PLAIN_SYMBOLS), shuffled(
            SYMBOLS_2)
        if variant == 0:
            # Left Shift held over all capitals; right Shift per symbol.
            body = held("shift", upper.lower()) + keys(plain) + keys(sym, "shift_r")
        else:
            # Right Shift per capital; left Shift held over all symbols; a
            # letter erased.
            body = (keys(upper, "shift_r") + keys(plain) + held("shift", sym) +
                    keys(secrets.choice(LETTERS)) + BACKSPACE)
        chunks = (keys("echo " + head) + body + rollover(a, b) + keys(double + tail) +
                  ENTER)
        return chunks, head + upper + plain + sym + a + b + double + tail

    @staticmethod
    def caps_line(separator):
        """Backspace on the empty line, a word erased with one Backspace too
        many, then Caps Lock on (letters upper case, digits and symbols
        unchanged), Shift with Caps Lock (lower case), Caps Lock off."""
        l4, l2a, l2b = rand(4, LETTERS), rand(2, LETTERS), rand(2, LETTERS)
        r3, r2 = rand(3), rand(2)
        chunks = (BACKSPACE * 2 + keys("ls") + BACKSPACE * 3 + keys("echo") +
                  keys(separator) + keys(r3) + CAPS + keys(l4) + keys("[1") +
                  held("shift", l2a + "2") + CAPS + keys(l2b + r2) + ENTER)
        return chunks, r3 + l4.upper() + "[1" + l2a + "@" + l2b + r2

    @staticmethod
    def spacing_line():
        """Shift held over two letters, a letter erased, the digits, Tab as a
        separator and several spaces, which the shell collapses."""
        r6, l2, r4a, r4b = rand(6), rand(2, LETTERS), rand(4), rand(4)
        digits = shuffled(DIGITS)
        chunks = (keys("echo " + r6) + held("shift", l2) +
                  keys(secrets.choice(LETTERS)) + BACKSPACE + keys(digits) +
                  keys("\t" + r4a + "   " + r4b) + ENTER)
        return chunks, r6 + l2.upper() + digits + " " + r4a + " " + r4b

    @staticmethod
    def long_line(overflow):
        """A line of exactly the shell's 128-character limit, plus `overflow`
        characters after it, which the shell drops without echo."""
        content = shuffled(DIGITS) + shuffled(LETTERS) + rand(
            LINE_MAX - len("echo ") - len(DIGITS) - len(LETTERS))
        chunks = keys("echo " + content) + keys(rand(overflow)) + ENTER
        return chunks, content

    def mixed_line(self, scenario):
        """One command line typed partly over USB and partly into the serial
        console; a Backspace typed over USB erases a character that came from
        the serial console."""
        self.scenario = scenario
        for _ in range(200):
            a, b, c = rand(3), rand(3), rand(2)
            kbd, shell = KeyboardModel(self.kbd), ShellModel(self.shell)
            first = Plan(keys("echo " + a + "u"), kbd, shell)
            fragment = "s" + b
            fragment_echo = "".join(shell.feed(ch)[0] for ch in fragment)
            second = Plan(BACKSPACE + keys(c) + ENTER, kbd, shell)
            output = echo_output(second.lines[0])
            if output != a + "u" + "s" + b[:2] + c:
                raise RuntimeError("the models give %r for the mixed line" % output)
            if (acceptable(first.typed + fragment + second.typed,
                           second.lines[0], output) and output not in self.text()):
                break
        else:
            raise RuntimeError("could not generate a line without reserved text")
        started = time.monotonic()
        self.type_plan(first, "the USB part of the mixed line")
        self.cursor = self.serial_type(fragment, self.cursor, fragment_echo,
                                       "the echo of the serial part of the "
                                       "mixed line")
        self.type_plan(second, "the last USB part of the mixed line")
        self.kbd, self.shell = kbd, shell
        self.cursor = self.consume(self.cursor, output + "\r\n" + PROMPT,
                                   OUTPUT_TIMEOUT,
                                   "the output of the line typed over USB and "
                                   "the serial console")
        self.lines_typed += 1
        say("USB %s: typed %r over USB and %r into the serial console, then "
            "Backspace and %r over USB; output %r (%.1f s)." % (
                scenario, "echo " + a + "u", fragment, c, output,
                time.monotonic() - started))

    def commands(self, scenario):
        """Realistic session steps typed over USB: help, an unknown command,
        and a user program (usertest 0) like in the scripted commands."""
        self.scenario = scenario
        self.command("help", "\r\n".join(HELP_LINES), "help")
        for _ in range(200):
            unknown = "zz" + rand(5, LETTERS)
            if acceptable(unknown):
                break
        self.command(unknown, "sh: unknown command: " + unknown,
                     "an unknown command")
        self.command("usertest 0", self.usertest_line, "usertest 0")
        say("USB %s: help, %r and usertest 0 typed over %s; outputs OK." % (
            scenario, unknown, self.keyboards[-1]))

    # -- Scenarios -----------------------------------------------------------

    def run(self):
        self.wait_session()
        self.check_ready()
        self.cursor = self.settle()
        self.serial_sync("usb_serial_sync")
        if MULTI:
            self.uefi_scenarios()
        else:
            self.bios_scenarios()
        self.check_journal()
        self.exit_session()
        say("USB session: all scenarios passed in %.1f s (%d lines typed over "
            "USB, ended by 'exit 0')." % (time.monotonic() - T0, self.lines_typed))

    def bios_scenarios(self):
        # The keyboard that SeaBIOS used (high-speed, on the root controller
        # with MSI-X): a random token, then Shift and Backspace corrections.
        self.echo_line("usb_token", lambda: self.token_line(16), "the token line")
        self.echo_line("usb_shift_backspace", self.shift_backspace_line,
                       "the Shift/Backspace line")
        self.scenario = "usb_editing_edges"
        kbd, shell = KeyboardModel(self.kbd), ShellModel(self.shell)
        empty = Plan(ENTER, kbd, shell)
        self.type_plan(empty, "the empty line")
        self.kbd, self.shell = kbd, shell
        self.cursor = self.consume(self.cursor, PROMPT, OUTPUT_TIMEOUT,
                                   "the prompt after an empty line")
        self.echo_line("usb_editing_edges", lambda: self.caps_line(" "),
                       "the Caps Lock line")
        self.mixed_line("usb_serial_mixed")
        # Unplugged, the keyboard is gone, and keys reach nothing.
        self.unplug("usb_unplug", "kbd0")
        self.negative_control("usb_no_keyboard")
        # A full-speed keyboard hot-plugged after the shell started, on the
        # last root port; the only USB keyboard, so the line came through it.
        self.plug("usb_hotplug_add", "xhci0", 4, "kbd_fs", True)
        self.require_only("kbd_fs")
        self.echo_line("usb_hotplug_add", lambda: self.symbols_line(0),
                       "the symbols line")
        # Detached and attached again on the same port.
        self.unplug("usb_replug", "kbd_fs")
        self.plug("usb_replug", "xhci0", 4, "kbd_fs", True)
        self.require_only("kbd_fs")
        self.echo_line("usb_replug", self.spacing_line, "the spacing line")
        # A second, high-speed keyboard hot-plugged on the same controller.
        self.plug("usb_second_keyboard", "xhci0", 2, "kbd_hs", False)
        self.echo_line("usb_second_keyboard", lambda: self.token_line(12),
                       "the token line")
        self.echo_line("usb_long_line", lambda: self.long_line(2),
                       "the 128-character line")
        # Single USB hub + downstream keyboard (the only attached keyboard).
        self.unplug("usb_hub_single", "kbd_hs")
        self.unplug("usb_hub_single", "kbd_fs")
        self.plug_hub("usb_hub_single", "xhci0", 2, "hub0")
        self.plug("usb_hub_single", "xhci0", "2.1", "hkbd0", True)
        self.require_only("hkbd0")
        self.echo_line("usb_hub_single", lambda: self.token_line(16),
                       "the single-hub token line")
        # Chained/nested hubs: xhci0 -> hub0 (port 2) -> hub1 (port 2.2) -> hkbd1 (port 2.2.3).
        self.plug_hub("usb_hub_chained", "xhci0", "2.2", "hub1")
        self.plug("usb_hub_chained", "xhci0", "2.2.3", "hkbd1", True)
        self.echo_line("usb_hub_chained", lambda: self.token_line(16),
                       "the chained-hub token line")
        # Downstream keyboard unplug and hot-plug on a hub port.
        self.unplug("usb_hub_hotplug", "hkbd0")
        self.unplug("usb_hub_hotplug", "hkbd1")
        self.plug("usb_hub_hotplug", "xhci0", "2.3", "hkbd2", True)
        self.require_only("hkbd2")
        self.echo_line("usb_hub_hotplug", lambda: self.token_line(16),
                       "the hub hot-plug token line")
        # Unplugging and re-plugging the hub (and its downstream devices).
        self.unplug_hub("usb_hub_replug", "hub0", kids=("hkbd2",),
                        sub_hubs=("hub1",))
        self.plug_hub("usb_hub_replug", "xhci0", 2, "hub0")
        self.plug("usb_hub_replug", "xhci0", "2.1", "hkbd0", True)
        self.require_only("hkbd0")
        self.echo_line("usb_hub_replug", lambda: self.token_line(16),
                       "the hub re-plug token line")
        self.commands("usb_commands")

    def uefi_scenarios(self):
        # kbd1, the newest keyboard: full-speed, enumerated by OVMF, on the
        # MSI-only controller behind the bridge.
        self.echo_line("usb_token", lambda: self.token_line(16), "the token line")
        self.echo_line("usb_shift_backspace", self.shift_backspace_line,
                       "the Shift/Backspace line")
        # Unplug kbd1: hkbd0 (cold-plugged behind hub0 on xhci1 port 2.1) is now
        # the newest keyboard; type a fresh token through it, then unplug hkbd0
        # so only kbd0 remains.
        self.unplug("usb_unplug", "kbd1")
        self.echo_line("usb_hub_single", lambda: self.token_line(16),
                       "the single-hub token line")
        self.unplug("usb_unplug", "hkbd0")
        self.require_only("kbd0")
        self.echo_line("usb_root_controller", lambda: self.symbols_line(1),
                       "the symbols line")
        self.scenario = "usb_editing_edges"
        kbd, shell = KeyboardModel(self.kbd), ShellModel(self.shell)
        empty = Plan(ENTER, kbd, shell)
        self.type_plan(empty, "the empty line")
        self.kbd, self.shell = kbd, shell
        self.cursor = self.consume(self.cursor, PROMPT, OUTPUT_TIMEOUT,
                                   "the prompt after an empty line")
        self.require_only("kbd0")
        self.echo_line("usb_editing_edges", lambda: self.caps_line("\t"),
                       "the Caps Lock line")
        self.serial_sync("usb_serial_after_usb")
        # kbd1 attached again on the same port of the bridged controller, now
        # high-speed; then the only USB keyboard.
        self.plug("usb_replug", "xhci1", 1, "kbd1", False)
        self.unplug("usb_bridged_controller", "kbd0")
        self.require_only("kbd1")
        self.echo_line("usb_bridged_controller", lambda: self.long_line(0),
                       "the 128-character line")
        # A full-speed keyboard hot-plugged on a fresh port of the bridged
        # controller (both keyboards are on that controller).
        self.plug("usb_hotplug_add", "xhci1", 3, "kbd_br", True)
        self.echo_line("usb_hotplug_add", lambda: self.token_line(12),
                       "the token line")
        # Chained/nested hubs on the bridged MSI controller (hub0 is already on
        # xhci1 port 2): attach hub1 on 2.2 and hkbd1 on 2.2.3.
        self.plug_hub("usb_hub_chained", "xhci1", "2.2", "hub1")
        self.plug("usb_hub_chained", "xhci1", "2.2.3", "hkbd1", True)
        self.echo_line("usb_hub_chained", lambda: self.token_line(16),
                       "the chained-hub token line")
        # Downstream keyboard unplug and hot-plug on a hub port.
        self.unplug("usb_hub_hotplug", "hkbd1")
        self.plug("usb_hub_hotplug", "xhci1", "2.3", "hkbd2", True)
        self.echo_line("usb_hub_hotplug", lambda: self.token_line(16),
                       "the hub hot-plug token line")
        # Unplugging and re-plugging hub0 (cleaning up hub1 and hkbd2).
        self.unplug_hub("usb_hub_replug", "hub0", kids=("hkbd2",),
                        sub_hubs=("hub1",))
        self.plug_hub("usb_hub_replug", "xhci1", 2, "hub0")
        self.plug("usb_hub_replug", "xhci1", "2.1", "hkbd0", True)
        self.echo_line("usb_hub_replug", lambda: self.token_line(16),
                       "the hub re-plug token line")
        self.commands("usb_commands")

    # -- Failure handling ------------------------------------------------------

    def recover(self):
        """After a failure: releases held keys and ends the init session over
        the serial console, so that the kernel still runs its checks and the
        stress test (whose output helps diagnose the failure)."""
        try:
            if self.qmp is not None and self.pressed:
                self.qmp.execute("input-send-event", events=[
                    {"type": "key", "data": {
                        "down": False, "key": {"type": "qcode", "data": q}}}
                    for q in sorted(self.pressed)])
        except Exception:  # Best effort.
            pass
        if not self.session_live or self.exit_typed:
            return
        try:
            say("USB session: ending the init session with 'exit 0' typed into "
                "the serial console.")
            sock = connect_unix(SERIAL_PATH, 5.0)
            try:
                sock.sendall(b"\rexit 0\r")
                end = min(time.monotonic() + 10.0, DEADLINE)
                while (time.monotonic() < end and
                       self.text().find(INIT_EXITED, self.s0) < 0):
                    try:
                        while sock.recv(65536, socket.MSG_DONTWAIT):
                            pass
                    except OSError:
                        pass
                    time.sleep(0.05)
            finally:
                sock.close()
        except Exception:  # Best effort.
            pass

    def close(self):
        if self.qmp is not None:
            try:
                self.qmp.close()
            except OSError:
                pass


def report(scenario, detail, info):
    print("FAIL: %s: %s" % (scenario, detail), file=sys.stderr, flush=True)
    for line in info:
        print("  " + line, file=sys.stderr, flush=True)


def main():
    session = Session()
    code = 1
    try:
        session.run()
        code = 0
    except SerialFailureLine:
        say("USB session stopped: the serial log shows a failure line.")
        session.recover()
        code = 2
    except Failure as e:
        report(e.scenario, e.detail, e.info)
        session.recover()
    except Exception as e:  # A harness bug: report it like a failure.
        report("usb_session", "internal error of the USB harness (%s)" %
               type(e).__name__, traceback.format_exc().splitlines())
        session.recover()
    finally:
        session.close()
    return code


sys.exit(main())
USB_SESSION_PY
}

# Runs the USB keyboard scenarios of pass $1 (label $2, serial log $3,
# topology $4) until SECONDS reaches $5. Returns 0 once they passed; otherwise
# reports the failure (a failure line in the serial log like a failed boot)
# and returns 1.
run_usb_pass_session() {
  local name="$1" label="$2" log="$3" topology="$4" deadline="$5"
  local rc=0
  run_usb_session "${name}" "${log}" "${topology}" "$((deadline - SECONDS))" || rc=$?
  if [[ "${rc}" -eq 0 ]]; then
    return 0
  fi
  drain_after_failure "${log}" "${QEMU_PID}" "${deadline}"
  if [[ "${rc}" -eq 2 ]]; then
    report_failure "${label}" "${log}" "${name}"
  else
    print_qemu_stderr "${name}"
    echo "Captured ${label} serial output:" >&2
    cat "${log}" >&2
  fi
  return 1
}

# Legacy BIOS boot of ISO $2 on QEMU CPU model $4; pass name $1, display label
# $3. The remaining arguments are serial markers that only this pass requires.
# The bios pass also runs the USB keyboard scenarios (unless BOOT_TEST_USB=0),
# with the USB devices of `make run`, within its timeout plus
# USB_SESSION_TIMEOUT_SECS.
run_bios_pass() {
  local name="$1" iso="$2" label="$3" cpu="$4"
  shift 4
  local log="${RUN_DIR}/serial_${name}_output.log"
  local QEMU_PID=""
  trap kill_qemu RETURN

  local cpu_args=(-cpu "${cpu}")
  if [[ "${KVM}" == "1" ]]; then
    cpu_args=(-enable-kvm -cpu host)
  fi
  local USB_SESSION=0 usb_args=() usb_markers=() timeout="${BIOS_TIMEOUT_SECS}" deadline=0
  if [[ "${BOOT_TEST_USB}" == "1" && "${name}" == "bios" ]]; then
    USB_SESSION=1
    usb_args=("${QEMU_USB_ARGS[@]}" "${USB_FW_CFG_ARGS[@]}")
    usb_markers=("${USB_REQUIRED_MARKERS[@]}")
    label="${label} + USB keyboard"
    deadline=$((SECONDS + BIOS_TIMEOUT_SECS + USB_SESSION_TIMEOUT_SECS))
  fi
  echo "Running ${label} boot verification..."
  start_qemu "${name}" "${cpu_args[@]}" -cdrom "${iso}" ${usb_args[@]+"${usb_args[@]}"}
  if [[ "${USB_SESSION}" == "1" ]]; then
    if ! run_usb_pass_session "${name}" "${label}" "${log}" single "${deadline}"; then
      return 1
    fi
    timeout=$((deadline - SECONDS))
    if [[ "${timeout}" -lt 1 ]]; then
      timeout=1
    fi
  fi
  if ! wait_for_boot "${log}" "${QEMU_PID}" "${timeout}"; then
    report_failure "${label}" "${log}" "${name}"
    return 1
  fi
  if ! verify_serial_log "${log}" "${QEMU_PID}" "$@" ${usb_markers[@]+"${usb_markers[@]}"} ||
     ! verify_pass_absent_markers "${name}" "${log}" ||
     ! verify_x87_pointer_reset "${log}" "${RUN_DIR}/qemu_${name}_qmp.sock" ||
     ! verify_vga_text "${RUN_DIR}/qemu_${name}_monitor.sock" "${RUN_DIR}/vga_${name}_dump.bin" ||
     ! stop_qemu_and_recheck_log "${log}"; then
    print_qemu_stderr "${name}"
    cat "${log}" >&2
    return 1
  fi
  echo "Captured ${label} serial output:"
  cat "${log}"
}

# UEFI boot of ISO $1 with 4 GiB of RAM and x2APIC; also runs the USB keyboard
# scenarios (unless BOOT_TEST_USB=0) with a second, MSI-only xHCI controller
# behind a PCI bridge, within its timeout plus USB_SESSION_TIMEOUT_SECS.
run_uefi_pass() {
  local iso="$1"
  local name="uefi" label="UEFI/Multiboot2 (OVMF, x2APIC, 4 GiB)"
  local log="${RUN_DIR}/serial_${name}_output.log"
  local QEMU_PID=""
  trap kill_qemu RETURN

  local OVMF_ARGS=() ovmf_rc=0
  find_ovmf || ovmf_rc=$?
  if [[ "${ovmf_rc}" -eq 2 ]]; then
    return 1
  fi
  if [[ "${ovmf_rc}" -ne 0 ]]; then
    if [[ "${REQUIRE_UEFI:-0}" == "1" ]]; then
      echo "FAIL: No OVMF firmware found and REQUIRE_UEFI=1 (set OVMF=/path/to/OVMF.fd)." >&2
      return 1
    fi
    echo "SKIPPED: ${label} pass (no OVMF firmware found; set OVMF=/path/to/OVMF.fd)."
    echo "skipped" >"${RUN_DIR}/${name}_pass_result"
    return 0
  fi

  local cpu_args=(-cpu "qemu64,+x2apic")
  if [[ "${KVM}" == "1" ]]; then
    cpu_args=(-enable-kvm -cpu "host,+x2apic")
  fi
  local USB_SESSION=0 usb_args=() usb_markers=() timeout="${UEFI_TIMEOUT_SECS}" deadline=0
  if [[ "${BOOT_TEST_USB}" == "1" ]]; then
    USB_SESSION=1
    usb_args=("${QEMU_USB_ARGS[@]}" "${USB_MULTI_ARGS[@]}" "${USB_FW_CFG_ARGS[@]}")
    usb_markers=("${USB_REQUIRED_MARKERS[@]}")
    label="${label} + USB keyboards on 2 xHCI controllers"
    deadline=$((SECONDS + UEFI_TIMEOUT_SECS + USB_SESSION_TIMEOUT_SECS))
  fi
  echo "Running ${label} boot verification..."
  start_qemu "${name}" "${OVMF_ARGS[@]}" "${cpu_args[@]}" -m 4G -cdrom "${iso}" \
    ${usb_args[@]+"${usb_args[@]}"}
  if [[ "${USB_SESSION}" == "1" ]]; then
    if ! run_usb_pass_session "${name}" "${label}" "${log}" multi "${deadline}"; then
      return 1
    fi
    timeout=$((deadline - SECONDS))
    if [[ "${timeout}" -lt 1 ]]; then
      timeout=1
    fi
  fi
  if ! wait_for_boot "${log}" "${QEMU_PID}" "${timeout}"; then
    report_failure "${label}" "${log}" "${name}"
    return 1
  fi
  if ! verify_serial_log "${log}" "${QEMU_PID}" \
         "[PMM] mmap entry: base=0x100000000" "[PMM] Framebuffer: addr=" \
         "${QEMU64_MARKERS[@]}" ${usb_markers[@]+"${usb_markers[@]}"} ||
     ! verify_pass_absent_markers "${name}" "${log}" ||
     ! verify_x87_pointer_reset "${log}" "${RUN_DIR}/qemu_${name}_qmp.sock" ||
     ! verify_framebuffer "${log}" "${RUN_DIR}/qemu_${name}_monitor.sock" "${RUN_DIR}/vga_${name}_dump.bin" ||
     ! stop_qemu_and_recheck_log "${log}"; then
    print_qemu_stderr "${name}"
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
    echo "skipped" >"${RUN_DIR}/mb1_pass_result"
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
    echo "skipped" >"${RUN_DIR}/smep_pass_result"
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
PASSES_STARTED=1
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
    run_pass "${name}" >"${RUN_DIR}/${name}_pass_stdout.log" 2>&1 &
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
usb_passed=()
for i in "${!PASS_NAMES[@]}"; do
  name="${PASS_NAMES[i]}"
  rc="${PASS_RCS[i]}"
  pass_stdout="${RUN_DIR}/${name}_pass_stdout.log"
  cat "${pass_stdout}"
  # The pass output ends with the serial log, whose last line is cut off if
  # QEMU was stopped in the middle of it.
  if [[ -n "$(tail -c 1 "${pass_stdout}")" ]]; then
    echo
  fi
  if [[ "${rc}" -ne 0 ]]; then
    overall_rc=1
    status="FAIL"
  elif [[ -f "${RUN_DIR}/${name}_pass_result" ]]; then
    status="SKIPPED"
  else
    status="PASS"
    if [[ "${BOOT_TEST_USB}" == "1" && ( "${name}" == "bios" || "${name}" == "uefi" ) ]]; then
      usb_passed+=("${name}")
    fi
  fi
  summary+=("${name}: ${status}")
  save_pass_artifacts "${name}" "${status}"
done
PASSES_STARTED=0

if [[ "${overall_rc}" -ne 0 ]]; then
  echo "FAILURE: boot verification failed (${summary[*]})." >&2
  exit 1
fi
usb_summary=""
if [[ "${#usb_passed[@]}" -gt 0 ]]; then
  usb_summary=" USB keyboard scenarios passed in: ${usb_passed[*]}."
fi
echo "SUCCESS: boot verification passed (${summary[*]}): greeting, all boot self-tests, ${MIN_STRESS_PASSES}+ stress-test passes, and no panic, triple fault, or reboot.${usb_summary}"
