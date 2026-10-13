#!/usr/bin/env bash
# Boots the kernel ISO(s) headlessly in QEMU and verifies the boot self-tests.
#
# Passes (run in parallel):
#   1. Legacy BIOS + Multiboot2: serial markers plus the VGA text buffer at
#      0xB8000.
#   2. UEFI (OVMF) + Multiboot2 + x2APIC + 4 GiB RAM: serial markers plus the
#      GOP linear framebuffer.
#   3. Legacy BIOS + Multiboot1: serial markers plus the VGA text buffer (only
#      when the Multiboot1 ISO exists).
#
# Usage: test_boot.sh [kernel.iso [kernel-mb1.iso]]
#
# Environment:
#   BIOS_TIMEOUT_SECS  Timeout of each legacy BIOS pass (default: 60).
#   UEFI_TIMEOUT_SECS  Timeout of the UEFI pass (default: 90).
#   MIN_STRESS_PASSES  Stress-test iterations that must pass (default: 3).
#   OVMF               Combined OVMF image (passed with -bios), or the
#                      OVMF_CODE image when OVMF_VARS is also set.
#   OVMF_VARS          OVMF_VARS template; selects pflash mode with OVMF.
#   REQUIRE_UEFI=1     Fail instead of skipping when no OVMF image is found.
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
  "${COMPLETION_MARKER}"
  "${STRESS_MARKER}"
)

PASS_NAMES=(bios uefi mb1)
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
    -smp 4 \
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

# Legacy BIOS boot of ISO $2; pass name $1, display label $3.
run_bios_pass() {
  local name="$1" iso="$2" label="$3"
  local log="${BUILD_DIR}/serial_${name}_output.log"
  local QEMU_PID=""
  trap kill_qemu RETURN

  echo "Running ${label} boot verification..."
  start_qemu "${name}" -cdrom "${iso}"
  if ! wait_for_boot "${log}" "${QEMU_PID}" "${BIOS_TIMEOUT_SECS}"; then
    report_failure "${label}" "${log}"
    return 1
  fi
  if ! verify_serial_log "${log}" "${QEMU_PID}" ||
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

  echo "Running ${label} boot verification..."
  start_qemu "${name}" "${OVMF_ARGS[@]}" -cpu qemu64,+x2apic -m 4G -cdrom "${iso}"
  if ! wait_for_boot "${log}" "${QEMU_PID}" "${UEFI_TIMEOUT_SECS}"; then
    report_failure "${label}" "${log}"
    return 1
  fi
  if ! verify_serial_log "${log}" "${QEMU_PID}" \
         "[PMM] mmap entry: base=0x100000000" "[PMM] Framebuffer: addr=" ||
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
  run_bios_pass mb1 "${iso}" "BIOS/Multiboot1"
}

run_bios_pass bios "${ISO_FILE}" "BIOS/Multiboot2" >"${BUILD_DIR}/bios_pass_stdout.log" 2>&1 &
BIOS_JOB_PID=$!
run_uefi_pass "${ISO_FILE}" >"${BUILD_DIR}/uefi_pass_stdout.log" 2>&1 &
UEFI_JOB_PID=$!
run_mb1_pass "${MB1_ISO_FILE}" >"${BUILD_DIR}/mb1_pass_stdout.log" 2>&1 &
MB1_JOB_PID=$!

overall_rc=0
summary=()
for name in "${PASS_NAMES[@]}"; do
  pid_var="$(tr '[:lower:]' '[:upper:]' <<<"${name}")_JOB_PID"
  rc=0
  wait "${!pid_var}" || rc=$?
  cat "${BUILD_DIR}/${name}_pass_stdout.log"
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
