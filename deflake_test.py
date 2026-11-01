#!/usr/bin/env python3
"""Unit tests for deflake.py and test_boot.sh CLI hooks."""

from __future__ import annotations

import ast
import contextlib
import dataclasses
import fcntl
import io
import json
import os
from pathlib import Path
import random
import re
import shlex
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import tokenize
import unittest
from unittest import mock

import deflake

REPO_ROOT = Path(__file__).resolve().parent


def process_gone(pid: int, timeout_s: float = 10.0) -> bool:
  """Waits up to `timeout_s` for process `pid` to end (a zombie counts)."""
  deadline = time.monotonic() + timeout_s
  while True:
    if sys.platform.startswith("linux"):
      try:
        with open(f"/proc/{pid}/stat", encoding="latin-1") as f:
          if f.read().rsplit(")", 1)[1].split()[0] == "Z":
            return True
      except OSError:
        pass
    try:
      os.kill(pid, 0)
    except ProcessLookupError:
      return True
    except PermissionError:
      pass
    if time.monotonic() >= deadline:
      return False
    time.sleep(0.05)


def wait_for_file(path: Path, timeout_s: float = 30.0) -> bool:
  """Waits up to `timeout_s` for `path` to exist."""
  deadline = time.monotonic() + timeout_s
  while not path.exists():
    if time.monotonic() >= deadline:
      return False
    time.sleep(0.05)
  return True


def make_run_record(**fields: object) -> deflake.RunRecord:
  """A RunRecord of a failed host:bios run, with `fields` replaced."""
  record = deflake.RunRecord(
      config="host:bios",
      index=1,
      build="b1",
      status="FAIL",
      reasons=("test_failed:rbtree_edge_cases",),
      outcomes={},
      failed_tests=["rbtree_edge_cases"],
      phase="after suite (failed)",
      seeds=[],
      seed_lines=[],
      failure_lines=[],
      expectations=[],
      diagnostics=[],
      duration_s=1.0,
      start_utc="2026-10-20T18:00:00Z",
      lock_wait_s=0.0,
      loadavg_before=[0.0, 0.0, 0.0],
      qemu_argv=(),
      qemu_cwd="/repo",
      harness_argv=(),
      env_recorded={},
      nondefault_harness_env={},
      iso_snapshot="/store/abc.iso",
      mb1_iso_snapshot=None,
      exit_code=1,
      t_pass=None,
      t_fail=1.0,
  )
  return dataclasses.replace(record, **fields)


def make_test_boot_repo(tmp_path: Path, fake_qemu: str) -> dict[str, str]:
  """Sets up `tmp_path` as a repository with a copy of test_boot.sh, fake
  ISOs and OVMF images, and `fake_qemu` (a script) as qemu-system-x86_64 in
  tmp_path/fakebin. Returns the environment to run the copy in."""
  shutil.copy2(REPO_ROOT / "test_boot.sh", tmp_path / "test_boot.sh")
  (tmp_path / "test_boot.sh").chmod(0o755)
  for name in ("kernel.iso", "kernel-mb1.iso", "OVMF.fd", "OVMF_CODE.fd",
               "OVMF_VARS.fd"):
    (tmp_path / name).write_bytes(b"fake " + name.encode())
  (tmp_path / "fakebin").mkdir()
  qemu = tmp_path / "fakebin" / "qemu-system-x86_64"
  qemu.write_text(textwrap.dedent(fake_qemu), encoding="utf-8")
  qemu.chmod(0o755)
  env = os.environ.copy()
  env["PATH"] = f"{tmp_path / 'fakebin'}:{env.get('PATH', '')}"
  for var in (*deflake.BOOT_TEST_ENV_VARS, "USE_DOCKER", "KVM", "MAKEFLAGS",
              "MAKELEVEL", "MFLAGS", "MAKEOVERRIDES"):
    env.pop(var, None)
  return env


def make_serial_log(
    *,
    names: tuple[str, ...] | list[str] | None = None,
    cpus: tuple[int, int] | None = (24, 24),
    fail_test: str | None = None,
    injected_seed: str | None = None,
    injected_one_in: int = 20,
    stress_passes: int = 3,
    stress_fail: bool = False,
    fatal_after: str | None = None,
    greetings: int = 1,
    crlf: str = "\r\n",
    extra_lines_before_end: list[str] | None = None,
    include_suite_end: bool = True,
) -> bytes:
  """Builds a synthetic kernel serial log for testing."""
  if names is None:
    names = deflake.load_boot_test_names(REPO_ROOT)
  lines: list[str] = []
  for _ in range(greetings):
    lines.append("Hello, x86-64 Kernel World!")
  if cpus is not None:
    lines.append(f"[SMP] Online CPUs: {cpus[0]}/{cpus[1]}")
  lines.append("[CPU] vendor=AuthenticAMD")
  lines.append("[CPU] x87 pointer reset: disabled")

  any_fail = False
  for name in names:
    if name == "cpu_protection_features":
      lines.append("[TEST] cpu_protection_features: cpus=4")
    if name == "user_mode_kill":
      lines.append("[TEST] user_mode_kill: spinners=4 wait_ms=10")
    if name == "rbtree_edge_cases" and injected_seed is not None:
      lines.append(
          f"[TEST] rbtree_edge_cases: injected flake one_in={injected_one_in}"
          f" seed={injected_seed}"
      )
    if name == fail_test:
      lines.append(
          "[TEST] expectation failed: !injected_failure at"
          " selftest/boot_selftest.cpp:735"
      )
      lines.append(f"[TEST] {name}: FAIL")
      any_fail = True
    else:
      lines.append(f"[TEST] {name}: PASS")
    if fatal_after == name:
      lines.append("[FATAL] Unhandled page fault on CPU 3")
      return (crlf.join(lines) + crlf).encode("latin-1")

  if extra_lines_before_end:
    lines.extend(extra_lines_before_end)

  if include_suite_end:
    if any_fail:
      lines.append("[TEST] BOOT SELF-TESTS FAILED")
    else:
      lines.append("[TEST] ALL BOOT SELF-TESTS PASSED")
      for i in range(stress_passes):
        seed_val = 0x1000 + i
        lines.append(
            f"[STRESS] seed=0x{seed_val:016x} pass={i + 1} tasks=48"
        )
        if stress_fail and i == stress_passes - 1:
          lines.append(
              "[STRESS] random_multicpu_stress: FAIL (counter mismatch)"
              f" seed=0x{seed_val:016x}"
          )
        else:
          lines.append("[STRESS] random_multicpu_stress: PASS")
  return (crlf.join(lines) + crlf).encode("latin-1")


# A fake qemu-system-x86_64 for test_boot.sh's USB keyboard scenarios. Its
# guest boots like the kernel in the interactive boot session (the boot
# self-tests, then an init session that waits for 'exit 0'), has a driver that
# turns the key events of each USB keyboard into boot-protocol reports (like
# QEMU's usb-kbd) and those into US-layout characters, and the shell's line
# editor; the serial console socket drops unread input when the client hangs
# up (like QEMU). FAKE_BOOT_LINES names a file of lines to print before the
# self-tests end; FAKE_BUG injects one bug: "noninteractive" (the kernel
# ignores the fw_cfg opt-in), "symbols_unshifted" (Shift does not change
# digits and symbols) or "late_failure" (the stress test fails while the
# harness dumps the VGA text buffer or framebuffer, after its last serial log
# check before that dump).
FAKE_GUEST_QEMU = r'''#!/usr/bin/env python3
import json, os, queue, select, socket, sys, threading, time

argv = sys.argv[1:]
if "--version" in argv:
    print("QEMU emulator version 11.1.0")
    sys.exit(0)
BUG = os.environ.get("FAKE_BUG", "")


def values(name):
    return [argv[i + 1] for i, a in enumerate(argv[:-1]) if a == name]


def opts(spec):
    parts = spec.split(",")
    return parts[0], dict(p.split("=", 1) for p in parts[1:] if "=" in p)


serial_sock_path = None
serial = values("-serial")[0]
if serial.startswith("file:"):
    log_path = serial[len("file:"):]
else:
    for spec in values("-chardev"):
        _, o = opts(spec)
        if o.get("id") == serial[len("chardev:"):]:
            log_path, serial_sock_path = o["logfile"], o["path"]
qmp_path = values("-qmp")[0][len("unix:"):].split(",")[0]
monitor_path = values("-monitor")[0][len("unix:"):].split(",")[0]
interactive = ("name=opt/protos/boot_session,string=interactive" in
               values("-fw_cfg") and BUG != "noninteractive")
uefi = "-bios" in argv
def xhci_port(port_spec):
    parts = str(port_spec).split(".")
    parts[0] = str(int(parts[0]) + 4)
    return ".".join(parts)


controllers, cold, hubs, slot = {}, [], {}, 4
for spec in values("-device"):
    driver, o = opts(spec)
    if driver in ("qemu-xhci", "nec-usb-xhci"):
        if o.get("bus") == "br1":
            bdf = "01:%02x.0" % int(o["addr"])
        else:
            bdf, slot = "00:%02x.0" % slot, slot + 1
        controllers[o["id"]] = (bdf, "msi" if o.get("msix") == "off" else "msix")
    elif driver == "pci-bridge":
        controllers[o["id"]], slot = ("00:%02x.0" % slot, None), slot + 1
    elif driver == "usb-hub":
        hubs[o["id"]] = {"id": o["id"], "ctrl": o["bus"].split(".")[0],
                         "port": str(o["port"])}
    elif driver == "usb-kbd":
        port_str = str(o["port"])
        is_fs = o.get("usb_version") == "1" or "." in port_str
        cold.append({"id": o["id"], "ctrl": o["bus"].split(".")[0],
                     "port": port_str,
                     "speed": "fs" if is_fs else "hs"})

log_lock = threading.Lock()
log_file = open(log_path, "ab", buffering=0)


def log(text):
    with log_lock:
        log_file.write(text.replace("\n", "\r\n").encode("latin-1"))


SHIFTED = dict(zip("0123456789", ")!@#$%^&*("))
KEYS = {"grave_accent": "`~", "minus": "-_", "equal": "=+", "bracket_left": "[{",
        "bracket_right": "]}", "backslash": "\\|", "semicolon": ";:",
        "apostrophe": "'\"", "comma": ",<", "dot": ".>", "slash": "/?",
        "spc": "  ", "tab": "\t\t", "ret": "\r\r", "backspace": "\b\b"}
KEYS.update({d: d + SHIFTED[d] for d in SHIFTED})
shell_input = queue.Queue()
keyboards = []  # newest last
caps_lock = [False]
lock = threading.Lock()


def new_keyboard(spec):
    return dict({"caps": None, "armed": False}, **spec, events=[], mods=set(),
                keys=[], previous=[])


ps2_kb = new_keyboard({"id": "ps2", "armed": True, "caps": False})


def driver():
    """Turns queued key events into reports (one per poll) and new keys of a
    report into characters for the shell."""
    while True:
        with lock:
            ready = ([ps2_kb] if ps2_kb["events"] else []) + [
                k for k in keyboards if k["armed"] and k["events"]]
        for kb in ready:
            qcode, down = kb["events"].pop(0)
            if qcode in ("shift", "shift_r"):
                (kb["mods"].add if down else kb["mods"].discard)(qcode)
            elif down and qcode not in kb["keys"]:
                kb["keys"].append(qcode)
            elif not down and qcode in kb["keys"]:
                kb["keys"].remove(qcode)
            new = [k for k in kb["keys"] if k not in kb["previous"]]
            kb["previous"] = list(kb["keys"])
            shift = bool(kb["mods"])
            for key in new:
                if key == "caps_lock":
                    if kb["caps"] is None:
                        caps_lock[0] = not caps_lock[0]
                    else:
                        kb["caps"] = not kb["caps"]
                elif len(key) == 1 and key.isalpha():
                    caps = caps_lock[0] if kb["caps"] is None else kb["caps"]
                    shell_input.put(key.upper() if shift != caps else key)
                else:
                    use_shift = shift and BUG != "symbols_unshifted"
                    shell_input.put(KEYS[key][1 if use_shift else 0])
        time.sleep(0.002)


def attach(kb):
    time.sleep(0.05)
    kb["armed"] = True
    log("[USBKBD] keyboard attached: %s port %s (%s)\n" % (
        controllers[kb["ctrl"]][0], xhci_port(kb["port"]), kb["speed"]))


def detach(kb):
    time.sleep(0.02)
    log("[USBKBD] keyboard detached: %s port %s\n" % (
        controllers[kb["ctrl"]][0], xhci_port(kb["port"])))


def shell():
    line = []
    last_cr = False
    log("sh> ")
    while True:
        ch = shell_input.get()
        if ch == "\n" and last_cr:
            last_cr = False
            continue
        last_cr = ch == "\r"
        if ch in "\r\n":
            log("\n")
            words = "".join(line).split()
            line = []
            if words[:1] == ["echo"]:
                log(" ".join(words[1:16]) + "\n")
            elif words == ["help"]:
                log("built-in commands: help echo cpu programs run exit\n"
                    "user programs: init sh usertest\n")
            elif words == ["usertest", "0"]:
                log("sh: usertest exited with status -1250999896491\n")
            elif words[:1] == ["exit"]:
                return
            elif words:
                log("sh: unknown command: %s\n" % words[0])
            log("sh> ")
        elif ch == "\b":
            if line:
                line.pop()
                log("\b \b")
        elif " " <= ch.replace("\t", " ") <= "~" and len(line) < 128:
            line.append(ch.replace("\t", " "))
            log(line[-1])


def serial_server():
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(serial_sock_path)
    server.listen(1)
    while True:
        conn, _ = server.accept()
        poller = select.poll()
        poller.register(conn, select.POLLIN | select.POLLHUP)
        while True:
            events = poller.poll(1000)
            if events and events[0][1] & select.POLLHUP:
                break  # Like QEMU: unread input is lost.
            if events:
                data = conn.recv(14)  # One UART FIFO at a time.
                if not data:
                    break
                for ch in data.decode("latin-1"):
                    shell_input.put(ch)
                time.sleep(0.004)
        conn.close()


def boot():
    with open(os.environ["FAKE_BOOT_LINES"]) as f:
        log(f.read())
    if uefi:
        log("[PMM] mmap entry: base=0x100000000 len=0x40000000 type=1\n"
            "[PMM] Framebuffer: addr=0x80000000 pitch=5120 width=1280 "
            "height=800 bpp=32\n")
    script = "help\necho boot_sh_ok\nusertest 0\n"
    stress = "[STRESS] random_multicpu_stress: PASS\n" * 3
    if not interactive:
        log("[INIT] hello from ring 3\n")
        for ch in script + "exit 0\n":
            shell_input.put(ch)
        shell()
        log("[INIT] exited with status 0\n[TEST] ALL BOOT SELF-TESTS PASSED\n")
        log(stress)
        return
    log("[TEST] ALL BOOT SELF-TESTS PASSED\n[INIT] hello from ring 3\n")
    for ch in script:
        shell_input.put(ch)
    for bdf, mode in controllers.values():
        if mode:
            log("[USBKBD] controller %s ready (%s)\n" % (bdf, mode))
    if BUG == "ps2_prefixed":
        log("sh> [PS2KBD] controller i8042 ready (irq1)\n")
    else:
        log("[PS2KBD] controller i8042 ready (irq1)\n")
    with lock:
        for spec in cold:
            kb = new_keyboard(spec)
            keyboards.append(kb)
            threading.Thread(target=attach, args=(kb,), daemon=True).start()
    shell()
    log("[INIT] exited with status 0\n[TEST] driver_session_cleanup: PASS\n")
    log(stress)


def pci_devices():
    devices, bridge = [], None
    for qid, (bdf, mode) in controllers.items():
        bus, rest = bdf.split(":")
        device = {"bus": int(bus, 16), "slot": int(rest[:2], 16),
                  "function": 0, "qdev_id": qid,
                  "class_info": {"class": 3075 if mode else 1540}}
        if mode is None:
            bridge = device
            device["pci_bridge"] = {"bus": {"number": 1}, "devices": []}
        if bus == "00":
            devices.append(device)
        else:
            bridge["pci_bridge"]["devices"].append(device)
    return [{"bus": 0, "devices": devices}]


def qmp_server():
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(qmp_path)
    server.listen(1)
    while True:
        conn, _ = server.accept()
        send = lambda obj: conn.sendall(json.dumps(obj).encode() + b"\r\n")
        send({"QMP": {"version": {}, "capabilities": []}})
        for raw in conn.makefile("rb"):
            request = json.loads(raw)
            command, args = request["execute"], request.get("arguments", {})
            reply, events = {"return": {}}, []
            if command == "query-pci":
                reply = {"return": pci_devices()}
            elif command == "query-cpus-fast":
                reply = {"return": [{"qom-path": "/cpu%d" % i} for i in range(4)]}
            elif command == "qom-get":
                reply = {"return": (False if args["property"] == "xsaveerptr"
                                    else "AuthenticAMD")}
            elif command == "input-send-event":
                with lock:
                    target = keyboards[-1] if keyboards else ps2_kb
                for e in args["events"]:
                    if len(target["events"]) < 16:
                        target["events"].append((e["data"]["key"]["data"],
                                                 e["data"]["down"]))
            elif command == "device_add":
                ctrl = args["bus"].split(".")[0]
                port_str = str(args["port"])
                if args.get("driver") == "usb-hub":
                    with lock:
                        hubs[args["id"]] = {"id": args["id"], "ctrl": ctrl,
                                            "port": port_str}
                else:
                    is_fs = args.get("usb_version") == 1 or "." in port_str
                    kb = new_keyboard({"id": args["id"], "port": port_str,
                                       "ctrl": ctrl,
                                       "speed": "fs" if is_fs else "hs"})
                    with lock:
                        keyboards.append(kb)
                    threading.Thread(target=attach, args=(kb,),
                                     daemon=True).start()
            elif command == "device_del":
                target_id = args["id"]
                detached_kbs = []
                with lock:
                    if target_id in hubs:
                        h = hubs.pop(target_id)
                        prefix = h["port"] + "."
                        sub_ids = [hid for hid, sub in list(hubs.items())
                                   if sub["ctrl"] == h["ctrl"] and
                                   sub["port"].startswith(prefix)]
                        for hid in sub_ids:
                            hubs.pop(hid)
                        detached_kbs = [k for k in keyboards
                                        if k["ctrl"] == h["ctrl"] and
                                        k["port"].startswith(prefix)]
                        for k in detached_kbs:
                            keyboards.remove(k)
                        for dev_id in [target_id] + sub_ids + [
                                k["id"] for k in detached_kbs]:
                            events.append({"event": "DEVICE_DELETED",
                                           "data": {"device": dev_id}})
                    else:
                        kb = [k for k in keyboards if k["id"] == target_id][0]
                        keyboards.remove(kb)
                        detached_kbs = [kb]
                        events.append({"event": "DEVICE_DELETED",
                                       "data": {"device": kb["id"]}})
                for k in detached_kbs:
                    threading.Thread(target=detach, args=(k,),
                                     daemon=True).start()
            reply["id"] = request.get("id")
            send(reply)
            for event in events:
                send(event)
        conn.close()


def monitor_server():
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(monitor_path)
    server.listen(1)
    while True:
        conn, _ = server.accept()
        words = conn.makefile("rb").readline().decode().split()
        if words[:1] == ["pmemsave"]:
            if BUG == "late_failure":
                log("[STRESS] random_multicpu_stress: FAIL (late) seed=0x1\n")
            size = int(words[2])
            if int(words[1], 0) == 0xB8000:
                data = b"".join(bytes([ord(c), 0x0F])
                                for c in "Hello, x86-64 Kernel World!")
            else:
                data = b"\xff" * size
            with open(words[3], "wb") as f:
                f.write(data[:size])
        conn.close()


for target in (qmp_server, monitor_server, driver, boot):
    threading.Thread(target=target, daemon=True).start()
if serial_sock_path:
    threading.Thread(target=serial_server, daemon=True).start()
time.sleep(600)
'''


def qemu64_boot_lines(script: str) -> list[str]:
  """Lines with every marker that test_boot.sh requires in a qemu64 pass
  (REQUIRED_MARKERS and the non-KVM QEMU64_MARKERS), except those that a
  boot prints at fixed points of its flow (init, self-test end, stress)."""
  values = dict(re.findall(r'^\s*([A-Z0-9_]+)="([^"$]*)"$', script, re.M))
  lines = ["Hello, x86-64 Kernel World!", "[CPU] x87 pointer reset: enabled"]
  for array in ("REQUIRED_MARKERS", "QEMU64_MARKERS"):
    body = re.search(
        r"^\s*%s=\(\n(.*?)^\s*\)$" % array, script, re.M | re.S
    ).group(1)
    for item in re.findall(r'"([^"]*)"', body):
      ref = re.fullmatch(r"\$\{(\w+)\}", item)
      line = values[ref.group(1)] if ref else item
      if not line.startswith(("[INIT]", "[STRESS]", "[TEST] ALL BOOT")):
        lines.append(line)
  return lines


class RegistryTest(unittest.TestCase):

  # The kBootTests table before the USB driver work added tests.
  ORIGINAL_BOOT_TESTS = (
      "pmm_memory_map_init", "pmm_alloc_and_bounds", "pmm_free_and_reuse",
      "heap_varied_sizes_and_alignment", "heap_pattern_isolation",
      "cpp_new_delete_lifecycle", "heap_stress_reuse", "edge_cases_and_oom",
      "pmm_heap_exhaustion", "paging_edge_cases", "rbtree_edge_cases",
      "multiboot_mmap_edge_cases", "smp_discovery_and_ap_bringup",
      "smp_multicpu_sync_stress", "idt_percpu_and_lapic_timer",
      "task_scheduler_smp", "tlb_shootdown", "task_stack_guard_pages",
      "address_spaces", "address_space_switching", "user_mode_programs",
      "user_mode_preemption", "user_mode_preemptible_syscalls",
      "user_mode_kill", "user_mode_fpu_isolation",
      "user_mode_fpu_initial_state", "cpu_protection_features",
      "smap_smep_protection", "user_mode_kill_edge_cases",
      "user_mode_syscall_buffer_checks", "user_mode_fpu_lifecycle",
      "user_mode_mixed_workload",
  )

  def test_load_real_boot_test_names(self) -> None:
    names = deflake.load_boot_test_names(REPO_ROOT)
    # Every entry of the table parses (counted here independently of the
    # parser's regex). The table only grows (it has at least 33 tests), and the
    # original tests keep their order.
    source = (REPO_ROOT / "selftest" / "boot_selftest.cpp").read_text(
        encoding="utf-8"
    )
    table = source.split("constexpr BootTest kBootTests[] = {", 1)[1]
    entries = [
        line for line in table.split("};", 1)[0].splitlines()
        if line.strip().startswith('{"')
    ]
    self.assertEqual(len(names), len(entries))
    self.assertGreaterEqual(len(names), 33)
    self.assertEqual(names[0], "pmm_memory_map_init")
    for name in self.ORIGINAL_BOOT_TESTS:
      self.assertIn(name, names)
    positions = [names.index(name) for name in self.ORIGINAL_BOOT_TESTS]
    self.assertEqual(positions, sorted(positions))
    self.assertEqual(len(names), len(set(names)))

  def test_load_boot_test_names_errors_on_corrupt_source(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      with self.assertRaises(ValueError):
        deflake.load_boot_test_names(root)
      (root / "selftest").mkdir()
      (root / "selftest" / "boot_selftest.cpp").write_text(
          "int main() { return 0; }\n", encoding="utf-8"
      )
      with self.assertRaises(ValueError):
        deflake.load_boot_test_names(root)


class SerialParserTest(unittest.TestCase):

  def setUp(self) -> None:
    self.names = deflake.load_boot_test_names(REPO_ROOT)

  def test_line_endings_and_chunked_streaming_match(self) -> None:
    raw_lf = make_serial_log(
        names=self.names,
        fail_test="user_mode_kill",
        injected_seed="0x0000000000000028",
        crlf="\n",
    )
    raw_crlf = make_serial_log(
        names=self.names,
        fail_test="user_mode_kill",
        injected_seed="0x0000000000000028",
        crlf="\r\n",
    )
    raw_cr2lf = make_serial_log(
        names=self.names,
        fail_test="user_mode_kill",
        injected_seed="0x0000000000000028",
        crlf="\r\r\n",
    )
    rep_lf = deflake.parse_serial(raw_lf, self.names)
    rep_crlf = deflake.parse_serial(raw_crlf, self.names)
    rep_cr2lf = deflake.parse_serial(raw_cr2lf, self.names)
    self.assertEqual(rep_lf.results, rep_crlf.results)
    self.assertEqual(rep_lf.results, rep_cr2lf.results)
    self.assertEqual(rep_lf.seeds, rep_crlf.seeds)
    self.assertEqual(rep_lf.failure_lines, rep_crlf.failure_lines)

    rng = random.Random(0xC0FFEE)
    parser = deflake.SerialParser(self.names)
    idx = 0
    while idx < len(raw_crlf):
      step = rng.randint(1, 37)
      parser.feed(raw_crlf[idx : idx + step])
      idx += step
    rep_stream = parser.finish()
    self.assertEqual(rep_lf.results, rep_stream.results)
    self.assertEqual(rep_lf.seeds, rep_stream.seeds)
    self.assertEqual(rep_lf.failure_lines, rep_stream.failure_lines)

  def test_partial_last_line_and_report_lines_ignored_as_test_results(
      self,
  ) -> None:
    text = (
        "Hello, x86-64 Kernel World!\r\n"
        "[TEST] user_mode_kill: spinners=4 killed=3\r\n"
        "[TEST] wait for processes timed out: FAIL\r\n"
        "[TEST] rbtree_edge_cases: injected flake one_in=20 seed=0x1234abcd\r\n"
        "[TEST] expectation failed: x == y at foo.cpp:10\r\n"
        "[TEST] some interleaved prefix user_mode_kill: FAIL"
    )
    rep = deflake.parse_serial(text.encode("latin-1"), self.names)
    self.assertEqual(rep.results, {"user_mode_kill": ["FAIL"]})
    self.assertEqual(rep.garbled, {"user_mode_kill"})
    self.assertEqual([s for _, s, _ in rep.seeds], ["0x1234abcd"])
    self.assertEqual(
        [ln for _, _, ln in rep.seeds],
        ["[TEST] rbtree_edge_cases: injected flake one_in=20 seed=0x1234abcd"],
    )
    # `expectation failed` (lowercase) does not match FAILURE_RE ('FAIL|\[FATAL\]').
    self.assertTrue(
        all("expectation failed" not in ln for _, ln in rep.failure_lines)
    )
    self.assertEqual(len(rep.failure_lines), 2)

  def test_baseline_failure_excerpts(self) -> None:
    excerpt = (
        "Hello, x86-64 Kernel World!\r\n"
        "[SMP] Online CPUs: 24/24\r\n"
        "[TEST] pmm_memory_map_init: PASS\r\n"
        "[TEST] wait for processes timed out: FAIL\r\n"
        "[TEST] expectation failed: WaitForProcesses(pids, 4) at"
        " selftest/boot_selftest.cpp:5120\r\n"
        "[TEST] user_mode_kill: FAIL\r\n"
        "[TEST] user_mode_mixed_workload: PASS\r\n"
        "[TEST] BOOT SELF-TESTS FAILED\r\n"
    ).encode("latin-1")
    rep = deflake.parse_serial(excerpt, self.names)
    self.assertEqual(rep.results["user_mode_kill"], ["FAIL"])
    self.assertEqual(rep.suite_end, "failed")


class VerdictAndAccountingTest(unittest.TestCase):

  def setUp(self) -> None:
    self.names = deflake.load_boot_test_names(REPO_ROOT)

  def _end(self, **kwargs: object) -> deflake.BootEnd:
    defaults: dict[str, object] = {
        "t_start": 0.0,
        "t_pass": 4.5,
        "t_fail": None,
        "exited_rc": None,
        "timed_out": False,
        "stopped_by_tool": True,
    }
    defaults.update(kwargs)
    return deflake.BootEnd(**defaults)  # type: ignore[arg-type]

  def test_judge_qemu_boot_pass_and_each_failure_reason(self) -> None:
    rep_ok = deflake.parse_serial(make_serial_log(names=self.names), self.names)
    v_ok = deflake.judge_qemu_boot(
        rep_ok, self._end(), 24, 60.0, registry=self.names
    )
    self.assertEqual(v_ok.status, "PASS")
    self.assertEqual(v_ok.reasons, ())

    # 1. Timeout (only 2 stress passes).
    rep_2stress = deflake.parse_serial(
        make_serial_log(names=self.names, stress_passes=2), self.names
    )
    v_to = deflake.judge_qemu_boot(
        rep_2stress,
        self._end(t_pass=None, timed_out=True, stopped_by_tool=False),
        24,
        60.0,
        registry=self.names,
    )
    self.assertEqual(v_to.status, "FAIL")
    self.assertIn("timeout", v_to.reasons)

    # 2. QEMU self-exit before tool stopped it.
    v_exit = deflake.judge_qemu_boot(
        rep_ok,
        self._end(exited_rc=0, stopped_by_tool=False),
        24,
        60.0,
        registry=self.names,
    )
    self.assertEqual(v_exit.status, "FAIL")
    self.assertIn("qemu_exited:0", v_exit.reasons)

    # 3. Reboot (2 greetings).
    rep_reboot = deflake.parse_serial(
        make_serial_log(names=self.names, greetings=2), self.names
    )
    v_reb = deflake.judge_qemu_boot(
        rep_reboot, self._end(), 24, 60.0, registry=self.names
    )
    self.assertEqual(v_reb.status, "FAIL")
    self.assertIn("reboot", v_reb.reasons)

    # 4. Wrong CPU count or missing CPU line.
    rep_23 = deflake.parse_serial(
        make_serial_log(names=self.names, cpus=(23, 23)), self.names
    )
    v_23 = deflake.judge_qemu_boot(
        rep_23, self._end(), 24, 60.0, registry=self.names
    )
    self.assertEqual(v_23.status, "FAIL")
    self.assertIn("cpus_online", v_23.reasons)

    rep_nocpu = deflake.parse_serial(
        make_serial_log(names=self.names, cpus=None), self.names
    )
    v_nocpu = deflake.judge_qemu_boot(
        rep_nocpu, self._end(), 24, 60.0, registry=self.names
    )
    self.assertEqual(v_nocpu.status, "FAIL")
    self.assertIn("cpus_online", v_nocpu.reasons)

    # 5. No greeting.
    rep_nogreet = deflake.parse_serial(
        make_serial_log(names=self.names, greetings=0), self.names
    )
    v_nogreet = deflake.judge_qemu_boot(
        rep_nogreet, self._end(), 24, 60.0, registry=self.names
    )
    self.assertEqual(v_nogreet.status, "FAIL")
    self.assertIn("no_greeting", v_nogreet.reasons)

    # 6. Single test failed.
    rep_tfail = deflake.parse_serial(
        make_serial_log(names=self.names, fail_test="rbtree_edge_cases"),
        self.names,
    )
    v_tfail = deflake.judge_qemu_boot(
        rep_tfail,
        self._end(t_pass=None, t_fail=3.0),
        24,
        60.0,
        registry=self.names,
    )
    self.assertEqual(v_tfail.status, "FAIL")
    self.assertIn("test_failed:rbtree_edge_cases", v_tfail.reasons)

    # 7. Stress failed.
    rep_sfail = deflake.parse_serial(
        make_serial_log(names=self.names, stress_fail=True), self.names
    )
    v_sfail = deflake.judge_qemu_boot(
        rep_sfail,
        self._end(t_pass=None, t_fail=5.0),
        24,
        60.0,
        registry=self.names,
    )
    self.assertEqual(v_sfail.status, "FAIL")
    self.assertIn("stress_failed", v_sfail.reasons)

    # 8. Panic.
    rep_panic = deflake.parse_serial(
        make_serial_log(names=self.names, fatal_after="rbtree_edge_cases"),
        self.names,
    )
    v_panic = deflake.judge_qemu_boot(
        rep_panic,
        self._end(t_pass=None, t_fail=2.0),
        24,
        60.0,
        registry=self.names,
    )
    self.assertEqual(v_panic.status, "FAIL")
    self.assertIn("panic", v_panic.reasons)

    # 9. t_pass > timeout.
    v_late = deflake.judge_qemu_boot(
        rep_ok, self._end(t_pass=61.0), 24, 60.0, registry=self.names
    )
    self.assertEqual(v_late.status, "FAIL")
    self.assertIn("timeout", v_late.reasons)

  def test_judge_test_boot_matrix(self) -> None:
    s_ok = (
        "SUCCESS: boot verification passed (bios: PASS uefi: PASS mb1: PASS"
        " max: PASS smep: PASS): greeting, all boot self-tests, 3+ stress-test"
        " passes, and no panic, triple fault, or reboot."
    )
    s_fail = "FAILURE: boot verification failed (max: FAIL)."
    self.assertEqual(
        deflake.parse_test_boot_summary(s_ok),
        {
            "bios": "PASS",
            "uefi": "PASS",
            "mb1": "PASS",
            "max": "PASS",
            "smep": "PASS",
        },
    )
    self.assertEqual(deflake.parse_test_boot_summary(s_fail), {"max": "FAIL"})

    rep_ok = deflake.parse_serial(
        make_serial_log(names=self.names, cpus=(4, 4)), self.names
    )
    out_log = (
        "FAIL: Missing required marker '[CPU] x87 pointer reset: enabled'\n"
        + s_fail
    )
    # Even if serial log is 100% clean, if test_boot.sh says FAIL (e.g. x87 or VGA check), verdict MUST be FAIL.
    v_harness_fail = deflake.judge_test_boot(
        result_text="FAIL",
        exit_code=1,
        summary_status="FAIL",
        output_log=out_log,
        report=rep_ok,
        registry=self.names,
    )
    self.assertEqual(v_harness_fail.status, "FAIL")
    outcomes = deflake.test_outcomes(
        rep_ok,
        v_harness_fail,
        self.names,
        engine="test_boot",
        output_log=out_log,
    )
    self.assertEqual(outcomes["@harness"], "FAIL")

    # PASS case.
    v_pass = deflake.judge_test_boot(
        result_text="PASS",
        exit_code=0,
        summary_status="PASS",
        output_log=s_ok,
        report=rep_ok,
        registry=self.names,
    )
    self.assertEqual(v_pass.status, "PASS")

    # SKIPPED case.
    v_skip = deflake.judge_test_boot(
        result_text="SKIPPED",
        exit_code=0,
        summary_status="SKIPPED",
        output_log="SUCCESS: boot verification passed (uefi: SKIPPED).",
        report=deflake.parse_serial(b"", self.names),
        registry=self.names,
    )
    self.assertEqual(v_skip.status, "SKIPPED")

    # Missing result file & summary -> ERROR.
    v_err = deflake.judge_test_boot(
        result_text=None,
        exit_code=125,
        summary_status=None,
        output_log="docker: daemon error",
        report=deflake.parse_serial(b"", self.names),
        registry=self.names,
    )
    self.assertEqual(v_err.status, "ERROR")

  def test_accounting_injected_flake_and_panic_phase(self) -> None:
    rep_inj = deflake.parse_serial(
        make_serial_log(
            names=self.names,
            fail_test="rbtree_edge_cases",
            injected_seed="0x0000000000000028",
        ),
        self.names,
    )
    v_inj = deflake.judge_qemu_boot(
        rep_inj,
        self._end(t_pass=None, t_fail=3.0),
        24,
        60.0,
        registry=self.names,
    )
    outcomes = deflake.test_outcomes(
        rep_inj, v_inj, self.names, engine="qemu"
    )
    self.assertEqual(outcomes["rbtree_edge_cases"], "FAIL")
    self.assertEqual(outcomes["@boot"], "PASS")
    self.assertEqual(outcomes["@panic"], "PASS")
    self.assertEqual(outcomes["random_multicpu_stress"], "NO_RESULT")
    for name in self.names:
      if name != "rbtree_edge_cases":
        self.assertEqual(outcomes[name], "PASS")

    # Panic after rbtree_edge_cases -> phase is during next test.
    idx = self.names.index("rbtree_edge_cases")
    next_test = self.names[idx + 1]
    rep_panic = deflake.parse_serial(
        make_serial_log(names=self.names, fatal_after="rbtree_edge_cases"),
        self.names,
    )
    self.assertEqual(
        deflake.phase_of(rep_panic, self.names), f"during {next_test}"
    )
    v_panic = deflake.judge_qemu_boot(
        rep_panic,
        self._end(t_pass=None, t_fail=2.0),
        24,
        60.0,
        registry=self.names,
    )
    outcomes_panic = deflake.test_outcomes(
        rep_panic, v_panic, self.names, engine="qemu"
    )
    self.assertEqual(outcomes_panic["@panic"], "FAIL")
    self.assertEqual(outcomes_panic[next_test], "NO_RESULT")


class StatsAndR3SimulationTest(unittest.TestCase):

  def test_wilson_and_classify(self) -> None:
    self.assertIsNone(deflake.wilson(0, 0))
    lo, hi = deflake.wilson(8, 40)  # type: ignore[misc]
    self.assertAlmostEqual(lo, 0.1050, places=4)
    self.assertAlmostEqual(hi, 0.3476, places=4)
    lo0, hi0 = deflake.wilson(0, 200)  # type: ignore[misc]
    self.assertAlmostEqual(lo0, 0.0, places=4)
    self.assertAlmostEqual(hi0, 0.0188, places=4)
    lo10, hi10 = deflake.wilson(10, 200)  # type: ignore[misc]
    self.assertAlmostEqual(lo10, 0.0274, places=4)
    self.assertAlmostEqual(hi10, 0.0896, places=4)

    self.assertEqual(deflake.classify(0, 0), "no_data")
    self.assertEqual(deflake.classify(10, 0), "stable")
    self.assertEqual(deflake.classify(9, 1), "flaky")
    self.assertEqual(deflake.classify(0, 5), "broken")

  def test_r3_200_boot_injected_and_clean_simulation(self) -> None:
    names = deflake.load_boot_test_names(REPO_ROOT)
    cfg = deflake.Config(
        name="kvm24",
        engine="qemu",
        qemu_argv=deflake.KVM24_ARGV,
        expected_cpus=24,
        timeout_s=60.0,
    )
    build_inj = {
        "build_id": "binj12345678",
        "defines": ["-DPROTOS_SELFTEST=1", "-DPROTOS_INJECT_FLAKY_TEST=20"],
        "kernel_iso": {"sha256": "a" * 64},
        "grub": "GRUB 2.14-1",
        "toolchain": "host",
        "vcs": {"tool": None},
    }
    build_clean = {
        "build_id": "bcln12345678",
        "defines": ["-DPROTOS_SELFTEST=1"],
        "kernel_iso": {"sha256": "b" * 64},
        "grub": "GRUB 2.14-1",
        "toolchain": "host",
        "vcs": {"tool": None},
    }

    def make_records(
        build_id: str, inject: bool
    ) -> list[deflake.RunRecord]:
      recs: list[deflake.RunRecord] = []
      for i in range(1, 201):
        seed_int = (i * 20) if (inject and i <= 10) else (i * 20 + 1)
        seed_hex = f"0x{seed_int:016x}"
        fires = inject and (seed_int % 20 == 0)
        outcomes = {t: "PASS" for t in names}
        outcomes["@boot"] = "PASS"
        outcomes["@panic"] = "PASS"
        if fires:
          outcomes["rbtree_edge_cases"] = "FAIL"
          outcomes["random_multicpu_stress"] = "NO_RESULT"
          status = "FAIL"
          reasons = ("test_failed:rbtree_edge_cases",)
          failed_tests = ["rbtree_edge_cases"]
        else:
          outcomes["random_multicpu_stress"] = "PASS"
          status = "PASS"
          reasons = ()
          failed_tests = []
        recs.append(
            deflake.RunRecord(
                config="kvm24",
                index=i,
                build=build_id,
                status=status,
                reasons=reasons,
                outcomes=outcomes,
                failed_tests=failed_tests,
                phase="stress pass 4" if not fires else "after suite (failed)",
                seeds=[seed_hex],
                seed_lines=[
                    (
                        20,
                        seed_hex,
                        f"[TEST] rbtree_edge_cases: injected flake one_in=20"
                        f" seed={seed_hex}",
                    )
                ]
                if inject
                else [(40, seed_hex, f"[STRESS] seed={seed_hex} pass=1")],
                failure_lines=[],
                expectations=[],
                diagnostics=[],
                duration_s=9.5,
                start_utc="2026-10-20T18:00:00Z",
                lock_wait_s=0.0,
                loadavg_before=[0.5, 0.5, 0.5],
                qemu_argv=deflake.KVM24_ARGV,
                qemu_cwd=str(REPO_ROOT),
                harness_argv=(),
                env_recorded={},
                nondefault_harness_env={},
                iso_snapshot="/tmp/inj.iso",
                mb1_iso_snapshot=None,
                exit_code=0,
                t_pass=9.0 if not fires else None,
                t_fail=3.0 if fires else None,
                artifact_dir=f"runs/kvm24/{i:04d}" if fires else None,
            )
        )
      return recs

    inj_summary = deflake.aggregate(
        records=make_records("binj12345678", True),
        configs=[cfg],
        builds={"binj12345678": build_inj},
        registry=names,
        session_meta={"stop_reason": "runs"},
    )
    self.assertEqual(inj_summary["schema"], "protos-deflake/1")
    self.assertEqual(len(inj_summary["flaky"]), 1)
    self.assertEqual(inj_summary["flaky"][0]["test"], "rbtree_edge_cases")
    self.assertEqual(inj_summary["flaky"][0]["failed"], 10)
    self.assertEqual(inj_summary["flaky"][0]["passed"], 190)
    self.assertEqual(inj_summary["broken"], [])

    clean_summary = deflake.aggregate(
        records=make_records("bcln12345678", False),
        configs=[cfg],
        builds={"bcln12345678": build_clean},
        registry=names,
        session_meta={"stop_reason": "runs"},
    )
    self.assertEqual(clean_summary["flaky"], [])
    self.assertEqual(clean_summary["broken"], [])


class ConfigsAndCliTest(unittest.TestCase):

  def test_expected_cpus(self) -> None:
    self.assertEqual(deflake.expected_cpus(["qemu", "-smp", "12"]), 12)
    self.assertEqual(
        deflake.expected_cpus(["qemu", "-smp", "cpus=12,sockets=1"]), 12
    )
    self.assertEqual(
        deflake.expected_cpus(["qemu", "-smp", "sockets=2,cores=3,threads=2"]),
        12,
    )
    self.assertEqual(
        deflake.expected_cpus(["qemu", "-smp", "4,maxcpus=8"]), 4
    )
    self.assertEqual(deflake.expected_cpus(["qemu"]), 1)
    with self.assertRaises(ValueError):
      deflake.expected_cpus(["qemu", "-smp", "0"])
    with self.assertRaises(ValueError):
      deflake.expected_cpus(["qemu", "-smp", "bogus"])

  def test_apply_qemu_overrides_and_cpu_host_guard(self) -> None:
    argv = deflake.apply_qemu_overrides(deflake.KVM24_ARGV, "-smp 12 -m 512M")
    self.assertEqual(deflake.expected_cpus(argv), 12)
    self.assertIn("512M", argv)
    self.assertNotIn("256M", argv)

    # -accel tcg drops -enable-kvm; must also change -cpu host or it's rejected.
    with self.assertRaises(ValueError):
      deflake.apply_qemu_overrides(deflake.KVM24_ARGV, "-accel tcg")

    argv_tcg = deflake.apply_qemu_overrides(
        deflake.KVM24_ARGV, "-accel tcg -cpu max -smp 4"
    )
    self.assertNotIn("-enable-kvm", argv_tcg)
    self.assertIn("tcg", argv_tcg)
    self.assertIn("max", argv_tcg)
    self.assertEqual(deflake.expected_cpus(argv_tcg), 4)

    for forbidden in (
        "-cdrom foo.iso",
        "-serial file:x",
        "-monitor stdio",
        "-nographic",
        "-daemonize",
    ):
      with self.assertRaises(ValueError):
        deflake.apply_qemu_overrides(deflake.KVM24_ARGV, forbidden)

  def test_expand_configs_and_timeout_restrictions(self) -> None:
    parser = deflake.build_arg_parser()
    args_all = parser.parse_args(["--config", "all", "--runs", "1"])
    cfgs_all = deflake.expand_configs(
        config_specs=args_all.config,
        qemu_args_list=args_all.qemu_args,
        qemu_cmd_list=args_all.qemu_cmd,
        timeout_specs=args_all.timeout,
    )
    self.assertEqual(
        [c.name for c in cfgs_all], list(deflake.BUILTIN_CONFIG_NAMES)
    )

    # --timeout is rejected on kvm24, docker:*, host:*.
    for bad_argv in (
        ["--config", "kvm24", "--timeout", "120", "--runs", "1"],
        ["--config", "kvm24", "--timeout", "kvm24=120", "--runs", "1"],
        ["--config", "docker:uefi", "--timeout", "docker:uefi=120", "--runs", "1"],
        ["--config", "host:max", "--timeout", "host:max=120", "--runs", "1"],
    ):
      bad_args = parser.parse_args(bad_argv)
      with self.assertRaises(ValueError):
        deflake.expand_configs(
            config_specs=bad_args.config,
            qemu_args_list=bad_args.qemu_args,
            qemu_cmd_list=bad_args.qemu_cmd,
            timeout_specs=bad_args.timeout,
        )

    # --timeout is allowed on custom configs.
    args_custom = parser.parse_args([
        "--qemu-args",
        "-accel tcg -cpu max -smp 4",
        "--timeout",
        "custom1=180",
        "--runs",
        "1",
    ])
    cfgs_custom = deflake.expand_configs(
        config_specs=args_custom.config,
        qemu_args_list=args_custom.qemu_args,
        qemu_cmd_list=args_custom.qemu_cmd,
        timeout_specs=args_custom.timeout,
    )
    self.assertEqual(len(cfgs_custom), 1)
    self.assertEqual(cfgs_custom[0].name, "custom1")
    self.assertEqual(cfgs_custom[0].timeout_s, 180.0)

  def test_parse_duration(self) -> None:
    self.assertEqual(deflake.parse_duration("90s"), 90.0)
    self.assertEqual(deflake.parse_duration("30m"), 1800.0)
    self.assertEqual(deflake.parse_duration("1h30m"), 5400.0)
    with self.assertRaises(ValueError):
      deflake.parse_duration("0s")
    with self.assertRaises(ValueError):
      deflake.parse_duration("abc")

  def test_harness_watchdog_covers_test_boot_bounds(self) -> None:
    bios = deflake._make_builtin_config("host:bios")
    uefi = deflake._make_builtin_config("docker:uefi")
    smep = deflake._make_builtin_config("host:smep")
    # The USB keyboard scenarios of bios and uefi add their budget (60 s by
    # default) to the pass timeout; the other passes have none.
    self.assertEqual(deflake.harness_watchdog_s(bios, {}), 60 + 60 + 120)
    self.assertEqual(deflake.harness_watchdog_s(uefi, {}), 90 + 60 + 120)
    self.assertEqual(deflake.harness_watchdog_s(smep, {}), 60 + 120)
    self.assertEqual(
        deflake.harness_watchdog_s(bios, {"BOOT_TEST_USB": "0"}), 60 + 120
    )
    self.assertEqual(
        deflake.harness_watchdog_s(uefi, {"BOOT_TEST_USB": ""}), 90 + 60 + 120
    )
    self.assertEqual(
        deflake.harness_watchdog_s(
            uefi, {"USB_SESSION_TIMEOUT_SECS": "200", "UEFI_TIMEOUT_SECS": "300"}
        ),
        300 + 200 + 120,
    )
    # A lower or invalid timeout in the environment never shortens the
    # watchdog below the acceptance timeouts.
    self.assertEqual(
        deflake.harness_watchdog_s(
            bios, {"BIOS_TIMEOUT_SECS": "5", "USB_SESSION_TIMEOUT_SECS": "x"}
        ),
        60 + 60 + 120,
    )
    self.assertEqual(
        deflake.harness_watchdog_s(smep, {"BIOS_TIMEOUT_SECS": "100"}), 100 + 120
    )

  def test_duplicated_lists_match_makefile_and_test_boot_sh(self) -> None:
    makefile = (REPO_ROOT / "Makefile").read_text(encoding="utf-8")
    script = (REPO_ROOT / "test_boot.sh").read_text(encoding="utf-8")
    # Docker forwards exactly the variables of the Makefile's list; deflake.py
    # forwards (docker:<pass>) and records the same ones.
    env_vars = re.search(
        r"^BOOT_TEST_ENV_VARS := (.*)$", makefile, re.M
    ).group(1).split()
    self.assertEqual(env_vars, list(deflake.BOOT_TEST_ENV_VARS))
    for var in env_vars:
      self.assertIn("${%s" % var, script)
    # deflake's host:<pass>/docker:<pass> configurations name test_boot.sh's
    # passes.
    pass_names = re.search(r"^PASS_NAMES=\((.*)\)$", script, re.M).group(1)
    self.assertEqual(tuple(pass_names.split()), deflake.PASS_NAMES)
    # The bios pass verifies the USB devices of `make run`: test_boot.sh's
    # default must equal the Makefile's (checked once the Makefile has one).
    default = re.search(
        r'^QEMU_USB_FLAGS_DEFAULT="([^"]*)"$', script, re.M
    ).group(1)
    self.assertEqual(
        default,
        "-device qemu-xhci,id=xhci0 -device usb-kbd,bus=xhci0.0,port=1,id=kbd0",
    )
    in_makefile = re.search(r"^QEMU_USB_FLAGS \?= (.*)$", makefile, re.M)
    if in_makefile:
      self.assertEqual(in_makefile.group(1).split(), default.split())


class LockAndRunnersTest(unittest.TestCase):

  def setUp(self) -> None:
    self.names = deflake.load_boot_test_names(REPO_ROOT)

  def test_host_lock_acquires_and_releases_flock(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      lock_path = Path(tmp) / "test.lock"
      with deflake.HostLock(lock_path) as hl:
        self.assertGreaterEqual(hl.wait_s, 0.0)
        fd2 = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o666)
        try:
          with self.assertRaises(BlockingIOError):
            fcntl.flock(fd2, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
          os.close(fd2)
      # After context exit, lock is free.
      fd3 = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o666)
      try:
        fcntl.flock(fd3, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd3, fcntl.LOCK_UN)
      finally:
        os.close(fd3)

      # Per-user default_lock_path honors XDG_RUNTIME_DIR and UID fallback.
      self.assertEqual(
          deflake.default_lock_path({"XDG_RUNTIME_DIR": "/run/user/9999"}),
          Path("/run/user/9999/protos_deflake/host.lock"),
      )
      self.assertEqual(
          deflake.default_lock_path({}),
          Path(f"/tmp/protos_deflake-{os.getuid()}/host.lock"),
      )

      # Symlinked lock file or symlinked lock directory is rejected.
      target_file = Path(tmp) / "real.lock"
      target_file.write_text("", encoding="utf-8")
      symlink_lock = Path(tmp) / "sym.lock"
      symlink_lock.symlink_to(target_file)
      if hasattr(os, "O_NOFOLLOW"):
        with self.assertRaises(OSError):
          with deflake.HostLock(symlink_lock):
            pass

      real_dir = Path(tmp) / "real_dir"
      real_dir.mkdir()
      sym_dir = Path(tmp) / "sym_dir"
      sym_dir.symlink_to(real_dir)
      with self.assertRaises(OSError):
        with deflake.HostLock(sym_dir / "host.lock"):
          pass

  def test_vcs_provenance_git_fallback_and_toolchain_detection(self) -> None:
    if shutil.which("git") is None:
      self.skipTest("git not found")
    # git runs here and in vcs_provenance without the developer's git
    # configuration, which may sign commits or run hooks (the -c options cover
    # git older than 2.32, which ignores GIT_CONFIG_GLOBAL), and without the
    # GIT_* variables that may name another repository.
    git_env = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("GIT_")
    }
    git_env.update({
        "GIT_CONFIG_GLOBAL": os.devnull,
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_AUTHOR_NAME": "Test",
        "GIT_AUTHOR_EMAIL": "test@example.com",
        "GIT_COMMITTER_NAME": "Test",
        "GIT_COMMITTER_EMAIL": "test@example.com",
    })
    git = [
        "git", "-c", "commit.gpgsign=false", "-c", "core.hooksPath=/dev/null"
    ]
    with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(
        os.environ, git_env, clear=True
    ):
      repo = Path(tmp) / "git_repo"
      repo.mkdir()
      subprocess.run([*git, "init", "-q"], cwd=repo, check=True)
      (repo / "README.md").write_text("hello\n", encoding="utf-8")
      subprocess.run([*git, "add", "README.md"], cwd=repo, check=True)
      subprocess.run(
          [*git, "commit", "-q", "-m", "initial"], cwd=repo, check=True
      )
      prov_clean = deflake.vcs_provenance(repo)
      self.assertEqual(prov_clean["tool"], "git")
      self.assertFalse(prov_clean["dirty"])
      self.assertTrue(bool(prov_clean["commit_id"]))

      (repo / "README.md").write_text("modified\n", encoding="utf-8")
      prov_dirty = deflake.vcs_provenance(repo)
      self.assertEqual(prov_dirty["tool"], "git")
      self.assertTrue(prov_dirty["dirty"])

      # Host toolchain classification handles Ubuntu 24.04 / Debian 13 (GRUB
      # 2.12 + GCC 13/14) as "host" and Debian bookworm (GRUB 2.06 + Debian 12)
      # as "docker".
      build_dir = repo / "build"
      build_dir.mkdir()
      (build_dir / "kernel.bin").write_bytes(b"FAKEKERNELBIN")
      (build_dir / "kernel.o").write_bytes(
          b"\x00GCC: (Ubuntu 13.3.0-6ubuntu2~24.04) 13.3.0\x00"
      )
      (build_dir / ".kernel-config").write_text(
          "g++ -DPROTOS_SELFTEST=1 | as | build/kernel.o | g++ | ld\n",
          encoding="utf-8",
      )
      host_iso = build_dir / "kernel.iso"
      host_iso.write_bytes(b"ISO-FAKEKERNELBIN-GRUB 2.12-1ubuntu7-END")
      b_host = deflake.build_identity(host_iso, host_iso, repo)
      self.assertEqual(b_host["toolchain"], "host")
      self.assertEqual(b_host["vcs"]["tool"], "git")

      summary = deflake.aggregate(
          records=[],
          configs=[deflake._make_builtin_config("kvm24")],
          builds={b_host["build_id"]: b_host},
          registry=("pmm_memory_map_init",),
          session_meta={"stop_reason": "runs"},
      )
      rendered = deflake.render_text(summary, ("pmm_memory_map_init",))
      self.assertIn(f"git {prov_dirty['commit_id'][:8]} (dirty)", rendered)

      (build_dir / "kernel.o").write_bytes(
          b"\x00GCC: (Debian 12.2.0-14+deb12u1) 12.2.0\x00"
      )
      docker_iso = build_dir / "docker.iso"
      docker_iso.write_bytes(b"ISO-FAKEKERNELBIN-GRUB 2.06-13+deb12u1-END")
      b_docker = deflake.build_identity(docker_iso, docker_iso, repo)
      self.assertEqual(b_docker["toolchain"], "docker")

  def test_docker_env_and_kvm_forwarding(self) -> None:
    self.assertEqual(deflake.docker_kvm_args({}, kvm_gid=lambda: 108), ())
    self.assertEqual(
        deflake.docker_kvm_args({"KVM": "1"}, kvm_gid=lambda: 108),
        ("--device", "/dev/kvm", "-e", "KVM=1", "--group-add", "108"),
    )
    self.assertEqual(
        deflake.docker_kvm_args({"KVM": "1"}, kvm_gid=lambda: None),
        ("--device", "/dev/kvm", "-e", "KVM=1"),
    )
    args = deflake.build_arg_parser().parse_args([
        "--config", "docker:bios", "--dry-run", "--iso", "k.iso",
        "--mb1-iso", "mb1.iso",
    ])
    env = {
        "DOCKER": "podman",
        "DOCKER_IMAGE": "custom-kernel-img",
        "KVM": "1",
        "HOST_UID": "0",
        "HOST_GID": "0",
        "DOCKER_VOLUME_OPTS": "",
    }
    buf = io.StringIO()
    with mock.patch.dict(os.environ, env, clear=False):
      with contextlib.redirect_stdout(buf):
        rc = deflake.run_session(args, repo=REPO_ROOT)
    self.assertEqual(rc, 0)
    out = buf.getvalue()
    self.assertIn("podman run --rm", out)
    self.assertIn("custom-kernel-img", out)
    self.assertIn("--device /dev/kvm -e KVM=1", out)

  def test_run_qemu_boot_with_fake_qemu(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      fake_qemu = tmp_path / "qemu-system-x86_64"
      iso_snap = tmp_path / "kernel.iso"
      iso_snap.write_bytes(b"fake-iso")
      pass_log = make_serial_log(names=self.names, cpus=(4, 4))
      fail_early = (
          b"Hello, x86-64 Kernel World!\r\n"
          b"[SMP] Online CPUs: 4/4\r\n"
          b"[TEST] pmm_memory_map_init: FAIL\r\n"
      )
      fail_late = (
          b"[TEST] user_mode_mixed_workload: PASS\r\n"
          b"[TEST] BOOT SELF-TESTS FAILED\r\n"
      )
      fake_qemu.write_text(
          textwrap.dedent(f"""\
          #!/usr/bin/env python3
          import sys, time
          mode = sys.argv[1]
          if mode == "pass":
              sys.stdout.buffer.write({pass_log!r})
              sys.stdout.buffer.flush()
              time.sleep(30)
          elif mode == "timeout":
              sys.stdout.buffer.write(b"Hello, x86-64 Kernel World!\\r\\n")
              sys.stdout.buffer.flush()
              time.sleep(30)
          elif mode == "exit1":
              sys.stdout.buffer.write(b"Hello, x86-64 Kernel World!\\r\\n")
              sys.stdout.buffer.flush()
              sys.exit(1)
          elif mode == "drain":
              sys.stdout.buffer.write({fail_early!r})
              sys.stdout.buffer.flush()
              time.sleep(0.2)
              sys.stdout.buffer.write({fail_late!r})
              sys.stdout.buffer.flush()
              time.sleep(30)
          elif mode == "late_fatal":
              sys.stdout.buffer.write({pass_log!r})
              sys.stdout.buffer.flush()
              time.sleep(0.1)
              sys.stdout.buffer.write(b"[FATAL] late panic in grace window\\r\\n")
              sys.stdout.buffer.flush()
              time.sleep(30)
          """),
          encoding="utf-8",
      )
      fake_qemu.chmod(0o755)

      def make_cfg(mode: str, timeout_s: float = 5.0) -> deflake.Config:
        return deflake.Config(
            name=f"custom_{mode}",
            engine="qemu",
            qemu_argv=(str(fake_qemu), mode, "-cdrom", deflake.ISO_PLACEHOLDER),
            expected_cpus=4,
            timeout_s=timeout_s,
        )

      # (a) Passing boot.
      rec_pass = deflake.run_qemu_boot(
          make_cfg("pass", 10.0),
          tmp_path / "run_pass",
          iso_snap,
          tmp_path,
          self.names,
          index=1,
          build_id="b1",
          grace_s=0.2,
      )
      self.assertEqual(rec_pass.status, "PASS")

      # (b) Timeout boot.
      rec_to = deflake.run_qemu_boot(
          make_cfg("timeout", 0.5),
          tmp_path / "run_to",
          iso_snap,
          tmp_path,
          self.names,
          index=1,
          build_id="b1",
          grace_s=0.1,
      )
      self.assertEqual(rec_to.status, "FAIL")
      self.assertIn("timeout", rec_to.reasons)

      # (c) Early exit 1.
      rec_ex = deflake.run_qemu_boot(
          make_cfg("exit1", 5.0),
          tmp_path / "run_ex",
          iso_snap,
          tmp_path,
          self.names,
          index=1,
          build_id="b1",
          grace_s=0.1,
      )
      self.assertEqual(rec_ex.status, "FAIL")
      self.assertIn("qemu_exited:1", rec_ex.reasons)

      # (d) Drain captures later test results after early FAIL.
      rec_dr = deflake.run_qemu_boot(
          make_cfg("drain", 5.0),
          tmp_path / "run_dr",
          iso_snap,
          tmp_path,
          self.names,
          index=1,
          build_id="b1",
          grace_s=0.1,
      )
      self.assertEqual(rec_dr.status, "FAIL")
      self.assertEqual(rec_dr.outcomes["pmm_memory_map_init"], "FAIL")
      self.assertEqual(rec_dr.outcomes["user_mode_mixed_workload"], "PASS")

      # (e) Late fatal within grace window fails the run.
      rec_lf = deflake.run_qemu_boot(
          make_cfg("late_fatal", 5.0),
          tmp_path / "run_lf",
          iso_snap,
          tmp_path,
          self.names,
          index=1,
          build_id="b1",
          grace_s=0.4,
      )
      self.assertEqual(rec_lf.status, "FAIL")
      self.assertIn("panic", rec_lf.reasons)

  def test_test_boot_sh_cli_hooks_with_fake_qemu(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      script_copy = tmp_path / "test_boot.sh"
      shutil.copy2(REPO_ROOT / "test_boot.sh", script_copy)
      script_copy.chmod(0o755)
      (tmp_path / "kernel.iso").write_bytes(b"fake-iso")
      (tmp_path / "kernel-mb1.iso").write_bytes(b"fake-mb1-iso")
      (tmp_path / "OVMF.fd").write_bytes(b"fake-ovmf")

      fakebin = tmp_path / "fakebin"
      fakebin.mkdir()
      fake_qemu = fakebin / "qemu-system-x86_64"
      # The serial log is the -serial file: or, in the USB passes, the
      # logfile= of the -chardev that -serial chardev:<id> names.
      fake_qemu.write_text(
          textwrap.dedent("""\
          #!/usr/bin/env python3
          import sys, time
          argv = sys.argv[1:]
          if "--version" in argv:
              print("QEMU emulator version 11.1.0")
              sys.exit(0)
          serial_arg = argv[argv.index("-serial") + 1]
          serial_path = serial_arg[len("file:"):]
          if serial_arg.startswith("chardev:"):
              for i, arg in enumerate(argv[:-1]):
                  if arg == "-chardev":
                      opts = dict(o.split("=", 1) for o in argv[i + 1].split(",")[1:])
                      if opts["id"] == serial_arg[len("chardev:"):]:
                          serial_path = opts["logfile"]
          with open(serial_path, "wb") as f:
              f.write(b"Hello, x86-64 Kernel World!\\r\\n")
              f.write(b"[TEST] rbtree_edge_cases: FAIL\\r\\n")
              f.flush()
              time.sleep(0.3)
              f.write(b"[TEST] multiboot_mmap_edge_cases: PASS\\r\\n")
              f.write(b"[TEST] BOOT SELF-TESTS FAILED\\r\\n")
              f.flush()
          time.sleep(30)
          """),
          encoding="utf-8",
      )
      fake_qemu.chmod(0o755)

      env = os.environ.copy()
      env["PATH"] = f"{fakebin}:{env.get('PATH', '')}"
      for var in ("USE_DOCKER", "KVM", "BOOT_TEST_USB", "QEMU_USB_FLAGS",
                  "USB_SESSION_TIMEOUT_SECS"):
        env.pop(var, None)

      # 1. Invalid --passes values and USB settings fail closed before running
      # QEMU.
      bad_runs = [
          ([bad_arg], {})
          for bad_arg in ("--passes=bogus", "--passes=,", "--passes=*",
                          "--unknown-opt")
      ] + [
          (["--passes=bios"], {"BOOT_TEST_USB": "yes"}),
          (["--passes=bios"], {"USB_SESSION_TIMEOUT_SECS": "0"}),
          (["--passes=bios"], {"USB_SESSION_TIMEOUT_SECS": "1e3"}),
          (["--passes=bios"], {"QEMU_USB_FLAGS": "-device qemu-xhci,id=xhci0"}),
          (["--passes=bios"],
           {"QEMU_USB_FLAGS": "-device usb-kbd,bus=xhci0.0,port=1,id=kbd0"}),
      ]
      for args, extra_env in bad_runs:
        proc = subprocess.run(
            [str(script_copy), *args, "kernel.iso", "kernel-mb1.iso"],
            cwd=tmp_path,
            env={**env, **extra_env},
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 1, (args, extra_env))
        self.assertIn("FAIL:", proc.stderr + proc.stdout)
        self.assertNotIn("Running ", proc.stdout, (args, extra_env))

      # 2. --passes=max,bios --artifact-dir=... --drain-on-fail runs only bios and max in canonical order.
      art_dir = tmp_path / "artifacts"
      proc_ok = subprocess.run(
          [
              str(script_copy),
              "--passes=max,bios",
              f"--artifact-dir={art_dir}",
              "--drain-on-fail",
              "kernel.iso",
              "kernel-mb1.iso",
          ],
          cwd=tmp_path,
          env=env,
          capture_output=True,
          text=True,
          check=False,
      )
      self.assertEqual(proc_ok.returncode, 1)
      combined_out = proc_ok.stdout + proc_ok.stderr
      self.assertIn(
          "FAILURE: boot verification failed (bios: FAIL max: FAIL).",
          combined_out,
      )
      self.assertNotIn("uefi:", combined_out)
      argvs = {}
      for p in ("bios", "max"):
        self.assertEqual(
            (art_dir / p / "result").read_text(encoding="utf-8").strip(),
            "FAIL",
        )
        argv_items = (
            (art_dir / p / "qemu_argv")
            .read_bytes()
            .decode("utf-8")
            .rstrip("\0")
            .split("\0")
        )
        argvs[p] = argv_items
        self.assertEqual(argv_items[0], "qemu-system-x86_64")
        self.assertIn("-qmp", argv_items)
        if p == "max":
          self.assertIn("max", argv_items)
        serial_txt = (art_dir / p / "serial.log").read_text(encoding="utf-8")
        self.assertIn("[TEST] multiboot_mmap_edge_cases: PASS", serial_txt)
        self.assertIn("[TEST] BOOT SELF-TESTS FAILED", serial_txt)
        # A failure line in the serial log ends the USB session at once and is
        # reported like any failed boot (the first FAIL: line of the output).
        output_txt = (art_dir / p / "output.log").read_text(encoding="utf-8")
        self.assertIn(
            "did not complete within its timeout",
            deflake.first_harness_failure(output_txt) or "",
        )
      # The bios pass (USB scenarios by default) has the USB devices of
      # `make run`, the fw_cfg opt-in and a serial console socket that logs to
      # the usual file; the max pass keeps its file serial and no USB device.
      # Their files are in the run's own directory under build/, which
      # test_boot.sh deletes at its end.
      run_dir = (art_dir / "bios" / "qemu_run_dir").read_text(
          encoding="utf-8").strip()
      self.assertRegex(run_dir, r"^build/test_boot\.[A-Za-z0-9]+$")
      self.assertEqual(
          (art_dir / "max" / "qemu_run_dir").read_text(encoding="utf-8").strip(),
          run_dir,
      )
      bios_argv = argvs["bios"]
      chardev = bios_argv[bios_argv.index("-chardev") + 1]
      self.assertEqual(
          chardev,
          f"socket,id=ser0,path={run_dir}/qemu_bios_serial.sock,server=on,"
          f"wait=off,logfile={run_dir}/serial_bios_output.log",
      )
      self.assertEqual(
          bios_argv[bios_argv.index("-serial") + 1], "chardev:ser0"
      )
      usb_args = [
          "-device", "qemu-xhci,id=xhci0",
          "-device", "usb-kbd,bus=xhci0.0,port=1,id=kbd0",
          "-fw_cfg", "name=opt/protos/boot_session,string=interactive",
      ]
      start = bios_argv.index("-device")
      self.assertEqual(bios_argv[start:start + len(usb_args)], usb_args)
      self.assertEqual(
          argvs["max"][argvs["max"].index("-serial") + 1],
          f"file:{run_dir}/serial_max_output.log",
      )
      for absent in ("-chardev", "-device", "-fw_cfg"):
        self.assertNotIn(absent, argvs["max"])
      for p in ("bios", "max"):
        self.assertEqual(
            (art_dir / p / "qemu_stderr.log").read_text(encoding="utf-8"), ""
        )
      self.assertEqual(sorted(os.listdir(tmp_path / "build")), [])

      # 3. BOOT_TEST_USB=0 runs bios and uefi exactly as without the USB
      # scenarios (the USB settings are then ignored).
      art_off = tmp_path / "artifacts_off"
      proc_off = subprocess.run(
          [
              str(script_copy),
              "--passes=bios,uefi",
              f"--artifact-dir={art_off}",
              "kernel.iso",
              "kernel-mb1.iso",
          ],
          cwd=tmp_path,
          env={**env, "BOOT_TEST_USB": "0", "OVMF": str(tmp_path / "OVMF.fd"),
               "USB_SESSION_TIMEOUT_SECS": "bogus", "QEMU_USB_FLAGS": "bogus"},
          capture_output=True,
          text=True,
          check=False,
      )
      self.assertIn(
          "FAILURE: boot verification failed (bios: FAIL uefi: FAIL).",
          proc_off.stdout + proc_off.stderr,
      )
      common = ["qemu-system-x86_64", "-smp", "4", "-display", "none"]
      for p, rest in (
          ("bios", ["-cpu", "qemu64"]),
          ("uefi", ["-bios", str(tmp_path / "OVMF.fd"), "-cpu",
                    "qemu64,+x2apic", "-m", "4G"]),
      ):
        run_dir = (art_off / p / "qemu_run_dir").read_text(
            encoding="utf-8").strip()
        self.assertEqual(
            (art_off / p / "qemu_argv").read_bytes().decode().split("\0"),
            common + [
                "-serial", f"file:{run_dir}/serial_{p}_output.log",
                "-monitor", f"unix:{run_dir}/qemu_{p}_monitor.sock,server,nowait",
                "-qmp", f"unix:{run_dir}/qemu_{p}_qmp.sock,server,nowait",
                "-no-reboot",
            ] + rest + ["-cdrom", str(tmp_path / "kernel.iso"), ""],
        )
        output_txt = (art_off / p / "output.log").read_text(encoding="utf-8")
        self.assertNotIn("USB", output_txt)

  def test_concurrent_test_boot_runs_keep_their_files_apart(self) -> None:
    # Each fake QEMU logs the token of its run a while after it starts, so
    # that the second run starts while the first one waits for it.
    fake_qemu = """\
        #!/usr/bin/env python3
        import os, sys, time
        argv = sys.argv[1:]
        serial_path = argv[argv.index("-serial") + 1][len("file:"):]
        with open(serial_path, "ab") as f:
            f.write(b"Hello, x86-64 Kernel World!\\r\\n")
            f.flush()
            time.sleep(1.5)
            f.write(b"[TEST] run_%s: FAIL\\r\\n" % os.environ["FAKE_TOKEN"].encode())
            f.flush()
        time.sleep(30)
        """
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      env = make_test_boot_repo(tmp_path, fake_qemu)
      procs = {}
      for token in ("first", "second"):
        procs[token] = subprocess.Popen(
            [str(tmp_path / "test_boot.sh"), "--passes=max",
             f"--artifact-dir={tmp_path / ('art_' + token)}", "kernel.iso",
             "kernel-mb1.iso"],
            cwd=tmp_path,
            env={**env, "FAKE_TOKEN": token},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(0.5)
      for token, proc in procs.items():
        out, _ = proc.communicate(timeout=120)
        other = "second" if token == "first" else "first"
        self.assertEqual(proc.returncode, 1, out)
        serial = (tmp_path / f"art_{token}" / "max" / "serial.log").read_text(
            encoding="utf-8")
        self.assertEqual(serial.count("Hello, x86-64 Kernel World!"), 1, out)
        self.assertIn(f"[TEST] run_{token}: FAIL", serial)
        self.assertNotIn(f"run_{other}", serial + out)
      self.assertEqual(sorted(os.listdir(tmp_path / "build")), [])

  def test_test_boot_sh_signal_cleanup_kills_parallel_qemus(self) -> None:
    fake_qemu = """\
        #!/usr/bin/env python3
        import os, pathlib, sys, time
        pid_dir = pathlib.Path(os.environ["FAKE_PID_DIR"])
        tmp = pid_dir / f"{os.getpid()}.tmp"
        dst = pid_dir / f"{os.getpid()}.pid"
        tmp.write_text(str(os.getpid()))
        tmp.replace(dst)
        time.sleep(120)
        """
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      env = make_test_boot_repo(tmp_path, fake_qemu)
      pid_dir = tmp_path / "pids"
      pid_dir.mkdir()
      env["FAKE_PID_DIR"] = str(pid_dir)
      env["BOOT_TEST_JOBS"] = "2"
      env["BOOT_TEST_USB"] = "0"
      proc = subprocess.Popen(
          [str(tmp_path / "test_boot.sh"), "--passes=bios,max", "kernel.iso",
           "kernel-mb1.iso"],
          cwd=tmp_path,
          env=env,
          stdout=subprocess.PIPE,
          stderr=subprocess.STDOUT,
          text=True,
      )
      try:
        deadline = time.monotonic() + 20.0
        while len(list(pid_dir.glob("*.pid"))) < 2:
          self.assertLess(time.monotonic(), deadline)
          time.sleep(0.05)
        qemu_pids = [int(p.read_text()) for p in pid_dir.glob("*.pid")]
        proc.send_signal(signal.SIGTERM)
        proc.communicate(timeout=30)
      finally:
        if proc.poll() is None:
          proc.kill()
          proc.wait()
      self.assertEqual(proc.returncode, 143)
      for qpid in qemu_pids:
        self.assertTrue(process_gone(qpid), qpid)
      self.assertEqual(sorted(os.listdir(tmp_path / "build")), [])

  def test_test_boot_sh_reports_qemu_errors(self) -> None:
    fake_qemu = """\
        #!/usr/bin/env python3
        import os, sys
        sys.stderr.write(os.environ["FAKE_QEMU_ERROR"] + "\\n")
        sys.exit(1)
        """
    error = ("qemu-system-x86_64: -device nec-usb-xhci: 'nec-usb-xhci' is not "
             "a valid device model name")
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      env = make_test_boot_repo(tmp_path, fake_qemu)
      env["FAKE_QEMU_ERROR"] = error

      def run(passes: str, art: str, **extra_env: str) -> str:
        proc = subprocess.run(
            [str(tmp_path / "test_boot.sh"), f"--passes={passes}",
             f"--artifact-dir={tmp_path / art}", "kernel.iso",
             "kernel-mb1.iso"],
            cwd=tmp_path,
            env={**env, **extra_env},
            capture_output=True,
            text=True,
            check=False,
            timeout=120,
        )
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        return proc.stdout + proc.stderr

      # A QEMU that exits at once: its error is reported (and saved), and the
      # failure is that QEMU exited, not a timeout.
      out = run("max", "art_max")
      output = (tmp_path / "art_max" / "max" / "output.log").read_text(
          encoding="utf-8")
      first = deflake.first_harness_failure(output) or ""
      self.assertIn("QEMU exited unexpectedly before the boot completed", first)
      self.assertIn(error, out)
      self.assertEqual(
          (tmp_path / "art_max" / "max" / "qemu_stderr.log").read_text(
              encoding="utf-8"),
          error + "\n",
      )
      verdict = deflake.judge_test_boot(
          "FAIL", 1, "FAIL", output_log=output,
          report=deflake.parse_serial(b""))
      self.assertEqual(verdict.reasons, ("qemu_exited",))

      # Split OVMF images: QEMU boots the VARS template itself with
      # snapshot=on, which keeps it unchanged (no copy that rerun.sh misses).
      code, vars_ = tmp_path / "OVMF_CODE.fd", tmp_path / "OVMF_VARS.fd"
      run("uefi", "art_uefi", OVMF=str(code), OVMF_VARS=str(vars_),
          BOOT_TEST_USB="0")
      argv = (tmp_path / "art_uefi" / "uefi" / "qemu_argv").read_bytes()
      argv_items = argv.decode().split("\0")
      self.assertIn(f"if=pflash,format=raw,unit=0,readonly=on,file={code}",
                    argv_items)
      self.assertIn(f"if=pflash,format=raw,unit=1,snapshot=on,file={vars_}",
                    argv_items)
      # A VARS template that QEMU can't read fails the pass before QEMU runs.
      out = run("uefi", "art_novars", OVMF=str(code),
                OVMF_VARS=str(tmp_path / "missing_vars.fd"), BOOT_TEST_USB="0")
      self.assertIn("FAIL: The OVMF_VARS template", out)
      self.assertFalse((tmp_path / "art_novars" / "uefi" / "qemu_argv").exists())
      self.assertEqual(sorted(os.listdir(tmp_path / "build")), [])

  def test_test_boot_sh_docker_fallback_passes_arguments_on(self) -> None:
    if os.path.exists("/.dockerenv") or os.path.exists("/run/.containerenv"):
      self.skipTest("test_boot.sh has no Docker fallback in a container")
    if shutil.which("make") is None:
      self.skipTest("make not found")
    with tempfile.TemporaryDirectory() as tmp, \
        tempfile.TemporaryDirectory() as outside:
      tmp_path = Path(tmp)
      env = make_test_boot_repo(tmp_path, "#!/bin/sh\nexit 1\n")
      env["USE_DOCKER"] = "1"
      # Prints the words that make passes to the shell, like the recipe of
      # the Makefile's run-boot-test.
      (tmp_path / "Makefile").write_text(
          "run-boot-test:\n"
          "\t@printf '%s\\n' USE_DOCKER=$(USE_DOCKER) $(BOOT_TEST_OPTS) "
          "ISOS: $(BOOT_TEST_ISOS) >make_args.txt\n",
          encoding="utf-8",
      )
      make_args = tmp_path / "make_args.txt"

      def run(*args: str) -> subprocess.CompletedProcess[str]:
        if make_args.exists():
          make_args.unlink()
        return subprocess.run(
            [str(tmp_path / "test_boot.sh"), *args],
            cwd=tmp_path,
            env=env,
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )

      art = tmp_path / "art $dir 'q'"
      proc = run("--passes", "bios uefi", f"--artifact-dir={art}",
                 "--drain-on-fail", str(tmp_path / "kernel.iso"),
                 "missing-mb1.iso")
      self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
      self.assertEqual(
          make_args.read_text(encoding="utf-8").splitlines(),
          ["USE_DOCKER=1", "--passes=bios uefi", "--artifact-dir=art $dir 'q'",
           "--drain-on-fail", "ISOS:", "kernel.iso", "missing-mb1.iso"],
      )
      self.assertTrue(art.is_dir())
      # Without options and ISO files, make boots (and builds) its own ISOs.
      proc = run()
      self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
      self.assertEqual(make_args.read_text(encoding="utf-8").splitlines(),
                       ["USE_DOCKER=1", "ISOS:"])
      # The container sees only the repository.
      outside_iso = Path(outside) / "kernel.iso"
      outside_iso.write_bytes(b"iso")
      for args in ((str(outside_iso),),
                   (f"--artifact-dir={outside}/art", "kernel.iso")):
        proc = run(*args)
        self.assertEqual(proc.returncode, 1, args)
        self.assertIn("sees only the repository", proc.stderr)
        self.assertFalse(make_args.exists())

  def test_run_test_boot_pass_stops_the_harness_on_interrupt(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      pids = tmp_path / "pids"
      fake_tb = tmp_path / "fake_test_boot.sh"
      # Like test_boot.sh: a process group with a VM process in it.
      fake_tb.write_text(
          textwrap.dedent(f"""\
          #!/usr/bin/env python3
          import os, subprocess, time
          vm = subprocess.Popen(["sleep", "60"])
          with open({str(pids)!r} + ".tmp", "w") as f:
              f.write("%d %d" % (os.getpid(), vm.pid))
          os.rename({str(pids)!r} + ".tmp", {str(pids)!r})
          time.sleep(60)
          """),
          encoding="utf-8",
      )
      fake_tb.chmod(0o755)
      iso = tmp_path / "kernel.iso"
      iso.write_bytes(b"fake-iso")
      cfg = deflake._make_builtin_config("host:max")

      def interrupt_once_started() -> None:
        if wait_for_file(pids):
          os.kill(os.getpid(), signal.SIGINT)

      old_handler = signal.signal(signal.SIGINT, signal.default_int_handler)
      try:
        threading.Thread(target=interrupt_once_started, daemon=True).start()
        with self.assertRaises(KeyboardInterrupt):
          deflake.run_test_boot_pass(
              cfg,
              tmp_path / "run",
              iso,
              None,
              tmp_path,
              self.names,
              index=1,
              build_id="b1",
              session_id="s1",
              test_boot_script=fake_tb,
          )
      finally:
        signal.signal(signal.SIGINT, old_handler)
      for pid in map(int, pids.read_text(encoding="utf-8").split()):
        self.assertTrue(process_gone(pid), pid)

  def test_main_stops_the_run_on_sigterm_and_sighup(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      iso = tmp_path / "kernel.iso"
      iso.write_bytes(b"fake-iso")
      fake_qemu = tmp_path / "qemu-system-x86_64"
      fake_qemu.write_text(
          textwrap.dedent("""\
          #!/usr/bin/env python3
          import os, sys, time
          pid_file = os.environ["FAKE_QEMU_PID_FILE"]
          with open(pid_file + ".tmp", "w") as f:
              f.write(str(os.getpid()))
          os.rename(pid_file + ".tmp", pid_file)
          sys.stdout.write("Hello, x86-64 Kernel World!\\r\\n")
          sys.stdout.flush()
          time.sleep(120)
          """),
          encoding="utf-8",
      )
      fake_qemu.chmod(0o755)
      lock = tmp_path / "host.lock"
      # deflake.py as a command, with the default handlers of the signals.
      child_code = (
          "import signal, sys\n"
          "signal.signal(signal.SIGTERM, signal.SIG_DFL)\n"
          "signal.signal(signal.SIGHUP, signal.SIG_DFL)\n"
          "import deflake\n"
          "sys.exit(deflake.main(sys.argv[1:]))\n"
      )
      for sig in (signal.SIGTERM, signal.SIGHUP):
        out_dir = tmp_path / f"out_{sig.name}"
        pid_file = tmp_path / f"qemu_{sig.name}.pid"
        child = subprocess.Popen(
            [sys.executable, "-B", "-c", child_code, "--qemu-cmd",
             f"{shlex.quote(str(fake_qemu))} -smp 4 -serial mon:stdio"
             f" -cdrom {shlex.quote(str(iso))}",
             "--runs", "1", "--lock", str(lock), "--out", str(out_dir),
             "--quiet"],
            cwd=REPO_ROOT,
            env={**os.environ, "FAKE_QEMU_PID_FILE": str(pid_file)},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
          self.assertTrue(wait_for_file(pid_file), sig.name)
          child.send_signal(sig)
          out, _ = child.communicate(timeout=60)
        finally:
          if child.poll() is None:
            child.kill()
            child.wait()
        # The VM is gone and the host lock free before deflake exits, which
        # writes the session's summary.
        self.assertEqual(child.returncode, 128 + sig, out)
        self.assertTrue(process_gone(int(pid_file.read_text())), sig.name)
        session = json.loads(
            (out_dir / "session.json").read_text(encoding="utf-8"))
        self.assertEqual(session["stop_reason"], "interrupted")
        fd = os.open(lock, os.O_RDWR)
        try:
          fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
          os.close(fd)

  def test_docker_session_runs_and_records_the_image(self) -> None:
    image_id = "sha256:" + "ab" * 32
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      repo = tmp_path / "repo"
      (repo / "selftest").mkdir(parents=True)
      shutil.copy2(REPO_ROOT / "selftest" / "boot_selftest.cpp",
                   repo / "selftest" / "boot_selftest.cpp")
      iso = tmp_path / "kernel.iso"
      iso.write_bytes(b"fake-iso")
      mb1 = tmp_path / "kernel-mb1.iso"
      mb1.write_bytes(b"fake-mb1-iso")
      docker_log = tmp_path / "docker_run.json"
      fakebin = tmp_path / "fakebin"
      fakebin.mkdir()
      # A docker whose container runs test_boot.sh's failing pass.
      (fakebin / "docker").write_text(
          textwrap.dedent(f"""\
          #!/usr/bin/env python3
          import json, pathlib, sys
          args = sys.argv[1:]
          if args[:2] == ["image", "inspect"]:
              print({image_id!r})
          elif args[:1] == ["info"]:
              print("{{}}")
          elif args[:1] == ["run"]:
              pathlib.Path({str(docker_log)!r}).write_text(json.dumps(args))
              opts = dict(a.split("=", 1) for a in args if a.startswith("--")
                          and "=" in a)
              d = pathlib.Path(opts["--artifact-dir"]) / opts["--passes"]
              d.mkdir(parents=True)
              (d / "result").write_text("FAIL\\n")
              (d / "serial.log").write_text(
                  "Hello, x86-64 Kernel World!\\n[TEST] rbtree_edge_cases: FAIL\\n")
              (d / "output.log").write_text("FAIL: boot printed a failure.\\n")
              (d / "qemu_argv").write_bytes(
                  b"qemu-system-x86_64\\0-cdrom\\0/workspace/k.iso\\0")
              (d / "qemu_cwd").write_text("/workspace\\n")
              (d / "qemu_run_dir").write_text("build/test_boot.abc123\\n")
              print("FAILURE: boot verification failed (max: FAIL).")
              sys.exit(1)
          else:
              sys.exit(2)
          """),
          encoding="utf-8",
      )
      (fakebin / "docker").chmod(0o755)
      out_dir = tmp_path / "out"
      args = deflake.build_arg_parser().parse_args([
          "--config", "docker:max", "--runs", "1", "--no-lock", "--out",
          str(out_dir), "--iso", str(iso), "--mb1-iso", str(mb1), "--quiet",
      ])
      env = {"PATH": f"{fakebin}:{os.environ.get('PATH', '')}",
             "HOST_UID": "1234", "HOST_GID": "5678"}
      with mock.patch.dict(os.environ, env):
        self.assertEqual(deflake.run_session(args, repo=repo), 1)
      docker_args = json.loads(docker_log.read_text(encoding="utf-8"))
      self.assertIn(image_id, docker_args)
      self.assertEqual(docker_args[docker_args.index("--user") + 1],
                       "1234:5678")
      run_dir = out_dir / "runs" / "docker-max" / "0001"
      command = json.loads(
          (run_dir / "command.json").read_text(encoding="utf-8"))
      self.assertEqual(command["docker_image_id"], image_id)
      rerun = (run_dir / "rerun.sh").read_text(encoding="utf-8")
      self.assertIn(f"mkdir -p {repo / 'build' / 'test_boot.abc123'}\n", rerun)
      exec_words = shlex.split(rerun.splitlines()[-1])
      self.assertEqual(exec_words[exec_words.index("--user") + 1], "1234:5678")
      self.assertEqual(exec_words[-4:], [image_id, "qemu-system-x86_64",
                                         "-cdrom", "/workspace/k.iso"])

  def test_docker_run_identity(self) -> None:
    host = f"{os.getuid()}:{os.getgid()}"

    def identity(env: dict[str, str], rootless: bool,
                 selinux: bool) -> tuple[str, str]:
      return deflake.docker_run_identity(
          env, rootless=lambda _: rootless, selinux=lambda: selinux)

    self.assertEqual(identity({}, False, False), (host, ""))
    # Rootless Docker and Podman map container root to the host user.
    self.assertEqual(identity({}, True, True), ("0:0", ":z"))
    self.assertEqual(
        identity({"HOST_UID": "7", "HOST_GID": "8", "DOCKER_VOLUME_OPTS": ""},
                 True, True),
        ("7:8", ""),
    )
    self.assertEqual(identity({"HOST_UID": "7"}, True, False),
                     (f"7:{os.getgid()}", ""))
    argv = deflake.build_docker_test_boot_argv(
        Path("/repo"), "max", "art", "k.iso", "mb1.iso", user="0:0",
        volume_opts=":z")
    self.assertEqual(argv[argv.index("--user") + 1], "0:0")
    self.assertIn("/repo:/workspace:z", argv)
    with tempfile.TemporaryDirectory() as tmp:
      for name, info, rootless in (
          ("docker", '{"SecurityOptions":["name=seccomp","name=rootless"]}', True),
          ("podman", '{"host":{"security":{"rootless":true}}}', True),
          ("rootful", '{"SecurityOptions":["name=seccomp"]}', False),
      ):
        fake = Path(tmp) / name
        fake.write_text(f"#!/bin/sh\necho '{info}'\n", encoding="utf-8")
        fake.chmod(0o755)
        self.assertEqual(deflake.docker_is_rootless(str(fake)), rootless, name)
      self.assertFalse(deflake.docker_is_rootless(str(Path(tmp) / "missing")))

  def test_rerun_and_reproduce_scripts_rerun_the_archived_run(self) -> None:
    repo = Path("/repo")
    tool = REPO_ROOT / "deflake.py"
    # reproduce.sh of a --qemu-cmd run boots the archived ISO, not the file
    # at the -cdrom path of the command.
    cmd = ("qemu-system-x86_64 -enable-kvm -cpu host -smp 4 -serial mon:stdio"
           " -cdrom build/kernel.iso")
    cfg = deflake.expand_configs(qemu_cmd_list=[cmd])[0]
    rec = make_run_record(config=cfg.name, harness_argv=())
    script = deflake.render_reproduce_script(rec, cfg, repo, tool)
    words = shlex.split(script.splitlines()[-1])
    qemu_cmd = words[words.index("--qemu-cmd") + 1]
    self.assertNotIn("build/kernel.iso", script)
    reproduced = deflake.expand_configs(qemu_cmd_list=[qemu_cmd])[0]
    self.assertEqual(reproduced.custom_iso, Path(rec.iso_snapshot))
    self.assertEqual(reproduced.qemu_argv, cfg.qemu_argv)

    # rerun.sh of a test_boot.sh USB pass recreates the run's directory and
    # leaves out the interactive boot session, which would wait forever
    # without the harness.
    qemu_argv = ("qemu-system-x86_64", "-serial",
                 "file:build/test_boot.abc123/serial_bios_output.log",
                 *deflake.INTERACTIVE_SESSION_ARGS, "-cdrom", rec.iso_snapshot)
    rec = make_run_record(qemu_argv=qemu_argv, qemu_run_dir="build/test_boot.abc123",
                          harness_argv=("./test_boot.sh", "--passes=bios"))
    script = deflake.render_rerun_script(
        rec, deflake._make_builtin_config("host:bios"), repo)
    lines = script.splitlines()
    self.assertIn("mkdir -p build/test_boot.abc123", lines)
    self.assertEqual(shlex.split(lines[-1])[1:],
                     [a for a in qemu_argv if a not in
                      deflake.INTERACTIVE_SESSION_ARGS])
    self.assertIn(f"# {shlex.join(qemu_argv)}", lines)
    # In Docker: the harness's user and bind mount options, and the image.
    rec = dataclasses.replace(
        rec,
        config="docker:bios",
        harness_argv=deflake.build_docker_test_boot_argv(
            repo, "bios", "art", "k.iso", "mb1.iso", user="0:0",
            volume_opts=":z"),
    )
    script = deflake.render_rerun_script(
        rec, deflake._make_builtin_config("docker:bios"), repo,
        docker_image_id="sha256:1234")
    lines = script.splitlines()
    self.assertIn("mkdir -p /repo/build/test_boot.abc123", lines)
    words = shlex.split(lines[-1])
    self.assertEqual(words[words.index("--user") + 1], "0:0")
    self.assertIn("/repo:/workspace:z", words)
    self.assertIn("/store:/store:ro,z", words)
    self.assertEqual(words[words.index("sha256:1234") + 1:],
                     [a for a in qemu_argv if a not in
                      deflake.INTERACTIVE_SESSION_ARGS])

  def test_sources_parse_as_python_3_8(self) -> None:
    for name in ("deflake.py", "deflake_test.py"):
      source = (REPO_ROOT / name).read_text(encoding="utf-8")
      ast.parse(source, feature_version=(3, 8))
      # Newer parsers accept a parenthesized with statement (Python 3.9+)
      # also for feature_version=(3, 8).
      tokens = list(tokenize.generate_tokens(io.StringIO(source).readline))
      for i, tok in enumerate(tokens[:-1]):
        if not (tok.type == tokenize.NAME and tok.string == "with" and
                tokens[i + 1].type == tokenize.OP and
                tokens[i + 1].string == "("):
          continue
        depth = 0
        for t in tokens[i + 1:]:
          if t.type == tokenize.OP and t.string in ("(", "[", "{"):
            depth += 1
          elif t.type == tokenize.OP and t.string in (")", "]", "}"):
            depth -= 1
            if depth == 0:
              break
          elif depth == 1 and t.type == tokenize.NAME and t.string == "as":
            self.fail(f"{name}:{tok.start[0]}: parenthesized with statement")

  def test_end_to_end_session_writes_artifacts_and_rerun_script(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      iso_path = tmp_path / "kernel.iso"
      iso_path.write_bytes(b"fake-iso-image-GRUB 2.14-1-end")
      fake_qemu = tmp_path / "qemu-system-x86_64"
      fail_log = make_serial_log(
          names=self.names,
          cpus=(4, 4),
          fail_test="rbtree_edge_cases",
          injected_seed="0x0000000000000028",
      )
      fake_qemu.write_text(
          textwrap.dedent(f"""\
          #!/usr/bin/env python3
          import sys, time
          if "--version" in sys.argv:
              print("QEMU emulator version 11.1.0")
              sys.exit(0)
          sys.stdout.buffer.write({fail_log!r})
          sys.stdout.buffer.flush()
          time.sleep(30)
          """),
          encoding="utf-8",
      )
      fake_qemu.chmod(0o755)

      out_dir = tmp_path / "session_out"
      lock_file = tmp_path / "deflake.lock"
      cmd_str = (
          f"{shlex.quote(str(fake_qemu))} -smp 4 -m 256M -serial mon:stdio"
          f" -cdrom {shlex.quote(str(iso_path))}"
      )
      rc = deflake.main([
          "--qemu-cmd",
          cmd_str,
          "--runs",
          "1",
          "--lock",
          str(lock_file),
          "--out",
          str(out_dir),
          "--quiet",
      ])
      # Session returns 1 when any run fails.
      self.assertEqual(rc, 1)
      summary = json.loads(
          (out_dir / "summary.json").read_text(encoding="utf-8")
      )
      self.assertEqual(summary["schema"], "protos-deflake/1")
      self.assertEqual(len(summary["runs"]), 1)
      run0 = summary["runs"][0]
      self.assertEqual(run0["status"], "FAIL")
      self.assertEqual(run0["failed_tests"], ["rbtree_edge_cases"])
      self.assertEqual(run0["seeds"], ["0x0000000000000028"])

      run_dir = out_dir / run0["artifact_dir"]
      for fname in (
          "serial.log",
          "command.json",
          "rerun.sh",
          "reproduce.sh",
          "verdict.json",
          "seeds.txt",
          "build.json",
      ):
        self.assertTrue((run_dir / fname).is_file(), f"missing {fname}")
      rerun_mode = (run_dir / "rerun.sh").stat().st_mode
      self.assertTrue(bool(rerun_mode & stat.S_IXUSR))

  def test_run_test_boot_pass_and_docker_argv(self) -> None:
    docker_argv = deflake.build_docker_test_boot_argv(
        REPO_ROOT,
        "max",
        "build/deflake/art",
        "build/deflake/isos/k.iso",
        "build/deflake/isos/mb1.iso",
        container_name="deflake-test-1",
    )
    self.assertIn("--passes=max", docker_argv)
    self.assertIn("--artifact-dir=build/deflake/art", docker_argv)
    self.assertIn("--drain-on-fail", docker_argv)
    for forbidden_env in (
        "BOOT_TEST_PASSES",
        "BOOT_TEST_ARTIFACT_DIR",
        "BOOT_TEST_DRAIN_ON_FAIL",
    ):
      self.assertNotIn(forbidden_env, docker_argv)

    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      iso_snap = tmp_path / "kernel.iso"
      mb1_snap = tmp_path / "kernel-mb1.iso"
      iso_snap.write_bytes(b"fake-iso")
      mb1_snap.write_bytes(b"fake-mb1")
      fake_tb = tmp_path / "fake_test_boot.sh"
      pass_log = make_serial_log(names=self.names, cpus=(4, 4))
      fake_tb.write_text(
          textwrap.dedent(f"""\
          #!/usr/bin/env python3
          import os, pathlib, sys
          assert os.environ.get("USE_DOCKER") == "0"
          args = sys.argv[1:]
          passes = [a.split("=", 1)[1] for a in args if a.startswith("--passes=")][0]
          art = pathlib.Path([a.split("=", 1)[1] for a in args if a.startswith("--artifact-dir=")][0])
          assert "--drain-on-fail" in args
          pdir = art / passes
          pdir.mkdir(parents=True, exist_ok=True)
          (pdir / "result").write_text("PASS\\n")
          (pdir / "serial.log").write_bytes({pass_log!r})
          (pdir / "output.log").write_text("Running pass...\\n")
          (pdir / "qemu_argv").write_bytes(b"qemu-system-x86_64\\0-cpu\\0max\\0")
          (pdir / "qemu_cwd").write_text(str(pathlib.Path.cwd()) + "\\n")
          (pdir / "qemu_run_dir").write_text("build/test_boot.Ab12Cd\\n")
          print(f"SUCCESS: boot verification passed ({{passes}}: PASS): greeting")
          sys.exit(0)
          """),
          encoding="utf-8",
      )
      fake_tb.chmod(0o755)
      cfg = deflake.Config(
          name="host:max",
          engine="test_boot",
          mode="host",
          pass_name="max",
          timeout_s=60.0,
      )
      rec = deflake.run_test_boot_pass(
          cfg,
          tmp_path / "run_tb",
          iso_snap,
          mb1_snap,
          tmp_path,
          self.names,
          index=1,
          build_id="b1",
          session_id="sess1",
          test_boot_script=fake_tb,
      )
      self.assertEqual(rec.status, "PASS")
      self.assertEqual(rec.qemu_argv, ("qemu-system-x86_64", "-cpu", "max"))
      self.assertEqual(rec.qemu_run_dir, "build/test_boot.Ab12Cd")
      self.assertEqual(rec.outcomes["@harness"], "PASS")

      docker_cfg = deflake.Config(
          name="docker:max",
          engine="test_boot",
          mode="docker",
          pass_name="max",
          timeout_s=60.0,
      )
      docker_rerun = deflake.render_rerun_script(rec, docker_cfg, tmp_path)
      self.assertIn(str(tmp_path / "build" / "deflake" / "isos"), docker_rerun)
      self.assertIn(f"-v {tmp_path}:{tmp_path}:ro", docker_rerun)
      self.assertIn("cp -p", docker_rerun)


class UsbHarnessTest(unittest.TestCase):
  """test_boot.sh's USB keyboard scenarios against FAKE_GUEST_QEMU."""

  def setup_fake_guest(self, tmp_path: Path) -> None:
    """Copies test_boot.sh into tmp_path, with fake ISOs and FAKE_GUEST_QEMU."""
    script = (REPO_ROOT / "test_boot.sh").read_text(encoding="utf-8")
    (tmp_path / "test_boot.sh").write_text(script, encoding="utf-8")
    (tmp_path / "test_boot.sh").chmod(0o755)
    for name in ("kernel.iso", "kernel-mb1.iso", "OVMF.fd"):
      (tmp_path / name).write_bytes(b"fake")
    (tmp_path / "boot_lines.txt").write_text(
        "".join(line + "\n" for line in qemu64_boot_lines(script)),
        encoding="utf-8",
    )
    (tmp_path / "fakebin").mkdir()
    fake_qemu = tmp_path / "fakebin" / "qemu-system-x86_64"
    fake_qemu.write_text(FAKE_GUEST_QEMU, encoding="utf-8")
    fake_qemu.chmod(0o755)

  def run_test_boot(
      self, tmp_path: Path, passes: str, bug: str = "",
      extra_env: dict[str, str] | None = None,
  ) -> tuple[subprocess.CompletedProcess[str], Path, float]:
    art_dir = tmp_path / f"artifacts_{passes.replace(',', '_')}_{bug}"
    env = os.environ.copy()
    for var in ("USE_DOCKER", "KVM", "BOOT_TEST_USB", "QEMU_USB_FLAGS",
                "USB_SESSION_TIMEOUT_SECS"):
      env.pop(var, None)
    env.update({
        "PATH": f"{tmp_path / 'fakebin'}:{env.get('PATH', '')}",
        "OVMF": str(tmp_path / "OVMF.fd"),
        "BOOT_TEST_JOBS": "2",
        "FAKE_BOOT_LINES": str(tmp_path / "boot_lines.txt"),
        "FAKE_BUG": bug,
        **(extra_env or {}),
    })
    started = time.monotonic()
    proc = subprocess.run(
        [str(tmp_path / "test_boot.sh"), f"--passes={passes}",
         f"--artifact-dir={art_dir}", "kernel.iso", "kernel-mb1.iso"],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    return proc, art_dir, time.monotonic() - started

  def test_test_boot_sh_fails_on_failure_during_the_last_checks(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      self.setup_fake_guest(tmp_path)
      # Without the bug, the bios pass passes (without the USB scenarios).
      proc, art_dir, _ = self.run_test_boot(
          tmp_path, "bios", extra_env={"BOOT_TEST_USB": "0"})
      self.assertEqual(proc.returncode, 0, (proc.stdout + proc.stderr)[-3000:])
      # The stress test fails while test_boot.sh dumps the VGA text buffer,
      # after verify_serial_log: the saved serial log shows the failure, so
      # the pass fails.
      proc, art_dir, _ = self.run_test_boot(
          tmp_path, "bios", "late_failure", extra_env={"BOOT_TEST_USB": "0"})
      self.assertEqual(proc.returncode, 1, (proc.stdout + proc.stderr)[-3000:])
      self.assertEqual(
          (art_dir / "bios" / "result").read_text(encoding="utf-8").strip(),
          "FAIL",
      )
      self.assertIn(
          "random_multicpu_stress: FAIL (late)",
          (art_dir / "bios" / "serial.log").read_text(encoding="latin-1"),
      )
      output = (art_dir / "bios" / "output.log").read_text(encoding="utf-8")
      self.assertIn("Verified VGA text buffer", output)
      self.assertEqual(deflake.first_harness_failure(output),
                       "FAIL: Found a failure line in serial output:")

  def test_test_boot_sh_usb_scenarios_with_fake_guest(self) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      tmp_path = Path(tmp)
      self.setup_fake_guest(tmp_path)

      # 1. With an ideal driver every scenario of both passes passes, and so
      # do the usual checks after the session.
      proc, art_dir, _ = self.run_test_boot(tmp_path, "bios,uefi")
      out = proc.stdout + proc.stderr
      self.assertEqual(proc.returncode, 0, out[-3000:])
      self.assertIn("USB keyboard scenarios passed in: bios uefi.", proc.stdout)
      scenarios = {
          "bios": ("usb_serial_sync", "usb_token", "usb_shift_backspace",
                   "usb_editing_edges", "usb_serial_mixed", "usb_unplug",
                   "ps2_typing", "ps2_caps", "usb_hotplug_add", "usb_replug",
                   "usb_second_keyboard", "usb_long_line", "usb_hub_single",
                   "usb_hub_chained", "usb_hub_hotplug", "usb_hub_replug",
                   "usb_commands"),
          "uefi": ("usb_serial_sync", "usb_token", "usb_shift_backspace",
                   "usb_unplug", "usb_hub_single", "usb_root_controller",
                   "usb_editing_edges", "usb_serial_after_usb", "usb_replug",
                   "usb_bridged_controller", "usb_hotplug_add",
                   "usb_hub_chained", "usb_hub_hotplug", "usb_hub_replug",
                   "usb_commands", "ps2_typing"),
      }
      for p, names in scenarios.items():
        output = (art_dir / p / "output.log").read_text(encoding="utf-8")
        for name in names:
          self.assertIn("USB %s: " % name, output)
        self.assertIn("USB session: all scenarios passed", output)
        serial = (art_dir / p / "serial.log").read_text(encoding="latin-1")
        self.assertIn("[TEST] driver_session_cleanup: PASS", serial)
      self.assertEqual(sorted(os.listdir(tmp_path / "build")), [])

      # 2. A driver that ignores Shift on digits and symbols fails the
      # Shift/Backspace scenario (and only the bios pass runs).
      proc, art_dir, _ = self.run_test_boot(tmp_path, "bios",
                                            "symbols_unshifted")
      self.assertEqual(proc.returncode, 1)
      output = (art_dir / "bios" / "output.log").read_text(encoding="utf-8")
      self.assertTrue(
          (deflake.first_harness_failure(output) or "").startswith(
              "FAIL: usb_shift_backspace: "),
          output[:2000],
      )

      # 3. A kernel that ignores the interactive boot session (as before the
      # driver exists) fails at once instead of timing out.
      proc, art_dir, elapsed = self.run_test_boot(tmp_path, "uefi",
                                                  "noninteractive")
      self.assertEqual(proc.returncode, 1)
      output = (art_dir / "uefi" / "output.log").read_text(encoding="utf-8")
      self.assertTrue(
          (deflake.first_harness_failure(output) or "").startswith(
              "FAIL: usb_session: the init session ended before the boot "
              "self-tests passed"),
          output[:2000],
      )
      self.assertLess(elapsed, 30)
      # 4. A shell prompt ('sh> ') interleaved before '[PS2KBD] controller
      # i8042 ready (irq1)' on the same serial line still matches PS2_READY_RE.
      proc, _, _ = self.run_test_boot(tmp_path, "bios", "ps2_prefixed")
      self.assertEqual(
          proc.returncode, 0, (proc.stdout + proc.stderr)[-3000:]
      )


if __name__ == "__main__":
  unittest.main()
