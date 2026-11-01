#!/usr/bin/env python3
"""Deflaking runner and failure-diagnosis system for protos QEMU boot tests.

Runs the built-in integration configurations (`kvm24`,
`docker:{bios,uefi,mb1,max,smep}`, `host:{bios,uefi,mb1,max,smep}`) or custom
developer QEMU configurations for a run budget (`--runs N`) or wall-clock time
budget (`--duration DUR`), strictly one VM at a time under a per-boot flock
(`--lock FILE`).

For every run it parses the kernel's serial output, attributes results across
the boot self-tests (`kBootTests` in `selftest/boot_selftest.cpp`) plus
pseudo-tests (`random_multicpu_stress`, `@panic`, `@boot`, `@harness`), and
reports per-configuration and per-test pass/fail counts with Wilson 95%
confidence intervals and flaky/broken test classifications. For every failed
run it archives the booted ISO by sha256 and saves the full serial log, exact
QEMU command line, kernel build identity, logged seeds, and an executable
`rerun.sh` script that boots the exact same configuration and archived ISO.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import Any, Callable, Iterator, Mapping, Sequence

SCHEMA = "protos-deflake/1"
PASS_NAMES: tuple[str, ...] = ("bios", "uefi", "mb1", "max", "smep")
DOCKER_CONFIG_NAMES: tuple[str, ...] = tuple(f"docker:{p}" for p in PASS_NAMES)
HOST_CONFIG_NAMES: tuple[str, ...] = tuple(f"host:{p}" for p in PASS_NAMES)
BUILTIN_CONFIG_NAMES: tuple[str, ...] = (
    "kvm24",
    *DOCKER_CONFIG_NAMES,
    *HOST_CONFIG_NAMES,
)

ISO_PLACEHOLDER = "{iso}"
KVM24_ARGV: tuple[str, ...] = (
    "qemu-system-x86_64",
    "-enable-kvm",
    "-cpu",
    "host",
    "-smp",
    "24",
    "-m",
    "256M",
    "-serial",
    "mon:stdio",
    "-cdrom",
    ISO_PLACEHOLDER,
    "-display",
    "none",
)

BOOT_TEST_ENV_VARS: tuple[str, ...] = (
    "BIOS_TIMEOUT_SECS",
    "UEFI_TIMEOUT_SECS",
    "MIN_STRESS_PASSES",
    "OVMF",
    "OVMF_VARS",
    "REQUIRE_UEFI",
    "BOOT_TEST_JOBS",
    "BOOT_TEST_USB",
    "QEMU_USB_FLAGS",
    "USB_SESSION_TIMEOUT_SECS",
)
# The test_boot.sh passes that also run the USB keyboard scenarios (unless
# BOOT_TEST_USB=0), which add up to USB_SESSION_TIMEOUT_SECS (default 60) to
# the pass timeout.
USB_PASS_NAMES: tuple[str, ...] = ("bios", "uefi")
DEFAULT_USB_SESSION_TIMEOUT_S = 60.0
# The QEMU arguments with which test_boot.sh asks the kernel for its
# interactive boot session in the USB passes.
INTERACTIVE_SESSION_ARGS: tuple[str, ...] = (
    "-fw_cfg",
    "name=opt/protos/boot_session,string=interactive",
)
# The tag of the Docker image (the Makefile's DOCKER_IMAGE).
DEFAULT_DOCKER_IMAGE = "x86-64-kernel-build"
# How much longer than test_boot.sh's own bound for a pass (its timeout plus
# the USB scenarios) a test_boot run may take before deflake stops it (the
# post-boot checks, draining after a failure, and a loaded host).
TEST_BOOT_WATCHDOG_MARGIN_S = 120.0

FLAG_OPTIONS: frozenset[str] = frozenset({
    "-enable-kvm",
    "-no-reboot",
    "-no-shutdown",
    "-nographic",
    "-S",
    "-snapshot",
    "-nodefaults",
    "-no-hpet",
})

FORBIDDEN_OVERRIDE_OPTIONS: frozenset[str] = frozenset({
    "-cdrom",
    "-serial",
    "-monitor",
    "-nographic",
    "-daemonize",
})

def default_lock_path(env: Mapping[str, str] | None = None) -> Path:
    """Returns a per-user lock path under XDG_RUNTIME_DIR or /tmp."""
    src_env = os.environ if env is None else env
    runtime_dir = src_env.get("XDG_RUNTIME_DIR")
    if runtime_dir:
        return Path(runtime_dir) / "protos_deflake" / "host.lock"
    return Path(f"/tmp/protos_deflake-{os.getuid()}") / "host.lock"


DEFAULT_LOCK_PATH = default_lock_path()

GREETING = "Hello, x86-64 Kernel World!"
SUITE_PASSED_MARKER = "[TEST] ALL BOOT SELF-TESTS PASSED"
SUITE_FAILED_MARKER = "[TEST] BOOT SELF-TESTS FAILED"
STRESS_PASS_MARKER = "[STRESS] random_multicpu_stress: PASS"

RESULT_RE = re.compile(r"^\[TEST\] ([a-z0-9_]+): (PASS|FAIL)$")
GARBLED_RESULT_RE = re.compile(r"(?:^|[\s\]])([a-z0-9_]+): (PASS|FAIL)$")
FAILURE_RE = re.compile(r"FAIL|\[FATAL\]")
ONLINE_RE = re.compile(r"\[SMP\] Online CPUs: (\d+)/(\d+)")
STRESS_FAIL_RE = re.compile(
    r"\[STRESS\] random_multicpu_stress: FAIL \((.*?)\) seed=(0x[0-9A-Fa-f]+)"
)
EXPECTATION_RE = re.compile(
    r"\[TEST\] expectation failed: (.*) at (\S+:\d+)$"
)
SEED_RE = re.compile(r"seed=(0x[0-9A-Fa-f]+)")
SUMMARY_LINE_RE = re.compile(
    r"^(SUCCESS|FAILURE): boot verification (?:passed|failed) \(([^)]*)\)"
)
DURATION_PART_RE = re.compile(r"(\d+(?:\.\d+)?)\s*([smhSMH]?)")
GRUB_VERSION_RE = re.compile(rb"GRUB [0-9][0-9.]*-[0-9A-Za-z.+~]+")
GCC_VERSION_RE = re.compile(rb"GCC: \([^)]+\) [0-9][0-9A-Za-z.+~-]*")

Z95 = 1.959963984540054


class ConfigError(ValueError):
    """Raised on invalid CLI arguments or configuration specifications."""


def load_boot_test_names(repo: Path) -> tuple[str, ...]:
    """Loads the ordered boot self-test names from kBootTests in boot_selftest.cpp."""
    selftest_path = repo / "selftest" / "boot_selftest.cpp"
    if not selftest_path.is_file():
        raise ConfigError(f"boot_selftest.cpp not found at '{selftest_path}'")
    text = selftest_path.read_text(encoding="utf-8", errors="replace")
    marker = "constexpr BootTest kBootTests[] = {"
    if marker not in text:
        raise ConfigError(f"kBootTests[] table not found in '{selftest_path}'")
    block = text.split(marker, 1)[1].split("};", 1)[0]
    names = re.findall(r'^\s*\{"([a-z0-9_]+)",\s*\w+\},', block, re.MULTILINE)
    if not names:
        raise ConfigError(f"No boot test entries parsed from '{selftest_path}'")
    if len(names) != len(set(names)):
        raise ConfigError(f"Duplicate boot test names in '{selftest_path}'")
    return tuple(names)


@dataclasses.dataclass
class SerialReport:
    """Structured parse result of a kernel serial log."""

    greeting_count: int = 0
    online: list[tuple[int, int]] = dataclasses.field(default_factory=list)
    suite_end: str | None = None
    results: dict[str, list[str]] = dataclasses.field(default_factory=dict)
    garbled: set[str] = dataclasses.field(default_factory=set)
    stress_pass_count: int = 0
    stress_fails: list[tuple[str, str]] = dataclasses.field(default_factory=list)
    fatal_lines: list[tuple[int, str]] = dataclasses.field(default_factory=list)
    failure_lines: list[tuple[int, str]] = dataclasses.field(default_factory=list)
    expectations: list[tuple[str, str]] = dataclasses.field(default_factory=list)
    seeds: list[tuple[int, str, str]] = dataclasses.field(default_factory=list)
    diagnostics: list[str] = dataclasses.field(default_factory=list)
    line_count: int = 0
    last_line: str = ""


class SerialParser:
    """Incremental parser for kernel serial output."""

    def __init__(self, registry: Sequence[str] = ()) -> None:
        self._registry: tuple[str, ...] = tuple(registry)
        self._registry_set: frozenset[str] = frozenset(registry)
        self._partial: str = ""
        self._report = SerialReport()
        self._finished: bool = False

    def feed(self, data: bytes) -> None:
        """Feeds raw serial bytes into the parser."""
        if not data:
            return
        chunk = data.decode("latin-1").replace("\r", "")
        combined = self._partial + chunk
        lines = combined.split("\n")
        self._partial = lines.pop()
        for line in lines:
            self._process_line(line)

    def finish(self) -> SerialReport:
        """Flushes any trailing partial line and returns the final SerialReport."""
        if not self._finished:
            self._finished = True
            if self._partial:
                self._process_line(self._partial)
                self._partial = ""
        return self._report

    def report(self) -> SerialReport:
        """Returns the current SerialReport."""
        return self._report

    def has_failure(self) -> bool:
        """Returns True if any processed or partial line matches FAILURE_RE."""
        if self._report.failure_lines:
            return True
        if self._partial and FAILURE_RE.search(self._partial):
            return True
        return False

    def has_suite_end_or_panic(self) -> bool:
        """Returns True if the boot self-test suite finished or a panic occurred."""
        if self._report.suite_end is not None or self._report.fatal_lines:
            return True
        if self._partial and (
            SUITE_PASSED_MARKER in self._partial
            or SUITE_FAILED_MARKER in self._partial
            or "[FATAL]" in self._partial
        ):
            return True
        return False

    def meets_pass_condition(
        self, expected_cpus: int, min_stress: int = 3
    ) -> bool:
        """Returns True if the live log has every pass marker and no failure."""
        r = self._report
        return (
            not self.has_failure()
            and r.greeting_count >= 1
            and (expected_cpus, expected_cpus) in r.online
            and r.suite_end == "passed"
            and r.stress_pass_count >= min_stress
        )

    def _process_line(self, line: str) -> None:
        r = self._report
        r.line_count += 1
        lineno = r.line_count
        if line:
            r.last_line = line

        if GREETING in line:
            r.greeting_count += 1

        for m_online in ONLINE_RE.finditer(line):
            r.online.append((int(m_online.group(1)), int(m_online.group(2))))

        if SUITE_FAILED_MARKER in line:
            r.suite_end = "failed"
        elif SUITE_PASSED_MARKER in line and r.suite_end != "failed":
            r.suite_end = "passed"

        if STRESS_PASS_MARKER in line:
            r.stress_pass_count += 1

        m_stress_fail = STRESS_FAIL_RE.search(line)
        if m_stress_fail:
            r.stress_fails.append(
                (m_stress_fail.group(1), m_stress_fail.group(2))
            )

        m_res = RESULT_RE.match(line)
        if m_res:
            name, status = m_res.group(1), m_res.group(2)
            r.results.setdefault(name, []).append(status)
        elif self._registry_set:
            m_garbled = GARBLED_RESULT_RE.search(line)
            if m_garbled and m_garbled.group(1) in self._registry_set:
                name, status = m_garbled.group(1), m_garbled.group(2)
                r.results.setdefault(name, []).append(status)
                r.garbled.add(name)

        if "[FATAL]" in line:
            r.fatal_lines.append((lineno, line))

        if FAILURE_RE.search(line):
            r.failure_lines.append((lineno, line))

        m_exp = EXPECTATION_RE.search(line)
        if m_exp:
            r.expectations.append((m_exp.group(1), m_exp.group(2)))

        for m_seed in SEED_RE.finditer(line):
            r.seeds.append((lineno, m_seed.group(1), line))

        if (
            "missed deadline" in line
            or "expectation failed:" in line
            or "[FATAL]" in line
            or FAILURE_RE.search(line)
        ):
            r.diagnostics.append(line)


def parse_serial(
    data: bytes, registry: Sequence[str] = ()
) -> SerialReport:
    """Parses raw serial log bytes in one shot."""
    parser = SerialParser(registry)
    parser.feed(data)
    return parser.finish()


def phase_of(report: SerialReport, names: Sequence[str]) -> str:
    """Returns a human-readable boot phase description for where the log ended."""
    if report.suite_end == "passed":
        stress_idx = max(
            report.stress_pass_count + 1,
            len([ln for _, _, ln in report.seeds if "[STRESS]" in ln]),
            1,
        )
        return f"stress pass {stress_idx}"
    if report.suite_end == "failed":
        return "after suite (failed)"
    if not report.results:
        return "early boot"
    last_idx = -1
    for i, name in enumerate(names):
        if name in report.results:
            last_idx = i
    if 0 <= last_idx + 1 < len(names):
        return f"during {names[last_idx + 1]}"
    if names and last_idx == len(names) - 1:
        return f"after {names[-1]}"
    return "during boot self-tests"


@dataclasses.dataclass(frozen=True)
class BootEnd:
    """Execution timing and termination facts of a QEMU boot."""

    t_start: float = 0.0
    t_pass: float | None = None
    t_fail: float | None = None
    exited_rc: int | None = None
    timed_out: bool = False
    stopped_by_tool: bool = False


@dataclasses.dataclass(frozen=True)
class Verdict:
    """Verdict of a single boot run."""

    status: str  # "PASS" | "FAIL" | "SKIPPED" | "ERROR"
    reasons: tuple[str, ...] = ()


def _ordered_failed_test_names(
    report: SerialReport, registry: Sequence[str] = ()
) -> list[str]:
    ordered: list[str] = []
    seen: set[str] = set()
    for name in registry:
        if "FAIL" in report.results.get(name, ()):
            ordered.append(name)
            seen.add(name)
    for name, vals in report.results.items():
        if name not in seen and "FAIL" in vals:
            ordered.append(name)
            seen.add(name)
    return ordered


def judge_qemu_boot(
    report: SerialReport,
    end: BootEnd,
    expected_cpus: int,
    timeout_s: float = 60.0,
    min_stress: int = 3,
    registry: Sequence[str] = (),
) -> Verdict:
    """Evaluates a direct QEMU boot against the acceptance rule.

    The boot passes if the pass markers (`SerialParser.meets_pass_condition`)
    appeared within `timeout_s` with exactly one greeting, no failure line was
    logged, and QEMU did not exit on its own. Otherwise the verdict lists the
    reasons.
    """
    reasons: list[str] = []

    failed_tests = _ordered_failed_test_names(report, registry)
    for name in failed_tests:
        reasons.append(f"test_failed:{name}")

    if report.stress_fails:
        reasons.append("stress_failed")

    if report.fatal_lines:
        reasons.append("panic")

    unexplained_failure_line = False
    for _, line in report.failure_lines:
        if SUITE_FAILED_MARKER in line:
            if not failed_tests:
                unexplained_failure_line = True
            continue
        if "[FATAL]" in line:
            continue
        if STRESS_FAIL_RE.search(line):
            continue
        m_res = RESULT_RE.match(line)
        if m_res and m_res.group(2) == "FAIL" and m_res.group(1) in failed_tests:
            continue
        m_garb = GARBLED_RESULT_RE.search(line)
        if (
            m_garb
            and m_garb.group(2) == "FAIL"
            and m_garb.group(1) in failed_tests
        ):
            continue
        unexplained_failure_line = True

    if unexplained_failure_line:
        reasons.append("failure_line")

    if report.greeting_count == 0:
        reasons.append("no_greeting")
    elif report.greeting_count > 1:
        reasons.append("reboot")

    if (expected_cpus, expected_cpus) not in report.online:
        reasons.append("cpus_online")

    if end.exited_rc is not None and not end.stopped_by_tool:
        reasons.append(f"qemu_exited:{end.exited_rc}")

    has_test_or_crash_failure = bool(
        failed_tests
        or report.stress_fails
        or report.fatal_lines
        or unexplained_failure_line
        or report.suite_end == "failed"
    )

    if end.t_pass is not None:
        if end.t_pass > timeout_s:
            reasons.append("timeout")
    elif not has_test_or_crash_failure:
        if end.timed_out or not any(
            r.startswith("qemu_exited:")
            or r in ("no_greeting", "reboot", "cpus_online")
            for r in reasons
        ):
            reasons.append("timeout")

    passed = (
        not reasons
        and end.t_pass is not None
        and end.t_pass <= timeout_s
        and report.greeting_count == 1
        and (expected_cpus, expected_cpus) in report.online
        and report.suite_end == "passed"
        and report.stress_pass_count >= min_stress
        and not report.failure_lines
    )
    if passed:
        return Verdict("PASS", ())
    if not reasons:
        reasons.append("timeout")
    return Verdict("FAIL", tuple(reasons))


def parse_test_boot_summary(text: str) -> dict[str, str]:
    """Parses the final SUCCESS/FAILURE summary line of test_boot.sh."""
    statuses: dict[str, str] = {}
    for line in reversed(text.replace("\r", "").splitlines()):
        m = SUMMARY_LINE_RE.match(line.strip())
        if m:
            inner = m.group(2)
            for match in re.finditer(r"([a-z0-9_]+):\s*(PASS|FAIL|SKIPPED)", inner):
                statuses[match.group(1)] = match.group(2)
            break
    return statuses


def first_harness_failure(output_log: str) -> str | None:
    """Returns the first FAIL: line from a test_boot.sh pass output log."""
    for line in output_log.replace("\r", "").splitlines():
        stripped = line.strip()
        if stripped.startswith("FAIL:"):
            return stripped
    return None


def judge_test_boot(
    result_text: str | None,
    exit_code: int | None,
    summary_status: str | None,
    output_log: str = "",
    report: SerialReport | None = None,
    watchdog_fired: bool = False,
    registry: Sequence[str] = (),
) -> Verdict:
    """Reads test_boot.sh's own verdict for a pass without re-evaluating its checks."""
    if watchdog_fired:
        return Verdict("ERROR", ("harness_watchdog",))
    if result_text is None or exit_code is None:
        return Verdict("ERROR", ("missing_harness_result",))
    if exit_code in (125, 126, 127):
        return Verdict("ERROR", (f"harness_exit:{exit_code}",))

    res = result_text.strip()
    if res == "PASS":
        if exit_code == 0 and summary_status in ("PASS", None):
            return Verdict("PASS", ())
        return Verdict("ERROR", ("harness_inconsistent",))

    if res == "SKIPPED":
        if exit_code == 0 and summary_status in ("SKIPPED", None):
            return Verdict("SKIPPED", ("pass_skipped",))
        return Verdict("ERROR", ("harness_inconsistent",))

    if res == "FAIL":
        if exit_code == 0 or summary_status not in ("FAIL", None):
            return Verdict("ERROR", ("harness_inconsistent",))
        reasons: list[str] = []
        if report is not None:
            for name in _ordered_failed_test_names(report, registry):
                reasons.append(f"test_failed:{name}")
            if report.stress_fails:
                reasons.append("stress_failed")
            if report.fatal_lines:
                reasons.append("panic")
        fail_msg = first_harness_failure(output_log)
        if fail_msg:
            if "QEMU exited unexpectedly" in fail_msg:
                reasons.append("qemu_exited")
            elif "reboot loop" in fail_msg:
                reasons.append("reboot")
            elif "did not complete within its timeout" in fail_msg:
                if report is None or not report.failure_lines:
                    reasons.append("timeout")
            elif not reasons:
                reasons.append(f"harness:{fail_msg}")
        if not reasons:
            reasons.append("test_boot_failed")
        return Verdict("FAIL", tuple(reasons))

    return Verdict("ERROR", (f"unknown_result:{res}",))


def test_outcomes(
    report: SerialReport,
    verdict: Verdict,
    names: Sequence[str],
    engine: str = "qemu",
    output_log: str = "",
) -> dict[str, str]:
    """Computes PASS / FAIL / NO_RESULT per boot self-test and pseudo-test."""
    outcomes: dict[str, str] = {}
    if verdict.status in ("ERROR", "SKIPPED"):
        for name in names:
            outcomes[name] = "NO_RESULT"
        outcomes["random_multicpu_stress"] = "NO_RESULT"
        outcomes["@panic"] = "NO_RESULT"
        outcomes["@boot"] = "NO_RESULT"
        if engine == "test_boot":
            outcomes["@harness"] = "NO_RESULT"
        return outcomes

    for name in names:
        vals = report.results.get(name)
        if vals:
            outcomes[name] = "FAIL" if "FAIL" in vals else "PASS"
        elif report.suite_end == "passed":
            outcomes[name] = "PASS"
        else:
            outcomes[name] = "NO_RESULT"

    for name, vals in report.results.items():
        if name not in outcomes:
            outcomes[name] = "FAIL" if "FAIL" in vals else "PASS"

    if report.stress_fails:
        outcomes["random_multicpu_stress"] = "FAIL"
    elif report.stress_pass_count >= 1 and (
        verdict.status == "PASS"
        or "timeout" not in verdict.reasons
        or report.stress_pass_count >= 3
    ):
        outcomes["random_multicpu_stress"] = "PASS"
    else:
        outcomes["random_multicpu_stress"] = "NO_RESULT"

    if report.fatal_lines and not report.stress_fails:
        outcomes["@panic"] = "FAIL"
    elif report.line_count == 0:
        outcomes["@panic"] = "NO_RESULT"
    else:
        outcomes["@panic"] = "PASS"

    boot_fail = False
    if engine == "qemu":
        for r in verdict.reasons:
            if (
                r in ("timeout", "reboot", "cpus_online", "no_greeting", "failure_line")
                or r.startswith("qemu_exited:")
            ):
                boot_fail = True
                break
    else:
        fail_msg = first_harness_failure(output_log) or ""
        if (
            "QEMU exited unexpectedly" in fail_msg
            or "reboot loop" in fail_msg
            or (
                "did not complete within its timeout" in fail_msg
                and not report.failure_lines
            )
        ):
            boot_fail = True
    outcomes["@boot"] = "FAIL" if boot_fail else "PASS"

    if engine == "test_boot":
        any_above_failed = any(v == "FAIL" for v in outcomes.values())
        if verdict.status == "FAIL" and not any_above_failed:
            outcomes["@harness"] = "FAIL"
        else:
            outcomes["@harness"] = "PASS"

    if verdict.status == "FAIL" and not any(
        v == "FAIL" for v in outcomes.values()
    ):
        fallback_key = "@harness" if engine == "test_boot" else "@boot"
        outcomes[fallback_key] = "FAIL"

    return outcomes


def classify(passed: int, failed: int) -> str:
    """Classifies a test's stability across runs of the same (config, build)."""
    if failed >= 1 and passed >= 1:
        return "flaky"
    if failed >= 1 and passed == 0:
        return "broken"
    if failed == 0 and passed >= 1:
        return "stable"
    return "no_data"


def wilson(
    failures: int, n: int, z: float = Z95
) -> tuple[float, float] | None:
    """Computes the 95% Wilson score confidence interval for failure rate."""
    if n <= 0:
        return None
    p = failures / n
    z2 = z * z
    denom = 1.0 + z2 / n
    center = (p + z2 / (2.0 * n)) / denom
    half = (
        z
        * math.sqrt((p * (1.0 - p)) / n + z2 / (4.0 * n * n))
        / denom
    )
    lo = max(0.0, center - half)
    hi = min(1.0, center + half)
    return (round(lo, 4), round(hi, 4))


@dataclasses.dataclass(frozen=True)
class Config:
    """Configuration specification for a deflaking target."""

    name: str
    engine: str  # "qemu" | "test_boot"
    qemu_argv: tuple[str, ...] = ()
    expected_cpus: int = 0
    mode: str = ""  # "docker" | "host" for test_boot
    pass_name: str = ""
    timeout_s: float = 60.0
    description: str = ""
    custom_iso: Path | None = None

    @property
    def slug(self) -> str:
        return re.sub(r"[^A-Za-z0-9_.-]+", "-", self.name)


def tokenize_qemu_options(
    argv: Sequence[str],
) -> list[tuple[str, tuple[str, ...]]]:
    """Splits QEMU arguments into (option, args_tuple) tokens."""
    tokens: list[tuple[str, tuple[str, ...]]] = []
    i = 0
    n = len(argv)
    while i < n:
        tok = argv[i]
        if tok.startswith("-"):
            if tok in FLAG_OPTIONS:
                tokens.append((tok, ()))
                i += 1
            else:
                if i + 1 >= n:
                    raise ConfigError(f"QEMU option '{tok}' requires a value")
                tokens.append((tok, (argv[i + 1],)))
                i += 2
        else:
            tokens.append(("", (tok,)))
            i += 1
    return tokens


def _option_family(opt: str) -> str:
    if opt in ("-enable-kvm", "-accel"):
        return "accel"
    if opt in ("-machine", "-M"):
        return "machine"
    return opt


def has_kvm_enabled(argv: Sequence[str]) -> bool:
    """Returns True if argv enables KVM via -enable-kvm or -accel/-machine."""
    i = 0
    n = len(argv)
    while i < n:
        tok = argv[i]
        if tok == "-enable-kvm":
            return True
        if tok in ("-accel", "-machine", "-M") and i + 1 < n:
            val = argv[i + 1]
            if "kvm" in val.split(",") or "accel=kvm" in val or val.startswith("kvm"):
                return True
            i += 2
            continue
        i += 1
    return False


def validate_qemu_argv(argv: Sequence[str]) -> None:
    """Validates a QEMU argument vector, refusing -cpu host without KVM."""
    kvm = has_kvm_enabled(argv)
    for i, tok in enumerate(argv):
        if tok == "-cpu" and i + 1 < len(argv):
            cpu_val = argv[i + 1]
            cpu_model = cpu_val.split(",", 1)[0].strip()
            if cpu_model == "host" and not kvm:
                raise ConfigError(
                    "Refusing '-cpu host' without KVM enabled "
                    "(specify -enable-kvm or a TCG-supported -cpu model such as 'max')"
                )


def expected_cpus(argv: Sequence[str]) -> int:
    """Parses the expected online CPU count from the last -smp option in argv."""
    smp_val: str | None = None
    i = 0
    while i < len(argv):
        if argv[i] == "-smp":
            if i + 1 >= len(argv):
                raise ConfigError("QEMU option '-smp' requires a value")
            smp_val = argv[i + 1]
            i += 2
        else:
            i += 1
    if smp_val is None:
        return 1

    parts = [p.strip() for p in smp_val.split(",") if p.strip()]
    if not parts:
        raise ConfigError(f"Invalid empty -smp value '{smp_val}'")

    explicit_cpus: int | None = None
    topology: dict[str, int] = {
        "sockets": 1,
        "dies": 1,
        "clusters": 1,
        "cores": 1,
        "threads": 1,
    }
    saw_topology = False

    for idx, part in enumerate(parts):
        if "=" not in part:
            if idx == 0 and re.fullmatch(r"\d+", part):
                explicit_cpus = int(part)
                continue
            raise ConfigError(f"Cannot parse -smp component '{part}' in '{smp_val}'")
        k, v = part.split("=", 1)
        if not re.fullmatch(r"\d+", v):
            raise ConfigError(f"Non-integer -smp parameter '{part}' in '{smp_val}'")
        iv = int(v)
        if k in ("cpus", "n"):
            explicit_cpus = iv
        elif k in topology:
            topology[k] = iv
            saw_topology = True
        elif k == "maxcpus":
            continue
        else:
            raise ConfigError(f"Unknown -smp parameter '{k}' in '{smp_val}'")

    count = explicit_cpus if explicit_cpus is not None else (
        math.prod(topology.values()) if saw_topology else 0
    )
    if count <= 0:
        raise ConfigError(f"Invalid CPU count {count} parsed from '-smp {smp_val}'")
    return count


def apply_qemu_overrides(
    base: Sequence[str], overrides_str: str
) -> tuple[str, ...]:
    """Applies developer QEMU option overrides onto the base kvm24 argv."""
    try:
        override_words = shlex.split(overrides_str)
    except ValueError as e:
        raise ConfigError(f"Invalid --qemu-args '{overrides_str}': {e}") from e
    if not override_words:
        raise ConfigError("--qemu-args must not be empty")

    if Path(override_words[0]).name.startswith("qemu-system-"):
        raise ConfigError(
            "Pass only QEMU options to --qemu-args, or use --qemu-cmd for a full command"
        )

    override_tokens = tokenize_qemu_options(override_words)
    for opt, _ in override_tokens:
        if not opt:
            raise ConfigError(
                f"Unexpected positional argument in --qemu-args: '{overrides_str}'"
            )
        if opt in FORBIDDEN_OVERRIDE_OPTIONS:
            raise ConfigError(
                f"Option '{opt}' cannot be overridden in --qemu-args "
                "(use --iso for ISO path; serial/display are managed by deflake.py)"
            )

    override_families = {_option_family(opt) for opt, _ in override_tokens}
    base_head = base[0]
    base_tokens = tokenize_qemu_options(base[1:])

    result: list[str] = [base_head]
    inserted_families: set[str] = set()

    for opt, vals in base_tokens:
        fam = _option_family(opt)
        if fam in override_families:
            if fam not in inserted_families:
                for o_opt, o_vals in override_tokens:
                    if _option_family(o_opt) == fam:
                        result.append(o_opt)
                        result.extend(o_vals)
                inserted_families.add(fam)
        else:
            if opt:
                result.append(opt)
            result.extend(vals)

    for o_opt, o_vals in override_tokens:
        fam = _option_family(o_opt)
        if fam not in inserted_families:
            result.append(o_opt)
            result.extend(o_vals)
    for o_opt, _ in override_tokens:
        inserted_families.add(_option_family(o_opt))

    validate_qemu_argv(result)
    return tuple(result)


def parse_qemu_command(cmd_str: str) -> tuple[tuple[str, ...], Path]:
    """Parses a full QEMU command string into a normalized argv template and ISO path."""
    try:
        words = shlex.split(cmd_str)
    except ValueError as e:
        raise ConfigError(f"Invalid --qemu-cmd '{cmd_str}': {e}") from e
    if not words:
        raise ConfigError("--qemu-cmd must not be empty")

    cdrom_idx = -1
    serial_vals: list[str] = []
    has_display = False
    i = 1 if not words[0].startswith("-") else 0
    if not Path(words[0]).name.startswith("qemu-system-"):
        raise ConfigError(
            f"--qemu-cmd must start with a qemu-system-* binary, got '{words[0]}'"
        )

    while i < len(words):
        tok = words[i]
        if tok == "-cdrom":
            if i + 1 >= len(words):
                raise ConfigError("'-cdrom' in --qemu-cmd requires a path")
            cdrom_idx = i + 1
            i += 2
        elif tok == "-serial":
            if i + 1 >= len(words):
                raise ConfigError("'-serial' in --qemu-cmd requires a value")
            serial_vals.append(words[i + 1])
            i += 2
        elif tok == "-display":
            has_display = True
            i += 2
        elif tok == "-nographic":
            has_display = True
            i += 1
        elif tok in FLAG_OPTIONS:
            i += 1
        elif tok.startswith("-"):
            i += 2
        else:
            i += 1

    if cdrom_idx < 0:
        raise ConfigError("--qemu-cmd must include '-cdrom <iso_path>'")
    if not any(v in ("mon:stdio", "stdio") for v in serial_vals):
        raise ConfigError(
            "--qemu-cmd must route serial to stdio ('-serial mon:stdio' or '-serial stdio')"
        )

    iso_path = Path(words[cdrom_idx])
    normalized = list(words)
    normalized[cdrom_idx] = ISO_PLACEHOLDER
    if not has_display:
        normalized.extend(["-display", "none"])

    validate_qemu_argv(normalized)
    return tuple(normalized), iso_path


def parse_duration(text: str) -> float:
    """Parses a duration string such as '90s', '30m', '1h30m' into seconds."""
    s = text.strip()
    if not s:
        raise ConfigError("Duration must not be empty")
    pos = 0
    total = 0.0
    for m in DURATION_PART_RE.finditer(s):
        if s[pos : m.start()].strip():
            raise ConfigError(f"Invalid duration '{text}'")
        val = float(m.group(1))
        unit = m.group(2).lower() or "s"
        mult = {"s": 1.0, "m": 60.0, "h": 3600.0}[unit]
        total += val * mult
        pos = m.end()
    if pos != len(s) or total <= 0:
        raise ConfigError(f"Invalid duration '{text}' (must be > 0, e.g. '30s', '5m')")
    return total


def parse_timeouts(
    values: Sequence[str], selected_configs: Sequence[Config]
) -> dict[str, float]:
    """Parses --timeout specs, rejecting --timeout for any built-in config."""
    if not values:
        return {}
    by_name = {c.name: c for c in selected_configs}
    builtin_set = frozenset(BUILTIN_CONFIG_NAMES)
    overrides: dict[str, float] = {}

    for raw in values:
        if "=" in raw:
            cfg_name, val_str = raw.split("=", 1)
            cfg_name = cfg_name.strip()
            if cfg_name in builtin_set or cfg_name in ("all", "docker", "host"):
                raise ConfigError(
                    f"--timeout is rejected for built-in configuration "
                    f"'{cfg_name}'; built-in configurations must use their "
                    "fixed acceptance timeouts"
                )
            if cfg_name not in by_name:
                raise ConfigError(
                    f"--timeout references unknown configuration '{cfg_name}'"
                )
            secs = parse_duration(val_str)
            overrides[cfg_name] = secs
        else:
            builtin_selected = [
                c.name for c in selected_configs if c.name in builtin_set
            ]
            if builtin_selected:
                raise ConfigError(
                    "--timeout is rejected when built-in configurations are "
                    f"selected ({', '.join(builtin_selected)}); allowed only "
                    "for custom configs"
                )
            secs = parse_duration(raw)
            for c in selected_configs:
                overrides[c.name] = secs
    return overrides


def _make_builtin_config(name: str) -> Config:
    if name == "kvm24":
        return Config(
            name="kvm24",
            engine="qemu",
            qemu_argv=KVM24_ARGV,
            expected_cpus=24,
            timeout_s=60.0,
            description="KVM -cpu host -smp 24 -m 256M BIOS",
        )
    if name.startswith("docker:") or name.startswith("host:"):
        mode, pass_name = name.split(":", 1)
        if pass_name not in PASS_NAMES:
            raise ConfigError(
                f"Unknown pass '{pass_name}' in '{name}' (passes: {', '.join(PASS_NAMES)})"
            )
        base_timeout = 90.0 if pass_name == "uefi" else 60.0
        return Config(
            name=name,
            engine="test_boot",
            mode=mode,
            pass_name=pass_name,
            timeout_s=base_timeout,
            description=f"test_boot.sh {pass_name} pass ({mode})",
        )
    raise ConfigError(f"Unknown configuration '{name}'")


def _env_seconds(env: Mapping[str, str], name: str, default: float) -> float:
    """Returns env[name] as seconds if it is a positive whole number, else default."""
    raw = env.get(name, "")
    if raw.isdigit() and int(raw) > 0:
        return float(raw)
    return default


def harness_watchdog_s(config: Config, env: Mapping[str, str]) -> float:
    """Returns how long one test_boot.sh run of `config` may take.

    test_boot.sh bounds a pass itself: by its timeout (BIOS_TIMEOUT_SECS or
    UEFI_TIMEOUT_SECS, which the environment may raise above the configuration's
    acceptance timeout) plus, in the bios and uefi passes, USB_SESSION_TIMEOUT_SECS
    for the USB keyboard scenarios (unless BOOT_TEST_USB=0). The watchdog adds
    TEST_BOOT_WATCHDOG_MARGIN_S, so that test_boot.sh, not the watchdog, reports
    a pass that hangs.
    """
    pass_timeout = config.timeout_s
    if config.pass_name:
        var = "UEFI_TIMEOUT_SECS" if config.pass_name == "uefi" else "BIOS_TIMEOUT_SECS"
        pass_timeout = max(pass_timeout, _env_seconds(env, var, config.timeout_s))
    usb_s = 0.0
    if config.pass_name in USB_PASS_NAMES and env.get("BOOT_TEST_USB", "1") != "0":
        usb_s = _env_seconds(
            env, "USB_SESSION_TIMEOUT_SECS", DEFAULT_USB_SESSION_TIMEOUT_S
        )
    return pass_timeout + usb_s + TEST_BOOT_WATCHDOG_MARGIN_S


def expand_configs(
    config_specs: Sequence[str] = (),
    qemu_args_list: Sequence[str] = (),
    qemu_cmd_list: Sequence[str] = (),
    timeout_specs: Sequence[str] = (),
) -> list[Config]:
    """Expands CLI configuration arguments into ordered Config objects."""
    configs: list[Config] = []
    seen_names: set[str] = set()
    custom_idx = 0

    def add_config(cfg: Config) -> None:
        if cfg.name not in seen_names:
            seen_names.add(cfg.name)
            configs.append(cfg)

    def add_custom_from_overrides(spec: str) -> None:
        nonlocal custom_idx
        custom_idx += 1
        cname = f"custom{custom_idx}"
        argv = apply_qemu_overrides(KVM24_ARGV, spec)
        cpus = expected_cpus(argv)
        add_config(
            Config(
                name=cname,
                engine="qemu",
                qemu_argv=argv,
                expected_cpus=cpus,
                timeout_s=60.0,
                description=f"kvm24 + {spec}",
            )
        )

    for raw in config_specs:
        if raw.strip().startswith("-"):
            add_custom_from_overrides(raw.strip())
            continue
        if raw.strip().startswith("custom:"):
            add_custom_from_overrides(raw.strip()[len("custom:") :].strip())
            continue
        for token in raw.split(","):
            tok = token.strip()
            if not tok:
                continue
            if tok == "all":
                for builtin_name in BUILTIN_CONFIG_NAMES:
                    add_config(_make_builtin_config(builtin_name))
            elif tok == "docker":
                for d_name in DOCKER_CONFIG_NAMES:
                    add_config(_make_builtin_config(d_name))
            elif tok == "host":
                for h_name in HOST_CONFIG_NAMES:
                    add_config(_make_builtin_config(h_name))
            elif tok in BUILTIN_CONFIG_NAMES:
                add_config(_make_builtin_config(tok))
            elif tok.startswith("-"):
                add_custom_from_overrides(tok)
            else:
                raise ConfigError(
                    f"Unknown --config '{tok}' "
                    "(valid: kvm24, docker:<pass>, host:<pass>, all, docker, host)"
                )

    for qa in qemu_args_list:
        add_custom_from_overrides(qa)

    for qc in qemu_cmd_list:
        custom_idx += 1
        cname = f"custom{custom_idx}"
        argv, custom_iso = parse_qemu_command(qc)
        cpus = expected_cpus(argv)
        add_config(
            Config(
                name=cname,
                engine="qemu",
                qemu_argv=argv,
                expected_cpus=cpus,
                timeout_s=60.0,
                description=f"cmd: {qc}",
                custom_iso=custom_iso,
            )
        )

    if not configs:
        for builtin_name in BUILTIN_CONFIG_NAMES:
            add_config(_make_builtin_config(builtin_name))

    timeout_map = parse_timeouts(timeout_specs, configs)
    if timeout_map:
        configs = [
            dataclasses.replace(c, timeout_s=timeout_map[c.name])
            if c.name in timeout_map
            else c
            for c in configs
        ]

    return configs


def sha256_file(path: Path) -> str:
    """Computes the hex sha256 digest of a file."""
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            chunk = f.read(1024 * 1024)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def archive_iso(path: Path, store: Path) -> tuple[Path, str]:
    """Copies an ISO into a sha256-addressed store and marks it read-only."""
    if not path.is_file():
        raise ConfigError(f"ISO file '{path}' not found")
    digest = sha256_file(path)
    store.mkdir(parents=True, exist_ok=True)
    dest = store / f"{digest}.iso"
    if not dest.is_file() or sha256_file(dest) != digest:
        tmp = store / f".{digest}.{os.getpid()}.tmp"
        shutil.copy2(path, tmp)
        os.chmod(tmp, 0o444)
        tmp.replace(dest)
    return dest.resolve(), digest


def _git_dirty(repo: Path) -> bool | None:
    """Returns whether `git status --porcelain` sees uncommitted changes, or None."""
    if not (repo / ".git").exists():
        return None
    try:
        res = subprocess.run(
            ["git", "--no-optional-locks", "status", "--porcelain"],
            cwd=repo,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if res.returncode != 0:
        return None
    return bool(res.stdout.strip())


def vcs_provenance(repo: Path) -> dict[str, Any]:
    """Queries read-only jj or git working-copy provenance without mutating VCS state."""
    if (repo / ".jj").is_dir():
        try:
            log_res = subprocess.run(
                [
                    "jj",
                    "log",
                    "--ignore-working-copy",
                    "-r",
                    "@",
                    "--no-graph",
                    "-T",
                    'change_id ++ " " ++ commit_id',
                ],
                cwd=repo,
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            diff_res = subprocess.run(
                ["jj", "diff", "--ignore-working-copy", "--git"],
                cwd=repo,
                capture_output=True,
                timeout=10,
                check=False,
            )
            if log_res.returncode == 0:
                parts = log_res.stdout.strip().split()
                change_id = parts[0] if len(parts) >= 1 else ""
                commit_id = parts[1] if len(parts) >= 2 else ""
                diff_bytes = diff_res.stdout or b""
                git_dirty = _git_dirty(repo)
                return {
                    "tool": "jj",
                    "workspace": str(repo),
                    "change_id": change_id,
                    "commit_id": commit_id,
                    "diff_sha256": hashlib.sha256(diff_bytes).hexdigest(),
                    "dirty": bool(diff_bytes.strip()) or bool(git_dirty),
                }
        except (OSError, subprocess.SubprocessError):
            pass

    if (repo / ".git").exists():
        try:
            head_res = subprocess.run(
                ["git", "--no-optional-locks", "rev-parse", "HEAD"],
                cwd=repo,
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            status_res = subprocess.run(
                ["git", "--no-optional-locks", "status", "--porcelain"],
                cwd=repo,
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            diff_res = subprocess.run(
                ["git", "--no-optional-locks", "diff", "HEAD"],
                cwd=repo,
                capture_output=True,
                timeout=10,
                check=False,
            )
            if head_res.returncode == 0 and status_res.returncode == 0:
                commit_id = head_res.stdout.strip()
                diff_bytes = diff_res.stdout or b""
                return {
                    "tool": "git",
                    "workspace": str(repo),
                    "change_id": None,
                    "commit_id": commit_id,
                    "diff_sha256": hashlib.sha256(diff_bytes).hexdigest(),
                    "dirty": bool(status_res.stdout.strip()),
                }
        except (OSError, subprocess.SubprocessError):
            pass

    return {
        "tool": None,
        "workspace": str(repo),
        "change_id": None,
        "commit_id": None,
        "diff_sha256": None,
        "dirty": None,
    }


def build_identity(
    iso_snapshot: Path,
    original_iso: Path,
    repo: Path,
    mb1_snapshot: Path | None = None,
    original_mb1_iso: Path | None = None,
) -> dict[str, Any]:
    """Extracts kernel build identity and provenance for an archived ISO."""
    iso_sha = sha256_file(iso_snapshot)
    build_id = f"b{iso_sha[:12]}"
    iso_bytes = iso_snapshot.read_bytes()
    stat = original_iso.stat() if original_iso.exists() else iso_snapshot.stat()

    grub_matches = sorted(
        {m.decode("ascii", errors="replace") for m in GRUB_VERSION_RE.findall(iso_bytes)}
    )
    grub_version = grub_matches[0] if grub_matches else None

    iso_dir = original_iso.resolve().parent
    candidate_dirs = [iso_dir, repo / "build"]
    kernel_bin_info: dict[str, Any] | None = None
    matched_dir: Path | None = None

    for d in candidate_dirs:
        kbin = d / "kernel.bin"
        if kbin.is_file():
            kbin_bytes = kbin.read_bytes()
            in_iso = bool(kbin_bytes and (kbin_bytes in iso_bytes))
            kernel_bin_info = {
                "path": str(kbin),
                "sha256": hashlib.sha256(kbin_bytes).hexdigest(),
                "in_iso": in_iso,
            }
            if in_iso:
                matched_dir = d
                break

    if matched_dir is None and (iso_dir / ".kernel-config").is_file():
        matched_dir = iso_dir

    kernel_config: str | None = None
    defines: list[str] = []
    compilers: list[str] = []

    if matched_dir is not None:
        cfg_file = matched_dir / ".kernel-config"
        if cfg_file.is_file():
            kernel_config = cfg_file.read_text(
                encoding="utf-8", errors="replace"
            ).strip()
            defines = re.findall(r"-D[A-Za-z0-9_]+(?:=[^\s|]+)?", kernel_config)
            fields = [f.strip() for f in kernel_config.split("|")]
            if len(fields) >= 3:
                obj_Set: set[str] = set()
                for obj_rel in fields[2].split():
                    obj_path = repo / obj_rel
                    if not obj_path.is_file():
                        obj_path = matched_dir / Path(obj_rel).name
                    if obj_path.is_file():
                        obj_bytes = obj_path.read_bytes()
                        for m_gcc in GCC_VERSION_RE.findall(obj_bytes):
                            obj_Set.add(m_gcc.decode("ascii", errors="replace"))
                compilers = sorted(obj_Set)

    if not defines:
        defines = ["-DPROTOS_SELFTEST=1"]

    # The Docker image (Dockerfile) pins Debian bookworm: GRUB 2.06 and
    # GCC 12 ("GCC: (Debian 12.2.0-...) 12.2.0"). Any other GRUB or GCC build
    # comes from a host toolchain.
    toolchain = "unknown"
    is_docker_grub = bool(grub_version and "2.06" in grub_version)
    is_host_grub = bool(grub_version and not is_docker_grub)
    is_docker_gcc = any("Debian 12." in c for c in compilers)
    is_host_gcc = any("Debian 12." not in c for c in compilers)

    if (is_host_grub or is_host_gcc) and not (is_docker_grub or is_docker_gcc):
        toolchain = "host"
    elif (is_docker_grub or is_docker_gcc) and not (is_host_grub or is_host_gcc):
        toolchain = "docker"
    elif (is_host_grub or is_host_gcc) and (is_docker_grub or is_docker_gcc):
        toolchain = "mixed"

    mb1_info: dict[str, Any] | None = None
    if mb1_snapshot is not None and mb1_snapshot.is_file():
        mb1_orig = original_mb1_iso or mb1_snapshot
        mb1_stat = mb1_orig.stat() if mb1_orig.exists() else mb1_snapshot.stat()
        mb1_info = {
            "original_path": str(mb1_orig),
            "snapshot": str(mb1_snapshot),
            "sha256": sha256_file(mb1_snapshot),
            "size": mb1_stat.st_size,
            "mtime_utc": datetime.fromtimestamp(
                mb1_stat.st_mtime, tz=timezone.utc
            ).isoformat(),
        }

    return {
        "build_id": build_id,
        "kernel_iso": {
            "original_path": str(original_iso),
            "snapshot": str(iso_snapshot),
            "sha256": iso_sha,
            "size": stat.st_size,
            "mtime_utc": datetime.fromtimestamp(
                stat.st_mtime, tz=timezone.utc
            ).isoformat(),
        },
        "kernel_mb1_iso": mb1_info,
        "kernel_bin": kernel_bin_info,
        "kernel_config": kernel_config,
        "defines": defines,
        "compilers": compilers,
        "grub": grub_version,
        "toolchain": toolchain,
        "vcs": vcs_provenance(repo),
    }


class Interrupted(KeyboardInterrupt):
    """A signal that stops the session, raised by `stop_signals_raise`."""

    def __init__(self, signum: int) -> None:
        super().__init__(signal.Signals(signum).name)
        self.signum = signum


STOP_SIGNALS: tuple[signal.Signals, ...] = (
    signal.SIGINT,
    signal.SIGTERM,
    signal.SIGHUP,
)


@contextlib.contextmanager
def stop_signals_raise() -> Iterator[None]:
    """Makes SIGINT, SIGTERM and SIGHUP raise `Interrupted` while active.

    By default, SIGTERM and SIGHUP (a closed SSH session, for example) end
    Python without running `finally` blocks, which would leave the VM or
    test_boot.sh of the current run running (in a session of its own, which a
    terminal's signals don't reach) after deflake released the host lock. As
    an exception, they stop the run like Ctrl-C: the runners stop its process
    group (and container), then the session writes its summary. Only the
    first signal raises, so that later ones can't cut that cleanup short.
    Signals that were ignored (nohup) stay ignored.
    """
    stopping = False

    def handler(signum: int, frame: Any) -> None:
        nonlocal stopping
        if not stopping:
            stopping = True
            raise Interrupted(signum)

    previous: dict[int, Any] = {}
    for signum in STOP_SIGNALS:
        old = signal.getsignal(signum)
        if old is signal.SIG_IGN:
            continue
        try:
            signal.signal(signum, handler)
        except ValueError:  # Not the main thread.
            continue
        previous[signum] = signal.SIG_DFL if old is None else old
    try:
        yield
    finally:
        for signum, old in previous.items():
            signal.signal(signum, old)


class HostLock:
    """Context manager that acquires an exclusive flock on a lock file."""

    def __init__(self, path: Path | None) -> None:
        self.path = path
        self.wait_s: float = 0.0
        self._fd: int | None = None

    def __enter__(self) -> HostLock:
        if self.path is None:
            self.wait_s = 0.0
            return self
        parent = self.path.parent
        if parent.is_symlink():
            raise OSError(f"Refusing symlinked lock directory: {parent}")
        parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        t0 = time.monotonic()
        flags = os.O_CREAT | os.O_RDWR | getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(str(self.path), flags, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd
        self.wait_s = round(time.monotonic() - t0, 4)
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        if self._fd is not None:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_UN)
            finally:
                os.close(self._fd)
                self._fd = None


def stop_process_group(
    proc: subprocess.Popen[Any],
    docker_name: str | None = None,
    docker_bin: str = "docker",
) -> None:
    """Terminates a process group (and optional Docker container) cleanly."""
    if proc.poll() is None:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except PermissionError:
            proc.terminate()

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and proc.poll() is None:
            time.sleep(0.05)

        if proc.poll() is None:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError:
                proc.kill()

    if docker_name:
        try:
            subprocess.run(
                [docker_bin, "kill", docker_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=10,
                check=False,
            )
        except OSError:
            pass

    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


@dataclasses.dataclass
class RunRecord:
    """Complete record of one executed boot run."""

    config: str
    index: int
    build: str
    status: str
    reasons: tuple[str, ...]
    outcomes: dict[str, str]
    failed_tests: list[str]
    phase: str
    seeds: list[str]
    seed_lines: list[tuple[int, str, str]]
    failure_lines: list[tuple[int, str]]
    expectations: list[tuple[str, str]]
    diagnostics: list[str]
    duration_s: float
    start_utc: str
    lock_wait_s: float
    loadavg_before: list[float]
    qemu_argv: tuple[str, ...]
    qemu_cwd: str
    harness_argv: tuple[str, ...]
    env_recorded: dict[str, str | None]
    nondefault_harness_env: dict[str, str]
    iso_snapshot: str
    mb1_iso_snapshot: str | None
    exit_code: int | None
    t_pass: float | None
    t_fail: float | None
    artifact_dir: str | None = None
    # The directory of test_boot.sh's run whose files (serial log, sockets)
    # qemu_argv names, relative to qemu_cwd; test_boot.sh deletes it at its end.
    qemu_run_dir: str | None = None


def _current_loadavg() -> list[float]:
    try:
        la = os.getloadavg()
        return [round(la[0], 2), round(la[1], 2), round(la[2], 2)]
    except OSError:
        return [0.0, 0.0, 0.0]


def _record_env(env: Mapping[str, str] | dict[str, str]) -> tuple[dict[str, str | None], dict[str, str]]:
    keys = (
        "PATH",
        "HOME",
        "USER",
        "LANG",
        "LC_ALL",
        "TMPDIR",
        "KVM",
        "USE_DOCKER",
        *BOOT_TEST_ENV_VARS,
    )
    recorded: dict[str, str | None] = {k: env.get(k) for k in keys}
    for k, v in env.items():
        if k.startswith("QEMU"):
            recorded[k] = v
    nondefault = {
        k: env[k] for k in BOOT_TEST_ENV_VARS if k in env and env[k] != ""
    }
    return recorded, nondefault


def run_qemu_boot(
    config: Config,
    run_dir: Path,
    iso_snapshot: Path,
    repo: Path,
    registry: Sequence[str],
    *,
    index: int,
    build_id: str,
    lock_wait_s: float = 0.0,
    grace_s: float = 1.0,
    observe: str = "complete",
    min_stress: int = 3,
) -> RunRecord:
    """Executes one direct QEMU boot and evaluates it with `judge_qemu_boot`."""
    run_dir.mkdir(parents=True, exist_ok=True)
    argv = tuple(
        str(iso_snapshot) if tok == ISO_PLACEHOLDER else tok
        for tok in config.qemu_argv
    )
    serial_path = run_dir / "serial.log"
    stderr_path = run_dir / "qemu_stderr.log"
    start_utc = datetime.now(timezone.utc).isoformat()
    loadavg = _current_loadavg()
    env_rec, nondefault_env = _record_env(os.environ)

    t0 = time.monotonic()
    try:
        with serial_path.open("wb") as out_f, stderr_path.open("wb") as err_f:
            proc = subprocess.Popen(
                argv,
                cwd=repo,
                stdin=subprocess.DEVNULL,
                stdout=out_f,
                stderr=err_f,
                start_new_session=True,
            )
    except OSError as e:
        stderr_path.write_text(f"Spawn error: {e}\n", encoding="utf-8")
        serial_path.write_bytes(b"")
        verdict = Verdict("ERROR", (f"spawn_failed:{e}",))
        outcomes = test_outcomes(SerialReport(), verdict, registry, "qemu")
        return RunRecord(
            config=config.name,
            index=index,
            build=build_id,
            status="ERROR",
            reasons=verdict.reasons,
            outcomes=outcomes,
            failed_tests=[],
            phase="early boot",
            seeds=[],
            seed_lines=[],
            failure_lines=[],
            expectations=[],
            diagnostics=[str(e)],
            duration_s=round(time.monotonic() - t0, 3),
            start_utc=start_utc,
            lock_wait_s=lock_wait_s,
            loadavg_before=loadavg,
            qemu_argv=argv,
            qemu_cwd=str(repo),
            harness_argv=(),
            env_recorded=env_rec,
            nondefault_harness_env=nondefault_env,
            iso_snapshot=str(iso_snapshot),
            mb1_iso_snapshot=None,
            exit_code=None,
            t_pass=None,
            t_fail=None,
        )

    parser = SerialParser(registry)
    offset = 0
    t_pass: float | None = None
    t_fail: float | None = None
    drain_done_at: float | None = None
    exited_rc: int | None = None
    stopped_by_tool = False
    timed_out = False

    def read_new_bytes() -> None:
        nonlocal offset
        try:
            with serial_path.open("rb") as rf:
                rf.seek(offset)
                chunk = rf.read()
                if chunk:
                    offset += len(chunk)
                    parser.feed(chunk)
        except OSError:
            pass

    try:
        while True:
            read_new_bytes()
            elapsed = time.monotonic() - t0
            rc = proc.poll()

            if t_fail is None and parser.has_failure():
                t_fail = round(elapsed, 4)

            if (
                t_pass is None
                and t_fail is None
                and parser.meets_pass_condition(config.expected_cpus, min_stress)
            ):
                t_pass = round(elapsed, 4)

            if rc is not None:
                exited_rc = rc
                break

            if elapsed >= config.timeout_s:
                timed_out = True
                stopped_by_tool = True
                break

            if observe != "timeout":
                if t_fail is not None:
                    if drain_done_at is None:
                        if parser.has_suite_end_or_panic():
                            extra = (
                                2.0
                                if parser.report().fatal_lines
                                else 1.0
                            )
                            drain_done_at = elapsed + extra
                        elif parser.report().greeting_count >= 2:
                            drain_done_at = elapsed + 1.0
                    elif elapsed >= drain_done_at:
                        stopped_by_tool = True
                        break
                elif t_pass is not None and elapsed >= t_pass + grace_s:
                    stopped_by_tool = True
                    break

            time.sleep(0.05)
    finally:
        stop_process_group(proc)
        read_new_bytes()

    report = parser.finish()
    if t_fail is None and report.failure_lines:
        t_fail = round(time.monotonic() - t0, 4)
    if (
        t_pass is None
        and t_fail is None
        and report.greeting_count == 1
        and (config.expected_cpus, config.expected_cpus) in report.online
        and report.suite_end == "passed"
        and report.stress_pass_count >= min_stress
        and not report.failure_lines
    ):
        t_pass = round(min(time.monotonic() - t0, config.timeout_s), 4)

    end = BootEnd(
        t_start=0.0,
        t_pass=t_pass,
        t_fail=t_fail,
        exited_rc=exited_rc,
        timed_out=timed_out,
        stopped_by_tool=stopped_by_tool,
    )
    verdict = judge_qemu_boot(
        report,
        end,
        config.expected_cpus,
        timeout_s=config.timeout_s,
        min_stress=min_stress,
        registry=registry,
    )
    outcomes = test_outcomes(report, verdict, registry, "qemu")
    failed_tests = [k for k, v in outcomes.items() if v == "FAIL"]
    duration_s = round(time.monotonic() - t0, 3)

    return RunRecord(
        config=config.name,
        index=index,
        build=build_id,
        status=verdict.status,
        reasons=verdict.reasons,
        outcomes=outcomes,
        failed_tests=failed_tests,
        phase=phase_of(report, registry),
        seeds=[s for _, s, _ in report.seeds],
        seed_lines=list(report.seeds),
        failure_lines=list(report.failure_lines),
        expectations=list(report.expectations),
        diagnostics=list(report.diagnostics),
        duration_s=duration_s,
        start_utc=start_utc,
        lock_wait_s=lock_wait_s,
        loadavg_before=loadavg,
        qemu_argv=argv,
        qemu_cwd=str(repo),
        harness_argv=(),
        env_recorded=env_rec,
        nondefault_harness_env=nondefault_env,
        iso_snapshot=str(iso_snapshot),
        mb1_iso_snapshot=None,
        exit_code=exited_rc if exited_rc is not None else proc.returncode,
        t_pass=t_pass,
        t_fail=t_fail,
    )


def docker_image_id(
    image: str = DEFAULT_DOCKER_IMAGE, docker_bin: str = "docker"
) -> str | None:
    """Returns the ID of the Docker image that `image` names, or None."""
    try:
        res = subprocess.run(
            [docker_bin, "image", "inspect", "--format", "{{.Id}}", image],
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    image_id = res.stdout.strip()
    return image_id if res.returncode == 0 and image_id else None


def docker_is_rootless(docker_bin: str = "docker") -> bool:
    """Returns whether the Docker engine is rootless Docker or Podman."""
    try:
        res = subprocess.run(
            [docker_bin, "info", "--format", "{{json .}}"],
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return re.search(r'"name=rootless"|"rootless": ?true', res.stdout) is not None


def selinux_enforcing() -> bool:
    """Returns whether SELinux is enforcing on this host."""
    try:
        return Path("/sys/fs/selinux/enforce").read_text().strip() == "1"
    except OSError:
        return False


def docker_run_identity(
    env: Mapping[str, str],
    *,
    docker_bin: str = "docker",
    rootless: Callable[[str], bool] = docker_is_rootless,
    selinux: Callable[[], bool] = selinux_enforcing,
) -> tuple[str, str]:
    """Returns the --user value and the bind mount options of docker runs.

    Like the Makefile's DOCKER_RUN_ARGS: the container runs as the host user,
    so that the files it writes in the bind-mounted checkout are the user's,
    but as root with rootless Docker or Podman, which map container root to
    the host user (and the other container users to subordinate IDs, which
    can't write the checkout); HOST_UID and HOST_GID override the choice. On
    SELinux hosts, ':z' gives the checkout the container label without which
    containers may not use it; DOCKER_VOLUME_OPTS overrides it.
    """
    uid = env.get("HOST_UID") or None
    gid = env.get("HOST_GID") or None
    if uid is None and gid is None and rootless(docker_bin):
        uid = gid = "0"
    if uid is None:
        uid = str(os.getuid())
    if gid is None:
        gid = str(os.getgid())
    volume_opts = env.get("DOCKER_VOLUME_OPTS")
    if volume_opts is None:
        volume_opts = ":z" if selinux() else ""
    return f"{uid}:{gid}", volume_opts


def _default_kvm_gid() -> int | None:
    try:
        return os.stat("/dev/kvm").st_gid
    except OSError:
        return None


def docker_kvm_args(
    env: Mapping[str, str],
    *,
    kvm_gid: Callable[[], int | None] = _default_kvm_gid,
) -> tuple[str, ...]:
    """Returns Docker arguments for KVM=1 mirroring Makefile's DOCKER_KVM_ARGS."""
    if env.get("KVM") != "1":
        return ()
    args: list[str] = ["--device", "/dev/kvm", "-e", "KVM=1"]
    gid = kvm_gid()
    if gid is not None:
        args.extend(["--group-add", str(gid)])
    return tuple(args)


def build_docker_test_boot_argv(
    repo: Path,
    pass_name: str,
    artifact_rel: str,
    iso_rel: str,
    mb1_rel: str,
    *,
    container_name: str | None = None,
    docker_bin: str = "docker",
    docker_image: str = DEFAULT_DOCKER_IMAGE,
    uid: int | None = None,
    gid: int | None = None,
    user: str | None = None,
    volume_opts: str = "",
    kvm_args: Sequence[str] = (),
) -> tuple[str, ...]:
    """Builds the exact docker run command mirroring Makefile's DOCKER_RUN.

    `user` (see `docker_run_identity`) replaces `uid`:`gid`, which default
    to the host user's.
    """
    host_uid = os.getuid() if uid is None else uid
    host_gid = os.getgid() if gid is None else gid
    cmd: list[str] = [docker_bin, "run", "--rm"]
    if container_name:
        cmd.extend(["--name", container_name])
    cmd.extend([
        "--platform=linux/amd64",
        "--user",
        user or f"{host_uid}:{host_gid}",
        "-e",
        "HOME=/tmp",
    ])
    for var in BOOT_TEST_ENV_VARS:
        cmd.extend(["-e", var])
    cmd.extend(kvm_args)
    cmd.extend([
        "-v",
        f"{repo}:/workspace{volume_opts}",
        "-w",
        "/workspace",
        docker_image,
        "./test_boot.sh",
        f"--passes={pass_name}",
        f"--artifact-dir={artifact_rel}",
        "--drain-on-fail",
        iso_rel,
        mb1_rel,
    ])
    return tuple(cmd)


def run_test_boot_pass(
    config: Config,
    run_dir: Path,
    iso_snapshot: Path,
    mb1_snapshot: Path | None,
    repo: Path,
    registry: Sequence[str],
    *,
    index: int,
    build_id: str,
    session_id: str,
    lock_wait_s: float = 0.0,
    test_boot_script: Path | None = None,
    docker_bin: str = "docker",
    docker_image: str = DEFAULT_DOCKER_IMAGE,
    docker_user: str | None = None,
    docker_volume_opts: str = "",
    docker_kvm_extra: Sequence[str] = (),
) -> RunRecord:
    """Executes one pass of test_boot.sh (host or docker) and reads its verdict."""
    run_dir.mkdir(parents=True, exist_ok=True)
    pass_name = config.pass_name
    start_utc = datetime.now(timezone.utc).isoformat()
    loadavg = _current_loadavg()

    env = dict(os.environ)
    if config.mode == "host":
        env["USE_DOCKER"] = "0"
    env_rec, nondefault_env = _record_env(env)

    script_rel = (
        str(test_boot_script) if test_boot_script is not None else "./test_boot.sh"
    )

    staging_dir: Path | None = None
    harness_dir = run_dir / "harness"

    if config.mode == "docker":
        repo_iso_store = repo / "build" / "deflake" / "isos"
        repo_iso, _ = archive_iso(iso_snapshot, repo_iso_store)
        iso_arg = str(repo_iso.relative_to(repo))
        if mb1_snapshot is not None and mb1_snapshot.is_file():
            repo_mb1, _ = archive_iso(mb1_snapshot, repo_iso_store)
            mb1_arg = str(repo_mb1.relative_to(repo))
        else:
            mb1_arg = "build/kernel-mb1.iso"

        try:
            artifact_rel = str(harness_dir.resolve().relative_to(repo.resolve()))
        except ValueError:
            staging_dir = (
                repo
                / "build"
                / "deflake"
                / ".staging"
                / f"{session_id}-{config.slug}-{index:04d}"
            )
            if staging_dir.exists():
                shutil.rmtree(staging_dir, ignore_errors=True)
            staging_dir.mkdir(parents=True, exist_ok=True)
            artifact_rel = str(staging_dir.relative_to(repo))

        container_name = f"deflake-{session_id}-{config.slug}-{index:04d}"
        harness_argv = build_docker_test_boot_argv(
            repo,
            pass_name,
            artifact_rel,
            iso_arg,
            mb1_arg,
            container_name=container_name,
            docker_bin=docker_bin,
            docker_image=docker_image,
            user=docker_user,
            volume_opts=docker_volume_opts,
            kvm_args=docker_kvm_extra,
        )
    else:
        container_name = None
        iso_arg = str(iso_snapshot)
        mb1_arg = (
            str(mb1_snapshot)
            if mb1_snapshot is not None
            else str(repo / "build" / "kernel-mb1.iso")
        )
        harness_argv = (
            script_rel,
            f"--passes={pass_name}",
            f"--artifact-dir={harness_dir}",
            "--drain-on-fail",
            iso_arg,
            mb1_arg,
        )

    test_boot_log_path = run_dir / "test_boot.log"
    watchdog_s = harness_watchdog_s(config, env)
    t0 = time.monotonic()
    watchdog_fired = False
    exit_code: int | None = None

    try:
        with test_boot_log_path.open("wb") as log_f:
            proc = subprocess.Popen(
                harness_argv,
                cwd=repo,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                exit_code = proc.wait(timeout=watchdog_s)
            except subprocess.TimeoutExpired:
                watchdog_fired = True
                stop_process_group(
                    proc, docker_name=container_name, docker_bin=docker_bin
                )
                exit_code = proc.returncode
            except BaseException:
                # Ctrl-C, SIGTERM or SIGHUP (see stop_signals_raise), which
                # don't reach test_boot.sh's session: stop it, its QEMU and
                # its container before the caller releases the host lock.
                stop_process_group(
                    proc, docker_name=container_name, docker_bin=docker_bin
                )
                raise
    except OSError as e:
        test_boot_log_path.write_text(f"Spawn error: {e}\n", encoding="utf-8")
        verdict = Verdict("ERROR", (f"spawn_failed:{e}",))
        outcomes = test_outcomes(
            SerialReport(), verdict, registry, "test_boot"
        )
        return RunRecord(
            config=config.name,
            index=index,
            build=build_id,
            status="ERROR",
            reasons=verdict.reasons,
            outcomes=outcomes,
            failed_tests=[],
            phase="early boot",
            seeds=[],
            seed_lines=[],
            failure_lines=[],
            expectations=[],
            diagnostics=[str(e)],
            duration_s=round(time.monotonic() - t0, 3),
            start_utc=start_utc,
            lock_wait_s=lock_wait_s,
            loadavg_before=loadavg,
            qemu_argv=(),
            qemu_cwd=str(repo),
            harness_argv=harness_argv,
            env_recorded=env_rec,
            nondefault_harness_env=nondefault_env,
            iso_snapshot=str(iso_snapshot),
            mb1_iso_snapshot=str(mb1_snapshot) if mb1_snapshot else None,
            exit_code=None,
            t_pass=None,
            t_fail=None,
        )

    if staging_dir is not None and staging_dir.exists():
        if harness_dir.exists():
            shutil.rmtree(harness_dir, ignore_errors=True)
        shutil.move(str(staging_dir), str(harness_dir))

    pass_art_dir = harness_dir / pass_name
    serial_src = pass_art_dir / "serial.log"
    serial_dst = run_dir / "serial.log"
    serial_bytes = b""
    if serial_src.is_file():
        serial_bytes = serial_src.read_bytes()
        shutil.copy2(serial_src, serial_dst)
    else:
        serial_dst.write_bytes(b"")

    output_log_text = ""
    output_src = pass_art_dir / "output.log"
    if output_src.is_file():
        output_log_text = output_src.read_text(
            encoding="latin-1", errors="replace"
        )
    elif test_boot_log_path.is_file():
        output_log_text = test_boot_log_path.read_text(
            encoding="latin-1", errors="replace"
        )

    result_text: str | None = None
    result_file = pass_art_dir / "result"
    if result_file.is_file():
        result_text = result_file.read_text(
            encoding="utf-8", errors="replace"
        ).strip()

    qemu_argv: tuple[str, ...] = ()
    qemu_argv_file = pass_art_dir / "qemu_argv"
    if qemu_argv_file.is_file():
        raw_argv = qemu_argv_file.read_bytes().split(b"\0")
        if raw_argv and raw_argv[-1] == b"":
            raw_argv.pop()
        qemu_argv = tuple(
            item.decode("utf-8", errors="surrogateescape") for item in raw_argv
        )

    qemu_cwd = str(repo)
    qemu_cwd_file = pass_art_dir / "qemu_cwd"
    if qemu_cwd_file.is_file():
        qemu_cwd = qemu_cwd_file.read_text(
            encoding="utf-8", errors="replace"
        ).strip()

    qemu_run_dir: str | None = None
    qemu_run_dir_file = pass_art_dir / "qemu_run_dir"
    if qemu_run_dir_file.is_file():
        qemu_run_dir = (
            qemu_run_dir_file.read_text(encoding="utf-8", errors="replace").strip()
            or None
        )

    test_boot_log_text = (
        test_boot_log_path.read_text(encoding="latin-1", errors="replace")
        if test_boot_log_path.is_file()
        else ""
    )
    summary_map = parse_test_boot_summary(test_boot_log_text)
    summary_status = summary_map.get(pass_name)

    report = parse_serial(serial_bytes, registry)
    verdict = judge_test_boot(
        result_text,
        exit_code,
        summary_status,
        output_log=output_log_text,
        report=report,
        watchdog_fired=watchdog_fired,
        registry=registry,
    )
    outcomes = test_outcomes(
        report, verdict, registry, "test_boot", output_log_text
    )
    failed_tests = [k for k, v in outcomes.items() if v == "FAIL"]
    duration_s = round(time.monotonic() - t0, 3)

    return RunRecord(
        config=config.name,
        index=index,
        build=build_id,
        status=verdict.status,
        reasons=verdict.reasons,
        outcomes=outcomes,
        failed_tests=failed_tests,
        phase=phase_of(report, registry),
        seeds=[s for _, s, _ in report.seeds],
        seed_lines=list(report.seeds),
        failure_lines=list(report.failure_lines),
        expectations=list(report.expectations),
        diagnostics=list(report.diagnostics),
        duration_s=duration_s,
        start_utc=start_utc,
        lock_wait_s=lock_wait_s,
        loadavg_before=loadavg,
        qemu_argv=qemu_argv,
        qemu_cwd=qemu_cwd,
        harness_argv=harness_argv,
        env_recorded=env_rec,
        nondefault_harness_env=nondefault_env,
        iso_snapshot=str(iso_snapshot),
        mb1_iso_snapshot=str(mb1_snapshot) if mb1_snapshot else None,
        exit_code=exit_code,
        t_pass=duration_s if verdict.status == "PASS" else None,
        t_fail=duration_s if verdict.status == "FAIL" else None,
        qemu_run_dir=qemu_run_dir,
    )


def without_interactive_session(argv: Sequence[str]) -> tuple[str, ...]:
    """Returns QEMU argv `argv` without INTERACTIVE_SESSION_ARGS."""
    n = len(INTERACTIVE_SESSION_ARGS)
    result: list[str] = []
    i = 0
    while i < len(argv):
        if tuple(argv[i : i + n]) == INTERACTIVE_SESSION_ARGS:
            i += n
            continue
        result.append(argv[i])
        i += 1
    return tuple(result)


def _docker_run_option(
    argv: Sequence[str], option: str, prefix: str = ""
) -> str | None:
    """Returns the value of the first `option` in `argv` starting with `prefix`."""
    for i in range(len(argv) - 1):
        if argv[i] == option and argv[i + 1].startswith(prefix):
            return argv[i + 1]
    return None


def render_rerun_script(
    record: RunRecord,
    config: Config,
    repo: Path,
    docker_image_id: str | None = None,
) -> str:
    """Renders an executable rerun.sh script that boots the exact archived ISO."""
    lines = [
        "#!/bin/sh",
        f"# Rerun script for {record.config} run #{record.index:04d} (build {record.build})",
        f"# Archived ISO: {record.iso_snapshot}",
    ]
    if config.engine == "qemu" or not record.harness_argv:
        argv = record.qemu_argv
        lines.append(f"cd {shlex.quote(record.qemu_cwd)} || exit 1")
        lines.append(f"exec {shlex.join(argv)}")
        return "\n".join(lines) + "\n"

    # Without test_boot.sh, which types 'exit 0' after its USB keyboard
    # scenarios, the kernel's interactive boot session would wait forever
    # before the stress test.
    qemu_argv = without_interactive_session(record.qemu_argv)
    notes: list[str] = []
    if qemu_argv != tuple(record.qemu_argv):
        notes = [
            "# This boot leaves out test_boot.sh's request for the kernel's",
            f"# interactive boot session ({shlex.join(INTERACTIVE_SESSION_ARGS)}),",
            "# which waits for test_boot.sh's USB keyboard scenarios (reproduce.sh",
            "# runs them) before the stress test. The exact QEMU command was:",
            f"# {shlex.join(record.qemu_argv)}",
        ]
    # The serial log and the QEMU sockets go in test_boot.sh's directory for
    # the run, which it deleted at its end.
    qemu_run_dir = record.qemu_run_dir or "build"
    if config.mode == "docker":
        repo_iso_store = repo / "build" / "deflake" / "isos"
        lines.append(f"mkdir -p {shlex.quote(str(repo_iso_store))}")
        for snap in (record.iso_snapshot, record.mb1_iso_snapshot):
            if snap:
                snap_path = Path(snap)
                if snap_path.name:
                    staged = repo_iso_store / snap_path.name
                    lines.append(
                        f"[ -f {shlex.quote(str(staged))} ] || cp -p"
                        f" {shlex.quote(str(snap_path))} {shlex.quote(str(staged))}"
                    )
        if qemu_argv:
            # The user and the bind mount options of the harness's container.
            docker_bin = (
                record.harness_argv[0]
                if record.harness_argv
                else (os.environ.get("DOCKER") or "docker")
            )
            user = _docker_run_option(record.harness_argv, "--user") or (
                f"{os.getuid()}:{os.getgid()}"
            )
            workspace = f"{repo}:/workspace"
            workspace_mount = (
                _docker_run_option(record.harness_argv, "-v", workspace) or workspace
            )
            volume_opts = workspace_mount[len(workspace) :].replace(":", ",", 1)
            extra_docker_opts: list[str] = []
            for i in range(len(record.harness_argv) - 1):
                if record.harness_argv[i] in ("--device", "--group-add"):
                    extra_docker_opts.extend(record.harness_argv[i : i + 2])
            image = (
                docker_image_id
                or os.environ.get("DOCKER_IMAGE")
                or DEFAULT_DOCKER_IMAGE
            )
            iso_store = str(Path(record.iso_snapshot).parent)
            docker_cmd = [
                docker_bin,
                "run",
                "--rm",
                "--platform=linux/amd64",
                "--user",
                user,
                "-e",
                "HOME=/tmp",
                *extra_docker_opts,
                "-v",
                workspace_mount,
                "-v",
                f"{iso_store}:{iso_store}:ro{volume_opts}",
                "-w",
                "/workspace",
                image,
                *qemu_argv,
            ]
            lines.append(f"mkdir -p {shlex.quote(str(repo / qemu_run_dir))}")
            lines.append(f"# Harness command: {shlex.join(record.harness_argv)}")
            lines.extend(notes)
            lines.append(f"exec {shlex.join(docker_cmd)}")
        else:
            lines.append(f"cd {shlex.quote(str(repo))} || exit 1")
            lines.append(f"exec {shlex.join(record.harness_argv)}")
    elif qemu_argv:
        lines.append(f"cd {shlex.quote(record.qemu_cwd)} || exit 1")
        lines.append(f"mkdir -p {shlex.quote(qemu_run_dir)}")
        lines.append(f"# Harness command: {shlex.join(record.harness_argv)}")
        lines.extend(notes)
        lines.append(f"exec {shlex.join(qemu_argv)}")
    else:
        lines.append(f"cd {shlex.quote(str(repo))} || exit 1")
        lines.append(f"exec {shlex.join(record.harness_argv)}")
    return "\n".join(lines) + "\n"


def render_reproduce_script(
    record: RunRecord, config: Config, repo: Path, tool_path: Path
) -> str:
    """Renders reproduce.sh to re-run 1 boot via deflake.py on the archived ISO."""
    cmd: list[str] = [str(tool_path.resolve()), "--runs", "1"]
    if config.name in BUILTIN_CONFIG_NAMES:
        cmd.extend(["--config", config.name])
    elif config.description.startswith("kvm24 + "):
        cmd.extend(["--qemu-args", config.description[len("kvm24 + ") :]])
        if config.timeout_s != 60.0:
            cmd.extend(["--timeout", f"custom1={config.timeout_s:g}"])
    elif config.description.startswith("cmd: "):
        # The command with the archived ISO as its -cdrom file, which may have
        # changed since the run (or be gone).
        argv = [
            record.iso_snapshot if tok == ISO_PLACEHOLDER else tok
            for tok in config.qemu_argv
        ]
        cmd.extend(["--qemu-cmd", shlex.join(argv)])
        if config.timeout_s != 60.0:
            cmd.extend(["--timeout", f"custom1={config.timeout_s:g}"])
    if config.custom_iso is None:
        cmd.extend(["--iso", record.iso_snapshot])
        if record.mb1_iso_snapshot:
            cmd.extend(["--mb1-iso", record.mb1_iso_snapshot])
    return (
        "#!/bin/sh\n"
        f"cd {shlex.quote(str(repo))} || exit 1\n"
        f"exec {shlex.join(cmd)} \"$@\"\n"
    )


def write_run_artifacts(
    run_dir: Path,
    record: RunRecord,
    config: Config,
    build_info: dict[str, Any],
    repo: Path,
    tool_path: Path,
    docker_image_id: str | None = None,
) -> None:
    """Writes command.json, verdict.json, seeds.txt, build.json, rerun.sh, reproduce.sh."""
    run_dir.mkdir(parents=True, exist_ok=True)
    cmd_payload: dict[str, Any] = {
        "config": record.config,
        "engine": config.engine,
        "argv": list(record.qemu_argv or record.harness_argv),
        "qemu_argv": list(record.qemu_argv),
        "qemu_cmdline": shlex.join(record.qemu_argv) if record.qemu_argv else "",
        "cwd": record.qemu_cwd,
        "harness_argv": list(record.harness_argv),
        "env": record.env_recorded,
        "nondefault_harness_env": record.nondefault_harness_env,
        "iso_snapshot": record.iso_snapshot,
        "mb1_iso_snapshot": record.mb1_iso_snapshot,
        "docker_image_id": docker_image_id if config.mode == "docker" else None,
    }
    (run_dir / "command.json").write_text(
        json.dumps(cmd_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if record.qemu_argv:
        (run_dir / "qemu_cmdline.txt").write_text(
            shlex.join(record.qemu_argv) + "\n", encoding="utf-8"
        )

    verdict_payload: dict[str, Any] = {
        "config": record.config,
        "index": record.index,
        "build": record.build,
        "status": record.status,
        "reasons": list(record.reasons),
        "failed_tests": list(record.failed_tests),
        "outcomes": record.outcomes,
        "phase": record.phase,
        "failure_lines": [
            {"line": ln, "text": txt} for ln, txt in record.failure_lines
        ],
        "expectations": [
            {"expr": expr, "location": loc} for expr, loc in record.expectations
        ],
        "t_pass": record.t_pass,
        "t_fail": record.t_fail,
        "duration_s": record.duration_s,
        "start_utc": record.start_utc,
        "lock_wait_s": record.lock_wait_s,
        "loadavg_before": record.loadavg_before,
        "exit_code": record.exit_code,
    }
    (run_dir / "verdict.json").write_text(
        json.dumps(verdict_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    seeds_lines = [
        f"{lineno}\t{seed}\t{line}"
        for lineno, seed, line in record.seed_lines
    ]
    (run_dir / "seeds.txt").write_text(
        ("\n".join(seeds_lines) + "\n") if seeds_lines else "",
        encoding="utf-8",
    )

    (run_dir / "build.json").write_text(
        json.dumps(build_info, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    rerun_path = run_dir / "rerun.sh"
    rerun_path.write_text(
        render_rerun_script(record, config, repo, docker_image_id),
        encoding="utf-8",
    )
    os.chmod(rerun_path, 0o755)

    repro_path = run_dir / "reproduce.sh"
    repro_path.write_text(
        render_reproduce_script(record, config, repo, tool_path),
        encoding="utf-8",
    )
    os.chmod(repro_path, 0o755)


def aggregate(
    records: Sequence[RunRecord],
    configs: Sequence[Config],
    builds: dict[str, dict[str, Any]],
    registry: Sequence[str],
    session_meta: dict[str, Any],
) -> dict[str, Any]:
    """Aggregates run records into the protos-deflake/1 summary structure."""
    configs_out: dict[str, Any] = {}
    flaky_list: list[dict[str, Any]] = []
    broken_list: list[dict[str, Any]] = []

    by_config: dict[str, list[RunRecord]] = {c.name: [] for c in configs}
    for r in records:
        by_config.setdefault(r.config, []).append(r)

    for cfg in configs:
        cfg_records = by_config.get(cfg.name, [])
        by_build: dict[str, list[RunRecord]] = {}
        for r in cfg_records:
            by_build.setdefault(r.build, []).append(r)
        if not by_build and builds:
            first_build = next(iter(builds))
            by_build[first_build] = []

        pseudo_names = (
            ("random_multicpu_stress", "@panic", "@boot", "@harness")
            if cfg.engine == "test_boot"
            else ("random_multicpu_stress", "@panic", "@boot")
        )
        all_test_names = list(registry) + list(pseudo_names)

        groups_out: list[dict[str, Any]] = []
        for b_id, b_records in by_build.items():
            runs_cnt = len(b_records)
            passed_cnt = sum(1 for r in b_records if r.status == "PASS")
            failed_cnt = sum(1 for r in b_records if r.status == "FAIL")
            errors_cnt = sum(1 for r in b_records if r.status == "ERROR")
            skipped_cnt = sum(1 for r in b_records if r.status == "SKIPPED")
            judged_cnt = passed_cnt + failed_cnt
            fail_rate = (
                round(failed_cnt / judged_cnt, 4) if judged_cnt > 0 else 0.0
            )
            ci = wilson(failed_cnt, judged_cnt)

            tests_out: dict[str, Any] = {}
            extra_names = sorted({
                k
                for r in b_records
                for k in r.outcomes
                if k not in all_test_names
            })
            for tname in (*all_test_names, *extra_names):
                t_pass = sum(
                    1
                    for r in b_records
                    if r.outcomes.get(tname) == "PASS"
                )
                t_fail = sum(
                    1
                    for r in b_records
                    if r.outcomes.get(tname) == "FAIL"
                )
                t_no = sum(
                    1
                    for r in b_records
                    if r.outcomes.get(tname, "NO_RESULT") == "NO_RESULT"
                )
                t_cls = classify(t_pass, t_fail)
                t_ci = wilson(t_fail, t_pass + t_fail)
                t_entry: dict[str, Any] = {
                    "passed": t_pass,
                    "failed": t_fail,
                    "no_result": t_no,
                    "class": t_cls,
                    "fail_rate_ci95": list(t_ci) if t_ci is not None else None,
                }
                tests_out[tname] = t_entry

                item = {
                    "config": cfg.name,
                    "build": b_id,
                    "test": tname,
                    "passed": t_pass,
                    "failed": t_fail,
                    "no_result": t_no,
                    "fail_rate": (
                        round(t_fail / (t_pass + t_fail), 4)
                        if (t_pass + t_fail) > 0
                        else 0.0
                    ),
                    "fail_rate_ci95": list(t_ci) if t_ci is not None else None,
                }
                if t_cls == "flaky":
                    flaky_list.append(item)
                elif t_cls == "broken":
                    broken_list.append(item)

            groups_out.append({
                "build": b_id,
                "runs": runs_cnt,
                "passed": passed_cnt,
                "failed": failed_cnt,
                "errors": errors_cnt,
                "skipped": skipped_cnt,
                "fail_rate": fail_rate,
                "fail_rate_ci95": list(ci) if ci is not None else None,
                "tests": tests_out,
            })

        configs_out[cfg.name] = {
            "engine": cfg.engine,
            "description": cfg.description,
            "argv": list(cfg.qemu_argv),
            "timeout_s": cfg.timeout_s,
            "groups": groups_out,
        }

    runs_out = [
        {
            "config": r.config,
            "index": r.index,
            "build": r.build,
            "status": r.status,
            "reasons": list(r.reasons),
            "failed_tests": list(r.failed_tests),
            "phase": r.phase,
            "seeds": list(r.seeds),
            "seed_lines": [txt for _, _, txt in r.seed_lines],
            "duration_s": r.duration_s,
            "start_utc": r.start_utc,
            "lock_wait_s": r.lock_wait_s,
            "loadavg_before": list(r.loadavg_before),
            "artifact_dir": r.artifact_dir,
        }
        for r in records
    ]

    return {
        "schema": SCHEMA,
        "session": session_meta,
        "builds": builds,
        "configs": configs_out,
        "flaky": flaky_list,
        "broken": broken_list,
        "runs": runs_out,
    }


def render_text(summary: dict[str, Any], registry: Sequence[str]) -> str:
    """Renders the human-readable summary report."""
    sess = summary["session"]
    lines: list[str] = []
    lines.append(
        f"protos deflake — session {sess.get('id', '')} — "
        f"{len(summary['runs'])} runs in {sess.get('elapsed_s', 0.0):.1f}s "
        f"(stopped: {sess.get('stop_reason', 'runs')}) — "
        f"lock wait {sess.get('total_lock_wait_s', 0.0):.1f}s"
    )
    for b_id, b_info in summary.get("builds", {}).items():
        iso_sha = b_info.get("kernel_iso", {}).get("sha256", "")[:12]
        grub = b_info.get("grub") or "unknown-grub"
        tc = b_info.get("toolchain", "unknown")
        defs = " ".join(b_info.get("defines", []))
        vcs = b_info.get("vcs") or {}
        if vcs.get("tool") == "jj":
            vcs_str = (
                f"jj {vcs.get('change_id', '')[:8]} {vcs.get('commit_id', '')[:8]}"
            )
        elif vcs.get("tool") == "git":
            dirty_suffix = " (dirty)" if vcs.get("dirty") else ""
            vcs_str = f"git {(vcs.get('commit_id') or '')[:8]}{dirty_suffix}"
        else:
            vcs_str = "no-vcs"
        lines.append(
            f"build: {b_id} sha256={iso_sha}... ({grub}, toolchain={tc}, {defs}) {vcs_str}"
        )

    lines.append("")
    lines.append("Runs per configuration")
    lines.append(
        f"  {'config':<16} {'build':<14} {'runs':>5} {'pass':>5} {'fail':>5} "
        f"{'error':>5} {'skip':>5} {'fail rate':>10}  95% CI (Wilson)"
    )
    cfg_names = list(summary["configs"].keys())
    for cname in cfg_names:
        cinfo = summary["configs"][cname]
        for g in cinfo["groups"]:
            ci = g.get("fail_rate_ci95")
            ci_str = (
                f"{ci[0] * 100:5.1f}% – {ci[1] * 100:5.1f}%"
                if ci is not None
                else "n/a"
            )
            fr_str = f"{g['fail_rate'] * 100:5.1f}%"
            lines.append(
                f"  {cname:<16} {g['build']:<14} {g['runs']:>5} {g['passed']:>5} "
                f"{g['failed']:>5} {g['errors']:>5} {g['skipped']:>5} {fr_str:>10}  {ci_str}"
            )

    lines.append("")
    lines.append("Boot self-tests: pass/fail (+no result) per configuration")
    header_cols = "  ".join(f"{c:>12}" for c in cfg_names)
    lines.append(f"  {'test':<34} {header_cols}")

    has_test_boot = any(
        summary["configs"][c]["engine"] == "test_boot" for c in cfg_names
    )
    pseudo = (
        ("random_multicpu_stress", "@panic", "@boot", "@harness")
        if has_test_boot
        else ("random_multicpu_stress", "@panic", "@boot")
    )
    flaky_pairs = {(f["config"], f["test"]) for f in summary["flaky"]}
    broken_pairs = {(b["config"], b["test"]) for b in summary["broken"]}

    for tname in (*registry, *pseudo):
        cells: list[str] = []
        row_tags: list[str] = []
        for cname in cfg_names:
            groups = summary["configs"][cname]["groups"]
            p_sum = sum(
                g["tests"].get(tname, {}).get("passed", 0) for g in groups
            )
            f_sum = sum(
                g["tests"].get(tname, {}).get("failed", 0) for g in groups
            )
            n_sum = sum(
                g["tests"].get(tname, {}).get("no_result", 0) for g in groups
            )
            cell = f"{p_sum}/{f_sum}" + (f"+{n_sum}" if n_sum else "")
            cells.append(f"{cell:>12}")
            if (cname, tname) in flaky_pairs and "FLAKY" not in row_tags:
                row_tags.append("FLAKY")
            if (cname, tname) in broken_pairs and "BROKEN" not in row_tags:
                row_tags.append("BROKEN")
        tag_str = ("  " + " ".join(row_tags)) if row_tags else ""
        lines.append(f"  {tname:<34} {'  '.join(cells)}{tag_str}")

    lines.append("")
    lines.append(
        "Flaky tests (>=1 failure and >=1 pass in the same configuration and build)"
    )
    if summary["flaky"]:
        for f in summary["flaky"]:
            ci = f.get("fail_rate_ci95")
            ci_str = (
                f" (95% CI {ci[0] * 100:.1f}%–{ci[1] * 100:.1f}%)"
                if ci
                else ""
            )
            tot = f["passed"] + f["failed"]
            lines.append(
                f"  {f['config']}  {f['build']}  {f['test']}  "
                f"{f['failed']}/{tot} failed  {f['fail_rate'] * 100:.1f}%{ci_str}"
            )
    else:
        lines.append("  none")

    lines.append("")
    if summary["broken"]:
        lines.append("Broken tests (failed in every run with a result)")
        for b in summary["broken"]:
            lines.append(
                f"  {b['config']}  {b['build']}  {b['test']}  "
                f"{b['failed']}/{b['failed']} failed"
            )
    else:
        lines.append("Broken tests (failed in every run with a result): none")

    failed_runs = [
        r for r in summary["runs"] if r["status"] in ("FAIL", "ERROR")
    ]
    if failed_runs:
        lines.append("")
        lines.append("Failed runs")
        for r in failed_runs:
            ft = ",".join(r["failed_tests"]) or ",".join(r["reasons"]) or r["status"]
            seeds_str = ",".join(r["seeds"][:4]) if r["seeds"] else "—"
            lines.append(
                f"  {r['config']} #{r['index']:04d}  {ft}  "
                f"seeds: {seeds_str}  {r.get('artifact_dir') or ''}"
            )

    return "\n".join(lines) + "\n"


def build_arg_parser() -> argparse.ArgumentParser:
    """Constructs the CLI argument parser."""
    p = argparse.ArgumentParser(
        prog="deflake.py",
        description=(
            "Runs protos QEMU integration tests repeatedly across built-in or "
            "custom configurations, measures per-configuration and "
            "per-boot-self-test pass/fail rates, identifies flaky tests, and "
            "saves full diagnostic artifacts and rerun scripts for every "
            "failed boot."
        ),
    )
    p.add_argument(
        "--runs",
        type=int,
        default=None,
        help="Number of runs per selected configuration (default: 1 if --duration is not set).",
    )
    p.add_argument(
        "--duration",
        type=str,
        default=None,
        help="Wall-clock session time budget (e.g. '30s', '5m', '1h30m').",
    )
    p.add_argument(
        "--config",
        action="append",
        default=[],
        help=(
            "Configuration(s) to run: kvm24, docker:{bios,uefi,mb1,max,smep}, "
            "host:{bios,uefi,mb1,max,smep}, or aliases all/docker/host. "
            "Comma-separated or repeatable."
        ),
    )
    p.add_argument(
        "--qemu-args",
        "--custom",
        dest="qemu_args",
        action="append",
        default=[],
        help="Custom QEMU option overrides on top of kvm24 (e.g. '-smp 12 -m 512M'). Repeatable.",
    )
    p.add_argument(
        "--qemu-cmd",
        action="append",
        default=[],
        help="Full custom QEMU command line (must contain -cdrom PATH and stdio serial).",
    )
    p.add_argument(
        "--iso",
        type=str,
        default=None,
        help="Path to Multiboot2 kernel ISO (default: build/kernel.iso).",
    )
    p.add_argument(
        "--mb1-iso",
        type=str,
        default=None,
        help="Path to Multiboot1 kernel ISO (default: <iso_dir>/kernel-mb1.iso or build/kernel-mb1.iso).",
    )
    p.add_argument(
        "--timeout",
        action="append",
        default=[],
        help="Per-boot timeout for custom configs ONLY (rejected for built-in configs).",
    )
    p.add_argument(
        "--observe",
        choices=("complete", "timeout"),
        default="complete",
        help="For qemu engine: stop once verdict + grace completes ('complete') or run to full timeout ('timeout').",
    )
    p.add_argument(
        "--grace",
        type=float,
        default=1.0,
        help="Grace period in seconds after pass markers to watch for late failures (default: 1.0).",
    )
    p.add_argument(
        "--lock",
        type=str,
        default=str(default_lock_path()),
        help=(
            "Per-boot flock file path (default: $XDG_RUNTIME_DIR/protos_deflake/host.lock"
            " or /tmp/protos_deflake-$UID/host.lock; '' or 'none' to disable)."
        ),
    )
    p.add_argument(
        "--no-lock",
        action="store_true",
        help="Disable per-boot flock acquisition.",
    )
    p.add_argument(
        "--out",
        type=str,
        default=None,
        help="Session output directory (default: build/deflake/sessions/<UTC>).",
    )
    p.add_argument(
        "--iso-store",
        type=str,
        default=None,
        help="Directory for sha256-archived ISOs (default: <out>/isos).",
    )
    p.add_argument(
        "--keep-passed",
        action="store_true",
        help="Keep per-run artifact directories for passing runs as well as failing runs.",
    )
    p.add_argument(
        "--max-failures",
        type=int,
        default=None,
        help="Stop the session after K failed runs.",
    )
    p.add_argument(
        "--order",
        choices=("round-robin", "grouped"),
        default="round-robin",
        help="Execution order across configurations (default: round-robin).",
    )
    p.add_argument(
        "--list-configs",
        action="store_true",
        help="Print selected configurations and exit 0.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands that would run without booting any VM and exit 0.",
    )
    p.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Suppress per-run progress lines (final summary is still printed).",
    )
    return p


def run_session(
    args: argparse.Namespace,
    *,
    repo: Path | None = None,
    raw_argv: Sequence[str] = (),
) -> int:
    """Executes a complete deflaking session and returns its exit code."""
    repo_root = (repo or Path(__file__).resolve().parent).resolve()
    tool_path = Path(__file__).resolve()

    if args.runs is not None and args.runs <= 0:
        raise ConfigError(f"--runs must be a positive integer, got {args.runs}")
    if args.max_failures is not None and args.max_failures <= 0:
        raise ConfigError(
            f"--max-failures must be a positive integer, got {args.max_failures}"
        )

    duration_s = (
        parse_duration(args.duration) if args.duration is not None else None
    )
    runs_per_cfg = (
        args.runs if args.runs is not None else (None if duration_s else 1)
    )

    configs = expand_configs(
        config_specs=args.config,
        qemu_args_list=args.qemu_args,
        qemu_cmd_list=args.qemu_cmd,
        timeout_specs=args.timeout,
    )

    if args.list_configs:
        for c in configs:
            print(f"{c.name:<16} engine={c.engine:<10} timeout={c.timeout_s:g}s  {c.description}")
        return 0

    uses_docker = any(c.engine == "test_boot" and c.mode == "docker" for c in configs)
    docker_bin = os.environ.get("DOCKER") or "docker"
    configured_docker_image = (
        os.environ.get("DOCKER_IMAGE") or DEFAULT_DOCKER_IMAGE
    )
    docker_user: str | None = None
    docker_volume_opts = ""
    kvm_docker_args: tuple[str, ...] = ()
    if uses_docker:
        docker_user, docker_volume_opts = docker_run_identity(
            os.environ, docker_bin=docker_bin
        )
        kvm_docker_args = docker_kvm_args(os.environ)

    if args.dry_run:
        iso_preview = args.iso or str(repo_root / "build" / "kernel.iso")
        mb1_preview = args.mb1_iso or str(repo_root / "build" / "kernel-mb1.iso")
        for c in configs:
            if c.engine == "qemu":
                argv = [
                    iso_preview if t == ISO_PLACEHOLDER else t
                    for t in c.qemu_argv
                ]
                print(f"[{c.name}] {shlex.join(argv)}")
            elif c.mode == "docker":
                d_argv = build_docker_test_boot_argv(
                    repo_root,
                    c.pass_name,
                    f"build/deflake/dry/{c.pass_name}",
                    iso_preview,
                    mb1_preview,
                    docker_bin=docker_bin,
                    docker_image=configured_docker_image,
                    user=docker_user,
                    volume_opts=docker_volume_opts,
                    kvm_args=kvm_docker_args,
                )
                print(f"[{c.name}] {shlex.join(d_argv)}")
            else:
                h_argv = (
                    "./test_boot.sh",
                    f"--passes={c.pass_name}",
                    f"--artifact-dir=build/deflake/dry/{c.pass_name}",
                    "--drain-on-fail",
                    iso_preview,
                    mb1_preview,
                )
                print(f"[{c.name}] USE_DOCKER=0 {shlex.join(h_argv)}")
        return 0

    registry = load_boot_test_names(repo_root)

    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = (
        Path(args.out).resolve()
        if args.out
        else (repo_root / "build" / "deflake" / "sessions" / session_id).resolve()
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    iso_store = (
        Path(args.iso_store).resolve()
        if args.iso_store
        else (out_dir / "isos").resolve()
    )
    iso_store.mkdir(parents=True, exist_ok=True)

    orig_iso = (
        Path(args.iso).resolve()
        if args.iso
        else (repo_root / "build" / "kernel.iso").resolve()
    )
    if args.mb1_iso:
        orig_mb1 = Path(args.mb1_iso).resolve()
    elif args.iso and (orig_iso.parent / "kernel-mb1.iso").is_file():
        orig_mb1 = (orig_iso.parent / "kernel-mb1.iso").resolve()
    else:
        orig_mb1 = (repo_root / "build" / "kernel-mb1.iso").resolve()

    needs_default_iso = any(c.custom_iso is None for c in configs)
    needs_mb1 = any(c.engine == "test_boot" and c.pass_name == "mb1" for c in configs)

    builds: dict[str, dict[str, Any]] = {}
    default_iso_snap: Path | None = None
    default_mb1_snap: Path | None = None
    default_build_id = ""

    if needs_default_iso:
        default_iso_snap, _ = archive_iso(orig_iso, iso_store)
        if orig_mb1.is_file():
            default_mb1_snap, _ = archive_iso(orig_mb1, iso_store)
        elif needs_mb1:
            raise ConfigError(
                f"Multiboot1 ISO '{orig_mb1}' not found (required by mb1 pass)"
            )
        b_info = build_identity(
            default_iso_snap,
            orig_iso,
            repo_root,
            mb1_snapshot=default_mb1_snap,
            original_mb1_iso=orig_mb1 if orig_mb1.is_file() else None,
        )
        default_build_id = b_info["build_id"]
        builds[default_build_id] = b_info

    custom_builds: dict[str, tuple[Path, str]] = {}
    for c in configs:
        if c.custom_iso is not None:
            c_orig = c.custom_iso.resolve()
            c_snap, _ = archive_iso(c_orig, iso_store)
            b_info = build_identity(c_snap, c_orig, repo_root)
            b_id = b_info["build_id"]
            builds[b_id] = b_info
            custom_builds[c.name] = (c_snap, b_id)

    builds_dir = out_dir / "builds"
    builds_dir.mkdir(parents=True, exist_ok=True)
    for b_id, b_info in builds.items():
        (builds_dir / f"{b_id}.json").write_text(
            json.dumps(b_info, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    lock_path: Path | None = None
    if not args.no_lock and args.lock and args.lock.lower() != "none":
        lock_path = Path(os.path.abspath(args.lock))

    # The docker:<pass> runs use (and record) the image that the tag names now,
    # also if the tag moves on during the session (when make rebuilds the
    # image from another checkout's Dockerfile, for example).
    image_id = (
        docker_image_id(configured_docker_image, docker_bin=docker_bin)
        if uses_docker
        else None
    )

    records: list[RunRecord] = []
    per_cfg_count: dict[str, int] = {c.name: 0 for c in configs}
    failed_runs_total = 0
    total_lock_wait = 0.0
    stop_reason = "runs"
    interrupted = False
    interrupt_signum = int(signal.SIGINT)

    start_utc = datetime.now(timezone.utc).isoformat()
    t_session_start = time.monotonic()
    runs_jsonl_path = out_dir / "runs.jsonl"

    def execute_one(cfg: Config) -> bool:
        nonlocal failed_runs_total, total_lock_wait, stop_reason, interrupted
        nonlocal interrupt_signum
        if duration_s is not None and (time.monotonic() - t_session_start) >= duration_s:
            stop_reason = "duration"
            return False
        if (
            args.max_failures is not None
            and failed_runs_total >= args.max_failures
        ):
            stop_reason = "max_failures"
            return False

        per_cfg_count[cfg.name] += 1
        idx = per_cfg_count[cfg.name]
        run_dir = out_dir / "runs" / cfg.slug / f"{idx:04d}"

        if cfg.name in custom_builds:
            iso_snap, b_id = custom_builds[cfg.name]
        else:
            assert default_iso_snap is not None
            iso_snap, b_id = default_iso_snap, default_build_id

        try:
            with HostLock(lock_path) as lk:
                total_lock_wait += lk.wait_s
                if cfg.engine == "qemu":
                    rec = run_qemu_boot(
                        cfg,
                        run_dir,
                        iso_snap,
                        repo_root,
                        registry,
                        index=idx,
                        build_id=b_id,
                        lock_wait_s=lk.wait_s,
                        grace_s=args.grace,
                        observe=args.observe,
                    )
                else:
                    rec = run_test_boot_pass(
                        cfg,
                        run_dir,
                        iso_snap,
                        default_mb1_snap,
                        repo_root,
                        registry,
                        index=idx,
                        build_id=b_id,
                        session_id=session_id,
                        lock_wait_s=lk.wait_s,
                        docker_bin=docker_bin,
                        docker_image=image_id or configured_docker_image,
                        docker_user=docker_user,
                        docker_volume_opts=docker_volume_opts,
                        docker_kvm_extra=kvm_docker_args,
                    )
        except KeyboardInterrupt as e:
            interrupted = True
            interrupt_signum = getattr(e, "signum", signal.SIGINT)
            stop_reason = "interrupted"
            if run_dir.exists():
                shutil.rmtree(run_dir, ignore_errors=True)
            return False

        keep = (
            args.keep_passed
            or rec.status in ("FAIL", "ERROR")
        )
        if keep:
            rel_art = f"runs/{cfg.slug}/{idx:04d}"
            rec.artifact_dir = rel_art
            write_run_artifacts(
                run_dir,
                rec,
                cfg,
                builds[b_id],
                repo_root,
                tool_path,
                docker_image_id=image_id,
            )
        else:
            shutil.rmtree(run_dir, ignore_errors=True)

        records.append(rec)
        if rec.status in ("FAIL", "ERROR"):
            failed_runs_total += 1

        with runs_jsonl_path.open("a", encoding="utf-8") as jf:
            jf.write(
                json.dumps({
                    "config": rec.config,
                    "index": rec.index,
                    "build": rec.build,
                    "status": rec.status,
                    "reasons": list(rec.reasons),
                    "failed_tests": list(rec.failed_tests),
                    "seeds": list(rec.seeds),
                    "duration_s": rec.duration_s,
                    "artifact_dir": rec.artifact_dir,
                })
                + "\n"
            )

        if not args.quiet:
            diag = rec.diagnostics[0][:140] if rec.diagnostics else ""
            ft_str = ",".join(rec.failed_tests)
            extra = f" {ft_str}" if ft_str else ""
            if diag and rec.status != "PASS":
                extra += f" — {diag}"
            if rec.artifact_dir:
                extra += f" -> {out_dir / rec.artifact_dir}"
            print(
                f"[{len(records):4d}] {rec.config} #{rec.index:04d} "
                f"{rec.status:<5} {rec.duration_s:5.1f}s{extra}",
                flush=True,
            )
        return True

    try:
        if args.order == "grouped" and runs_per_cfg is not None:
            for cfg in configs:
                for _ in range(runs_per_cfg):
                    if not execute_one(cfg):
                        break
                if interrupted or stop_reason in ("duration", "max_failures"):
                    break
        else:
            round_idx = 0
            while True:
                if runs_per_cfg is not None and round_idx >= runs_per_cfg:
                    stop_reason = "runs"
                    break
                round_idx += 1
                kept_going = True
                for cfg in configs:
                    if not execute_one(cfg):
                        kept_going = False
                        break
                if not kept_going:
                    break
    except KeyboardInterrupt as e:
        # Stopped between runs (execute_one handles a stop during a run).
        interrupted = True
        interrupt_signum = getattr(e, "signum", signal.SIGINT)
        stop_reason = "interrupted"

    elapsed_s = round(time.monotonic() - t_session_start, 3)
    session_meta = {
        "id": session_id,
        "argv": list(raw_argv),
        "start_utc": start_utc,
        "end_utc": datetime.now(timezone.utc).isoformat(),
        "elapsed_s": elapsed_s,
        "total_lock_wait_s": round(total_lock_wait, 3),
        "stop_reason": stop_reason,
        "out_dir": str(out_dir),
        "tool_sha256": sha256_file(tool_path) if tool_path.is_file() else "",
    }
    (out_dir / "session.json").write_text(
        json.dumps(session_meta, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    summary = aggregate(records, configs, builds, registry, session_meta)
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    summary_text = render_text(summary, registry)
    (out_dir / "summary.txt").write_text(summary_text, encoding="utf-8")
    try:
        print(summary_text, end="", flush=True)
    except OSError:  # For example, after SIGHUP from a closed terminal.
        pass

    if interrupted:
        return 128 + interrupt_signum
    if any(r.status == "FAIL" for r in records):
        return 1
    if any(r.status == "ERROR" for r in records):
        return 3
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point."""
    raw = list(sys.argv[1:] if argv is None else argv)
    trailing_custom: list[str] = []
    if "--" in raw:
        sep = raw.index("--")
        trailing_tokens = raw[sep + 1 :]
        raw = raw[:sep]
        if trailing_tokens:
            trailing_custom.append(shlex.join(trailing_tokens))

    parser = build_arg_parser()
    with stop_signals_raise():
        try:
            args = parser.parse_args(raw)
            for tc in trailing_custom:
                if tc.startswith("qemu-system-"):
                    args.qemu_cmd.append(tc)
                else:
                    args.qemu_args.append(tc)
            return run_session(args, raw_argv=["deflake.py", *raw])
        except ConfigError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 2
        except KeyboardInterrupt as e:
            # Stopped before the runs, or while the summary was written.
            return 128 + getattr(e, "signum", signal.SIGINT)


if __name__ == "__main__":
    sys.exit(main())
