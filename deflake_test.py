#!/usr/bin/env python3
"""Unit tests for deflake.py and test_boot.sh CLI hooks."""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import random
import shlex
import shutil
import stat
import subprocess
import tempfile
import textwrap
import unittest

import deflake

REPO_ROOT = Path(__file__).resolve().parent


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


class RegistryTest(unittest.TestCase):

  def test_load_real_boot_test_names(self) -> None:
    names = deflake.load_boot_test_names(REPO_ROOT)
    self.assertEqual(len(names), 32)
    self.assertEqual(names[0], "pmm_memory_map_init")
    self.assertEqual(names[-1], "user_mode_mixed_workload")
    self.assertIn("rbtree_edge_cases", names)
    self.assertIn("user_mode_kill", names)
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

      fakebin = tmp_path / "fakebin"
      fakebin.mkdir()
      fake_qemu = fakebin / "qemu-system-x86_64"
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
      env.pop("USE_DOCKER", None)
      env.pop("KVM", None)

      # 1. Invalid --passes values fail closed before running QEMU.
      for bad_arg in ("--passes=bogus", "--passes=,", "--passes=*", "--unknown-opt"):
        proc = subprocess.run(
            [str(script_copy), bad_arg, "kernel.iso", "kernel-mb1.iso"],
            cwd=tmp_path,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 1)
        self.assertIn("FAIL:", proc.stderr + proc.stdout)

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
        self.assertEqual(argv_items[0], "qemu-system-x86_64")
        self.assertIn("-qmp", argv_items)
        if p == "max":
          self.assertIn("max", argv_items)
        serial_txt = (art_dir / p / "serial.log").read_text(encoding="utf-8")
        self.assertIn("[TEST] multiboot_mmap_edge_cases: PASS", serial_txt)
        self.assertIn("[TEST] BOOT SELF-TESTS FAILED", serial_txt)

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


if __name__ == "__main__":
  unittest.main()
