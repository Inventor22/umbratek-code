"""Scenario tests for scripts/arm.py against tools/utra_sim.py. Usage: python tests/cli_tests.py [python-exe-for-cli]"""
import os
import subprocess
import sys
import time

from simrig import ARM_CLI as ARM, HOST, ROOT, start_sim, stop

CLI_PY = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("UTRA_TEST_PY") or sys.executable
LOCAL = HOST
JOG = ["jog", LOCAL, "-j", "6", "-d", "5", "--yes"]
SLOW_JOG = ["jog", LOCAL, "-j", "6", "-d", "10", "--speed", "5", "--yes"]

# name, sim args, cli args, expectations
CASES = [
    ("A_sticky_error", ["--error", "7", "--sticky-error"], JOG,
     dict(exit=1, moveSent=False, stopSent=True, contains=["still reports error 7", "confirmed the arm is stopped"])),
    ("B_clearable_error", ["--error", "7"], JOG, dict(exit=0, moveSent=True, executed=True, stopSent=False)),
    ("C_target_offset", ["--target-offset", "5"], JOG, dict(exit=1, anyWrite=False, contains=["could make the arm jump"])),
    ("D_no_report", ["--no-report"], JOG, dict(exit=1, anyWrite=False, contains=["Cannot read the measured joint"])),
    ("E_stale_target", ["--stale-target", "5"], JOG, dict(exit=1, anyWrite=False, contains=["could make the arm jump"])),
    ("F_queued", ["--queued", "3"], JOG, dict(exit=1, anyWrite=False, contains=["3 queued command"])),
    ("G_jog_ignored", ["--ignore-moves"], JOG,
     dict(exit=1, moveSent=True, executed=False, stopSent=True, contains=["ended 5.00 deg", "confirmed the arm is stopped"])),
    ("G2_tiny_jog_ignored", ["--ignore-moves"], ["jog", LOCAL, "-j", "6", "-d", "0.4", "--yes"],
     dict(exit=1, moveSent=True, executed=False, stopSent=True, contains=["ended 0.40 deg"])),
    ("H_home_sticky_error", ["--error", "3", "--sticky-error"], ["home", LOCAL, "--yes"],
     dict(exit=1, moveSent=False, stopSent=True, contains=["still reports error 3"])),
    ("I_watch_no_report", ["--no-report"], ["watch", LOCAL, "--count", "1"], dict(exit=1, anyWrite=False)),
    ("J_status_error", ["--error", "7"], ["status", LOCAL],
     dict(exit=0, anyWrite=False, contains=["Error/warning : 7 (unknown) / 0", "reports error 7 (unknown)"])),
    ("K_servo_unpowered", ["--servo-comm", "-3", "--error", "1", "--sticky-error"], JOG,
     dict(exit=1, anyWrite=False, contains=["emergency stop is active", f"reset {LOCAL}"])),
    ("K2_status_unpowered", ["--servo-comm", "-3", "--error", "1"], ["status", LOCAL],
     dict(exit=0, anyWrite=False, contains=["comm [-3 -3 -3 -3 -3 -3]", "1 (ESTOP_SWITCH)", "emergency stop is active",
                                            "placeholders"])),
    ("K3_joints_silent", ["--servo-comm", "-3"], JOG,
     dict(exit=1, anyWrite=False, contains=["can't reach the joints", f"reset {LOCAL}"])),
    ("Z1_reset_estop", ["--error", "1", "--sticky-error", "--servo-comm", "-3"], ["reset", LOCAL],
     dict(exit=1, onlyReset=True, contains=["Error now: 1 (ESTOP_SWITCH)", "emergency stop is active"])),
    ("Z2_reset_ok", ["--error", "7"], ["reset", LOCAL],
     dict(exit=0, onlyReset=True, contains=["Error now: 0", "No errors and every joint answers"])),
    ("Z3_reset_queued", ["--queued", "3"], ["reset", LOCAL], dict(exit=1, anyWrite=False, contains=["3 queued"])),
    ("Z4_reset_joints_silent", ["--error", "10", "--servo-comm", "-3"], ["reset", LOCAL],
     dict(exit=1, onlyReset=True, contains=["joints still aren't answering"])),
    ("L_enable_fails", ["--enable-fails"], JOG,
     dict(exit=1, moveSent=False, stopSent=True, contains=["did not become enabled", "confirmed the arm is stopped"])),
    ("M_enable_jump", ["--enable-jump", "5"], JOG,
     dict(exit=1, moveSent=False, stopSent=True, contains=["target is 5.0 deg", "confirmed the arm is stopped"])),
    ("X_drift_after_prompt", ["--drift-on-connect", "2"], JOG,
     dict(exit=1, anyWrite=False, contains=["moved 5.0 deg since the plan was shown"])),
    ("Y_nan_after_move", ["--nan-after-move"], JOG,
     dict(exit=1, moveSent=True, executed=True, stopSent=True, contains=["unusable joint angles"])),
    ("N_pause_mid_move", ["--pause-after", "0.4"], SLOW_JOG,
     dict(exit=1, moveSent=True, executed=False, stopSent=True, contains=["ended early: the arm is paused", "confirmed"])),
    ("O_error_mid_move", ["--error-after", "0.4"], SLOW_JOG,
     dict(exit=1, moveSent=True, executed=False, stopSent=True, contains=["reported error 9", "confirmed"])),
    ("P_home_ignored", ["--ignore-moves"], ["home", LOCAL, "--yes"],
     dict(exit=1, moveSent=True, executed=False, stopSent=True, contains=["didn't move"])),
    ("Q_home_ok", [], ["home", LOCAL, "--speed", "30", "--yes"], dict(exit=0, moveSent=True, executed=True, stopSent=False)),
    ("R_jog_ok", [], ["jog", LOCAL, "-j", "2", "-d", "-10", "--speed", "20", "--yes"],
     dict(exit=0, moveSent=True, executed=True, stopSent=False, contains=["-40.00"])),
    ("S_stop", [], ["stop", LOCAL], dict(exit=0, stopSent=True, moveSent=False, contains=["Stopped"])),
    ("T_piped_yes_refused", [], ["jog", LOCAL, "-j", "6", "-d", "5"],
     dict(exit=1, anyWrite=False, stdin="yes\n", contains=["re-run with --yes"])),
    ("T2_empty_answer", [], ["jog", LOCAL, "-j", "6", "-d", "5"], dict(exit=1, anyWrite=False, contains=["Cancelled"])),
    ("U_status_ok", [], ["status", LOCAL],
     dict(exit=0, anyWrite=False, contains=["Joints (deg)  :     0.00   -30.00", "Targets (deg) :     0.00   -30.00"])),
    ("V_watch_ok", [], ["watch", LOCAL, "--count", "2", "--interval", "0.2"], dict(exit=0, anyWrite=False, contains=["stopped"])),
    ("W_find_ok", [], ["find", "--subnet", f"{LOCAL}/32"], dict(exit=0, anyWrite=False, contains=["Found a UTRA controller"])),
]

# Argument validation must fail before any connection attempt (192.0.2.1 is unroutable).
ARG_CASES = [
    (["jog", "192.0.2.1", "-j", "6", "-d", "nan", "--yes"], "--deg must be a finite number"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "inf", "--yes"], "--deg must be a finite number"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "5", "--max-step", "nan", "--yes"], "--max-step must be a finite number"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "5", "--max-step", "0", "--yes"], "--max-step must be in"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "5", "--max-step", "200", "--yes"], "--max-step must be in"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "0", "--yes"], "--deg must be at least 0.1 deg"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "-0.05", "--yes"], "--deg must be at least 0.1 deg"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "30", "--yes"], "exceeds the per-jog limit"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "5", "--speed", "nan", "--yes"], "--speed must be a finite number"),
    (["jog", "192.0.2.1", "-j", "6", "-d", "5", "--speed", "100", "--yes"], "--speed must be in"),
    (["home", "192.0.2.1", "--acc", "inf", "--yes"], "--acc must be a finite number"),
    (["find", "--timeout", "nan"], "--timeout must be between"),
]


def run_cli(args, stdin=None):
    if stdin is None:
        return subprocess.run([CLI_PY, ARM] + args, cwd=ROOT, capture_output=True, text=True,
                              stdin=subprocess.DEVNULL, timeout=120)
    return subprocess.run([CLI_PY, ARM] + args, cwd=ROOT, capture_output=True, text=True,
                          input=stdin, timeout=120)


def main():
    failures = 0
    print(f"CLI python: {CLI_PY}")
    for name, sim_args, cli_args, exp in CASES:
        sim, path = start_sim(sim_args, f"cli_{name}.log")
        try:
            t0 = time.monotonic()
            r = run_cli(cli_args, exp.get("stdin"))
            dt = time.monotonic() - t0
        finally:
            stop(sim)
        log = open(path).read()
        out = r.stdout + r.stderr
        facts = {"exit": r.returncode, "anyWrite": "WRITE" in log, "moveSent": "WRITE MOVEJ" in log,
                 "executed": "motion complete" in log, "stopSent": "WRITE MOTION_STATUS data=04" in log,
                 "onlyReset": "WRITE RESET_ERR" in log and all("WRITE RESET_ERR" in line
                                                                for line in log.splitlines() if "WRITE" in line)}
        bad = [f"{k}={facts[k]} (want {v})" for k, v in exp.items() if k in facts and facts[k] != v]
        bad += [f"missing {s!r}" for s in exp.get("contains", []) if s not in out]
        failures += bool(bad)
        print(f"{'PASS' if not bad else 'FAIL'} {name:<22} {dt:5.1f}s  {facts}")
        if bad:
            print("     problems:", "; ".join(bad))
            print("     output:", out.strip().replace("\n", "\n             "))
    for args, expected in ARG_CASES:
        t0 = time.monotonic()
        r = run_cli(args)
        dt = time.monotonic() - t0
        ok = r.returncode == 1 and expected in r.stderr and dt < 2.5
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'} args {' '.join(args[2:] if args[0] != 'find' else args):<40} {dt:4.1f}s  {r.stderr.strip()}")
    print(f"\n{failures} failure(s)")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
