"""In-process interruption tests for scripts/arm.py against tools/utra_sim.py.

    python tests/inproc_tests.py            # runs every case, each in its own process against its own simulator
    python tests/inproc_tests.py CASE       # one case, against a simulator that's already running
"""
import importlib.util
import io
import os
import subprocess
import sys
import time
from contextlib import redirect_stderr, redirect_stdout

from simrig import ARM_CLI, HOST, PY, start_sim, stop

LOCAL = HOST
CASES = ("ctrl_c_mid_move", "ctrl_c_during_prepare", "stop_with_sdk_lock_held", "stop_unreachable")


def load_cli():
    spec = importlib.util.spec_from_file_location("armcli", ARM_CLI)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def run(m, argv):
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        code = m.main(argv)
    return code, out.getvalue() + err.getvalue()


def case(which):
    m = load_cli()
    if which == "ctrl_c_mid_move":
        original = m.StatusReport.read
        seen = {"moving": 0}

        def read(self):
            frame = original(self)
            if frame["status"] == 1:
                seen["moving"] += 1
                if seen["moving"] == 3:
                    raise KeyboardInterrupt
            return frame

        m.StatusReport.read = read
        code, text = run(m, ["jog", LOCAL, "-j", "6", "-d", "10", "--speed", "5", "--yes"])
        ok = code == 1 and "Interrupted. Sent stop; the controller confirmed" in text and seen["moving"] == 3
        print("PASS" if ok else "FAIL", which, code, text.strip().splitlines()[-1])

    elif which == "ctrl_c_during_prepare":
        def prepare(arm, axis):
            arm.reset_err()
            raise KeyboardInterrupt

        m.prepare_for_motion = prepare
        code, text = run(m, ["jog", LOCAL, "-j", "6", "-d", "5", "--yes"])
        ok = code == 1 and "Interrupted. Sent stop; the controller confirmed" in text
        print("PASS" if ok else "FAIL", which, code, text.strip().splitlines()[-1])

    elif which == "stop_with_sdk_lock_held":
        with redirect_stdout(io.StringIO()):
            arm = m.open_arm(LOCAL)
        arm.mutex.acquire()  # what an interrupted SDK request leaves behind
        t0 = time.monotonic()
        confirmed = m.stop_arm(LOCAL, arm)
        dt = time.monotonic() - t0
        ok = confirmed and dt < 2
        print("PASS" if ok else "FAIL", which, f"confirmed={confirmed} in {dt:.2f}s")

    elif which == "stop_unreachable":
        t0 = time.monotonic()
        confirmed = m.stop_arm("192.0.2.1")
        dt = time.monotonic() - t0
        err = m.abort_motion("192.0.2.1", None, "Test.")
        ok = not confirmed and dt < 4 and "press the hardware E-stop" in str(err)
        print("PASS" if ok else "FAIL", which, f"confirmed={confirmed} in {dt:.1f}s; message: {err}")
    else:
        raise SystemExit(f"unknown case {which!r}")
    sys.stdout.flush()
    os._exit(0 if ok else 1)  # the SDK's receive thread would keep the process alive


def main():
    failures = 0
    print(f"code under test runs on: {PY}")
    for which in CASES:
        sim, _ = start_sim((), f"inproc_{which}.log")
        try:
            r = subprocess.run([PY, os.path.abspath(__file__), which], capture_output=True, text=True, timeout=60)
        finally:
            stop(sim)
        line = (r.stdout.strip().splitlines() or ["(no output)"])[-1]
        ok = r.returncode == 0 and line.startswith("PASS")
        failures += not ok
        print(f"  {'PASS' if ok else 'FAIL'} {line if ok else line + '  ' + r.stderr.strip()[-400:]}")
    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    if len(sys.argv) > 1:
        case(sys.argv[1])
    sys.exit(main())
