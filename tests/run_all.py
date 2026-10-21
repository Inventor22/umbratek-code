#!/usr/bin/env python3
"""Runs every test suite, or the ones named, and prints one line per suite.

    python tests/run_all.py                    # everything
    python tests/run_all.py --quick            # smaller samples in the slow oracle suites
    python tests/run_all.py panel cli          # just these suites (see --list)
    python tests/run_all.py --python C:\\Python312\\python.exe   # run the code under test on another Python

Suites that drive the simulator each get their own loopback address (127.0.0.11, .12, ...) and panel port, so
they run side by side (--jobs, default 3). Oracle suites need the oracle environment (.venv-oracle: NumPy, SciPy,
OpenCV and Pinocchio, from conda-forge; see tests/README.md) and are reported SKIP, and the run fails, without it.
Each suite's full output goes to tests/logs/<suite>.out.
"""
import argparse
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LOGS = os.path.join(HERE, "logs")


def oracle_python():
    for path in (os.path.join(ROOT, ".venv-oracle", "python.exe"), os.path.join(ROOT, ".venv-oracle", "bin", "python")):
        if os.path.isfile(path):
            return path
    return None


def oracle_env(env):
    """The environment with the oracle environment activated: conda's DLLs (LAPACK and the like) load from its
    Library folders, which must be on PATH or NumPy's linear algebra crashes on Windows."""
    prefix = os.path.join(ROOT, ".venv-oracle")
    folders = [prefix] + [os.path.join(prefix, *p) for p in (("Library", "mingw-w64", "bin"), ("Library", "usr", "bin"),
                                                               ("Library", "bin"), ("Scripts",), ("bin",))]
    return dict(env, PATH=os.pathsep.join(folders + [env.get("PATH", "")]), CONDA_PREFIX=prefix)


# name: (what it checks, interpreter ("test" = the runner's Python, "oracle", or "node"), script and args,
# quick args, timeout s)
SUITES = {
    "cli": ("scripts/arm.py against the simulator", "test", ["cli_tests.py"], None, 600),
    "inproc": ("arm.py interrupted mid-move, in-process", "test", ["inproc_tests.py"], None, 300),
    "panel": ("jog_server.py against the simulator", "test", ["panel_tests.py"], None, 900),
    "js": ("the panel's IK, and its collision check against the server's (Node)", "test", ["js_tests.py"], None, 600),
    "collide": ("collision geometry against dense sampling (NumPy)", "oracle", ["collide_tests.py"], None, 300),
    "paths": ("check_path against dense sampling along moves", "test", ["path_tests.py", "6"], ["path_tests.py", "2"], 1800),
    "capture": ("the real arm's capture: our kinematics and tools against its measurements", "test",
                ["capture_tests.py"], None, 600),
    "sim": ("the simulator against the capture, and its registers and control port", "test", ["sim_tests.py"], None, 600),
}


def run_suite(name, index, args, results):
    _, interpreter, script, quick, timeout = SUITES[name]
    argv = list(quick if args.quick and quick else script)
    if interpreter == "oracle":
        exe = oracle_python()
        if exe is None:
            results[name] = ("SKIP", 0.0, "no oracle environment (.venv-oracle): see tests/README.md")
            return
    else:
        exe = sys.executable
    env = dict(os.environ, UTRA_TEST_HOST=f"127.0.0.{11 + index}", UTRA_TEST_HTTP_PORT=str(8800 + index),
               UTRA_TEST_LOGS=LOGS, UTRA_TEST_PY=args.python or sys.executable, PYTHONIOENCODING="utf-8",
               PYTHONUNBUFFERED="1")
    if interpreter == "oracle":
        env = oracle_env(env)
    out_path = os.path.join(LOGS, f"{name}.out")
    t0 = time.monotonic()
    with open(out_path, "w", encoding="utf-8") as out:
        try:
            code = subprocess.run([exe, os.path.join(HERE, argv[0]), *argv[1:]], cwd=ROOT, env=env, stdout=out,
                                  stderr=subprocess.STDOUT, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            code = "timeout"
    took = time.monotonic() - t0
    with open(out_path, encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()
    summary = next((line.strip() for line in reversed(lines) if line.strip()), "")
    results[name] = ("PASS" if code == 0 else "FAIL", took, summary if code != "timeout" else f"timed out after {timeout} s")


def main():
    parser = argparse.ArgumentParser(description="Run the test suites.")
    parser.add_argument("suites", nargs="*", help="suites to run (default: all)")
    parser.add_argument("--quick", action="store_true", help="smaller samples in the slow oracle suites")
    parser.add_argument("--jobs", type=int, default=3, help="suites to run at once (default 3)")
    parser.add_argument("--python", help="Python that runs the code under test (default: this one)")
    parser.add_argument("--list", action="store_true", help="list the suites and exit")
    args = parser.parse_args()
    if args.list:
        for name, (what, interpreter, *_rest) in SUITES.items():
            print(f"  {name:<10} {what}{' [oracle]' if interpreter == 'oracle' else ''}")
        return 0
    names = args.suites or list(SUITES)
    unknown = [n for n in names if n not in SUITES]
    if unknown:
        parser.error(f"unknown suite(s): {', '.join(unknown)}; see --list")
    os.makedirs(LOGS, exist_ok=True)
    print(f"Testing with {args.python or sys.executable}; oracle environment: {oracle_python() or 'MISSING'}")
    results, pending, running = {}, list(enumerate(names)), []
    t0 = time.monotonic()
    while pending or running:
        while pending and len(running) < max(1, args.jobs):
            index, name = pending.pop(0)
            thread = threading.Thread(target=run_suite, args=(name, list(SUITES).index(name), args, results))
            thread.start()
            running.append((name, thread))
        for name, thread in list(running):
            if not thread.is_alive():
                running.remove((name, thread))
                status, took, summary = results[name]
                print(f"{status:<4} {name:<10} {took:6.1f}s  {summary}", flush=True)
        time.sleep(0.1)
    failed = [n for n in names if results[n][0] != "PASS"]
    print(f"\n{len(names) - len(failed)}/{len(names)} suites passed in {time.monotonic() - t0:.0f} s"
          + (f"; see tests/logs/<suite>.out for {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
