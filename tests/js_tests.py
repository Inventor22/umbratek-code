"""Runs the JavaScript suites under Node: tests/ik_tests.mjs (the panel's inverse kinematics) and tests/xcheck.mjs
(the panel's in-browser collision check against the server's). Writes their inputs from the Python model first, so
the browser code is always tested on the model the server serves. Usage: python tests/js_tests.py"""
import json
import os
import random
import subprocess
import sys

from simrig import ROOT, SCRIPTS, log_path

sys.path.insert(0, SCRIPTS)
import jog_server  # noqa: E402
import utra_model  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def xcheck_cases(model, count=120, seed=7):
    """Random joint moves, short and long, with the Python model's verdict and nearest clearances."""
    rng = random.Random(seed)
    limits = jog_server.DRAG_LIMITS_DEG
    cases = []
    while len(cases) < count:
        start = [rng.uniform(lo, hi) for lo, hi in limits]
        if len(cases) % 3 == 0:
            goal = [max(lo, min(hi, s + rng.uniform(-40, 40))) for s, (lo, hi) in zip(start, limits)]
        else:
            goal = [rng.uniform(lo, hi) for lo, hi in limits]
        hit = model.check_path(start, goal)
        cases.append({"start": start, "goal": goal,
                      "hit": None if hit is None else {k: hit[k] for k in ("part", "other", "obstacle", "gap", "at")},
                      "clear": [list(c) for c in model.clearances(start)[:3]]})
    return cases


def main():
    model = utra_model.ArmModel()
    config = model.collision_config()
    ik_input = log_path("ik_config.json")
    with open(ik_input, "w") as f:
        json.dump({"collision": config, "limits": jog_server.DRAG_LIMITS_DEG}, f)
    xcheck_input = log_path("xcheck_cases.json")
    with open(xcheck_input, "w") as f:
        json.dump({"config": config, "cases": xcheck_cases(model)}, f)
    failures = 0
    for script, arg in (("ik_tests.mjs", ik_input), ("xcheck.mjs", xcheck_input)):
        print(f"== node {script}")
        r = subprocess.run(["node", os.path.join(HERE, script), arg], cwd=ROOT, capture_output=True, text=True,
                           timeout=600)
        print(r.stdout.rstrip())
        if r.returncode != 0:
            failures += 1
            print(f"  FAIL {script} exited {r.returncode}: {r.stderr.strip()[-600:]}")
    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
