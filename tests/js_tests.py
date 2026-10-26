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


def entering_cases(model, plain, zone, count=24, seed=9):
    """Short moves that start with part of the arm just outside the guarded `zone` (0.1 to 0.9 mm off it) and end a
    little inside it, by less than the 1 mm a part already that close may come closer to anything else: moves the same
    model with the zone unguarded (`plain`) lets through, with its verdict and nearest clearances, as xcheck_cases."""
    rng = random.Random(seed)
    limits = jog_server.DRAG_LIMITS_DEG

    def gap(q):
        return model.obstacle_gaps(q, {zone})[zone][0]

    def pose(a, b, t):
        return [x + (y - x) * t for x, y in zip(a, b)]

    cases = []
    for _ in range(4000):
        if len(cases) >= count:
            break
        a, b = [rng.uniform(lo, hi) for lo, hi in limits], [rng.uniform(lo, hi) for lo, hi in limits]
        if gap(a) < 0.02 or gap(b) > -0.02:
            continue
        level, lo, hi = rng.uniform(0.0001, 0.0009), 0.0, 1.0
        for _ in range(50):  # where the gap along a..b first comes down to `level`, give or take
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if gap(pose(a, b, mid)) > level else (lo, mid)
        start, deeper = pose(a, b, lo), rng.uniform(0.0002, 0.0008)
        t = lo
        for _ in range(400):  # on along a..b, in steps of a few hundredths of a degree, to `deeper` m into the zone
            t += 0.0001
            if t > 1.0 or gap(pose(a, b, t)) <= -deeper:
                break
        goal = pose(a, b, min(t, 1.0))
        g0, g1 = gap(start), gap(goal)
        if not (0.0 < g0 < 0.001 and g0 - 0.001 < g1 < 0.0) or plain.check_path(start, goal) is not None:
            continue
        hit = model.check_path(start, goal)
        cases.append({"start": start, "goal": goal, "startGap": g0, "endGap": g1,
                      "hit": None if hit is None else {k: hit[k] for k in ("part", "other", "obstacle", "gap", "at")},
                      "inside": None if hit is None else hit["inside"],
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
    # The same with a tool on the flange and keep-out zones, as the cell adds them: two small, and a cage so big that many
    # random poses start with part of the arm inside it, where the gap is signed (the depth inside, negated). The cage is
    # guarded, and closed, as interlocks that don't hold leave one: a part outside it may come no further than its face.
    def zones():
        return [utra_model.Obstacle("the press zone", "box", [0.45, 0.1, 0.1], [[1, 0, 0], [0, 0.8, -0.6], [0, 0.6, 0.8]],
                                    [0.2, 0.3, 0.25]),
                utra_model.Obstacle("the rack zone", "box", [-0.2, -0.45, 0.2], [[0.6, 0.8, 0], [-0.8, 0.6, 0], [0, 0, 1]],
                                    [0.15, 0.4, 0.5]),
                utra_model.Obstacle("the cage zone", "box", [0.35, 0.0, 0.55], [[0.8, 0.6, 0], [-0.6, 0.8, 0], [0, 0, 1]],
                                    [0.7, 0.6, 0.5])]
    tool = [(6, [0, 0, 0], [0, 0, 0.14], 0.035, "the tool"), (6, [0, 0.03, 0.14], [0, -0.03, 0.14], 0.02, "the tool")]
    guarded = zones()
    guarded[-1].guarded = True
    celled, plain = model.with_cell(tool, guarded), model.with_cell(tool, zones())
    cases = xcheck_cases(celled, seed=8)
    cage = len(celled.obstacles) - 1
    inside = sum(1 for c in cases if celled.obstacle_gaps(c["start"], {cage})[cage][0] < 0)
    deeper = sum(1 for c in cases if c["hit"] and c["hit"]["obstacle"] == cage and c["hit"]["gap"] < 0)
    print(f"  {inside} of {len(cases)} cell paths start with part of the arm inside the cage zone; {deeper} are refused "
          "for going deeper into it")
    if inside < 20 or deeper < 3:
        print("  FAIL too few paths start inside a zone to test the signed gap")
        return 1
    entering = entering_cases(celled, plain, cage)
    refused = sum(1 for c in entering if c["hit"] and c["hit"]["obstacle"] == cage and c["hit"]["gap"] < 0
                  and not c["inside"])
    print(f"  {len(entering)} more start {min((c['startGap'] for c in entering), default=0) * 1000:.2f} to "
          f"{max((c['startGap'] for c in entering), default=0) * 1000:.2f} mm outside the guarded cage zone and end "
          "less than 1 mm into it, which the same box unguarded lets through: "
          f"{refused} refused for going into it")
    if len(entering) < 12 or refused != len(entering):
        print("  FAIL a move from just outside a closed guarded zone into it must be refused, the allowance for parts "
              "already close stopping at its face")
        return 1
    xcheck_cell = log_path("xcheck_cell_cases.json")
    with open(xcheck_cell, "w") as f:
        json.dump({"config": celled.collision_config(), "cases": cases + entering}, f)
    failures = 0
    for script, arg in (("ik_tests.mjs", ik_input), ("xcheck.mjs", xcheck_input), ("xcheck.mjs", xcheck_cell)):
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
