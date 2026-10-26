"""Validate utra_model.check_path against dense sampling, and time it. Usage: python tests/path_tests.py [paths]"""
import math
import os
import random
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import utra_model as um  # noqa: E402

PATHS = int(sys.argv[1]) if len(sys.argv) > 1 else 40

m = um.ArmModel()
rng = random.Random(11)
LIMITS = [(-180, 180)] * 2 + [(-165, 165)] + [(-180, 180)] * 3


def clear_pose():
    while True:
        pose = [rng.uniform(*lim) for lim in LIMITS]
        if m.clearances(pose)[0][0] >= um.MARGIN_M:
            return pose


def dense_min(start, goal, limits, steps):
    """Smallest (gap - limit) below zero over dense samples, with where it happens (0 if never below)."""
    worst = (0.0, None, None)
    for k in range(steps + 1):
        t = k / steps
        pose = [s + (g - s) * t for s, g in zip(start, goal)]
        placed = m.placed(pose)
        for p, pair in enumerate(m.pairs):
            if m._lower_bound(placed, pair) >= limits[p]:
                continue
            gap, _ = m._gap(placed, pair)
            if gap - limits[p] < worst[0]:
                worst = (gap - limits[p], t, m._names(pair))
    return worst


times, verdicts, bad = [], [], 0
for n in range(PATHS):
    start = clear_pose()
    if n % 3 == 0:  # short moves of a few joints, like a staged plan
        goal = list(start)
        for j in rng.sample(range(6), rng.randint(1, 3)):
            goal[j] = max(LIMITS[j][0], min(LIMITS[j][1], start[j] + rng.uniform(-60, 60)))
    else:
        goal = [rng.uniform(*lim) for lim in LIMITS]
    t0 = time.perf_counter()
    hit = m.check_path(start, goal)
    times.append(time.perf_counter() - t0)
    verdicts.append(hit is None)
    # Dense enough that no point moves more than 0.5 mm between samples.
    turn = [abs(math.radians(g - s)) for s, g in zip(start, goal)]
    fastest = max(sum(turn[j] * m.reach[c][j] + turn[j] * m.capsules[c][3] for j in range(len(m.reach[c])))
                  for c in range(len(m.capsules)))
    limits = [um.MARGIN_M] * len(m.pairs)  # start is clear, so no pair has its limit adjusted
    worst, at, names = dense_min(start, goal, limits, max(20, int(fastest / 0.0005)))
    if hit is None and worst < -um.ADVANCE_M - 0.0005:
        bad += 1
        print(f"  MISSED: dense check finds {names} {worst * 1000:.2f} mm past the margin at t={at:.4f}")
    if hit is not None:
        if hit["gap"] >= um.MARGIN_M:
            bad += 1
            print("  BAD HIT (gap not below margin):", hit)
        if worst >= 0:
            bad += 1
            print("  FALSE HIT: dense check never goes below the margin", hit)
        elif at > hit["at"] + 1e-9 and False:
            pass
    print(f"  path {n}: {'clear' if hit is None else 'blocked at %.3f by %s/%s' % (hit['at'], hit['part'], hit['other'])}"
          f" in {times[-1] * 1000:.0f} ms; dense worst {worst * 1000:.2f} mm", flush=True)
print(f"{len(times)} paths: {sum(verdicts)} clear, {len(times) - sum(verdicts)} blocked; {bad} problems")
print(f"check_path time: median {statistics.median(times) * 1000:.1f} ms, max {max(times) * 1000:.1f} ms")

# From inside a keep-out zone the gap is signed, so a part already inside may move out, or no more than SLACK_M
# further in: check that on short moves starting inside a big zone, against dense sampling with the same limits.
cage = um.Obstacle("the cage zone", "box", [0.35, 0.0, 0.55], [[0.8, 0.6, 0], [-0.6, 0.8, 0], [0, 0, 1]], [0.7, 0.6, 0.5])
caged = m.with_cell((), [cage])
inside_paths, inside_hits = 0, 0
for n in range(400):
    if inside_paths >= max(6, PATHS // 5):
        break
    start = [rng.uniform(*lim) for lim in LIMITS]
    placed = caged.placed(start)
    limits = [um.MARGIN_M] * len(caged.pairs)
    for p, pair in enumerate(caged.pairs):
        gap, _ = caged._gap(placed, pair)
        if gap < um.MARGIN_M:
            limits[p] = gap - um.SLACK_M
    in_cage = [p for p, pair in enumerate(caged.pairs) if pair[2] and pair[1] == len(caged.obstacles) - 1
               and limits[p] < -um.SLACK_M]
    if not in_cage:
        continue
    inside_paths += 1
    goal = list(start)
    for j in rng.sample(range(6), rng.randint(1, 3)):
        goal[j] = max(LIMITS[j][0], min(LIMITS[j][1], start[j] + rng.uniform(-25, 25)))
    hit = caged.check_path(start, goal)
    turn = [abs(math.radians(g - s)) for s, g in zip(start, goal)]
    fastest = max(sum(turn[j] * caged.reach[c][j] + turn[j] * caged.capsules[c][3] for j in range(len(caged.reach[c])))
                  for c in range(len(caged.capsules)))
    worst = (0.0, None, None)
    steps = max(20, int(fastest / 0.0005))
    for k in range(1, steps + 1):
        t = k / steps
        pose = [s + (g - s) * t for s, g in zip(start, goal)]
        placed = caged.placed(pose)
        for p, pair in enumerate(caged.pairs):
            if caged._lower_bound(placed, pair) >= limits[p]:
                continue
            gap, _ = caged._gap(placed, pair)
            if gap - limits[p] < worst[0]:
                worst = (gap - limits[p], t, caged._names(pair))
    if hit is None and worst[0] < -um.ADVANCE_M - 0.0005:
        bad += 1
        print(f"  MISSED from inside: dense check finds {worst[2]} {worst[0] * 1000:.2f} mm past its limit at "
              f"t={worst[1]:.4f}")
    if hit is not None:
        inside_hits += 1
        if worst[0] >= 0:
            bad += 1
            print("  FALSE HIT from inside: dense check never goes past a limit", hit)
    verdict = "clear" if hit is None else \
        f"blocked: {hit['part']} {'deeper into' if hit.get('inside') else 'near'} {hit['other']}"
    print(f"  from inside the cage, path {inside_paths}: {verdict}; dense worst {worst[0] * 1000:.2f} mm", flush=True)
print(f"{inside_paths} short paths from inside a zone: {inside_hits} blocked")
if inside_paths < 6:
    bad += 1
    print("  too few starting poses inside the zone were found")
print(f"\n{bad} failure(s)")
sys.exit(1 if bad else 0)

