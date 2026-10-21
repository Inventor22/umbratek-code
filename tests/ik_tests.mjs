// Tests for scripts/web/ik.js. Usage: node tests/ik_tests.mjs config.json (tests/js_tests.py writes the config from
// utra_model and jog_server, so the solver is tested on the model the panel serves).
import { readFileSync } from "node:fs";

const {collision, limits} = JSON.parse(readFileSync(process.argv[2], "utf8"));
const ik = await import(new URL("../scripts/web/ik.js", import.meta.url).href);
const solver = ik.createSolver(collision);
const RAD = Math.PI / 180;

let passed = 0;
let failed = 0;
function check(name, ok, detail = "") {
  if (ok) passed += 1;
  else failed += 1;
  console.log(`  ${ok ? "PASS" : "FAIL"} ${name}${ok ? "" : `  <-- ${detail}`}`);
}

// A seeded random source, so failures repeat.
let seed = 12345;
const random = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
const between = (a, b) => a + (b - a) * random();

const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) * 1000;  // mm
function angleBetween(A, B) {
  const t = [0, 1, 2].reduce((s, i) => s + A[i][0] * B[i][0] + A[i][1] * B[i][1] + A[i][2] * B[i][2], 0);
  return Math.acos(Math.max(-1, Math.min(1, (t - 1) / 2))) / RAD;  // degrees
}

console.log("== forward kinematics match the controller");
{
  const q = [14.982, 32.512, 120.169, 92.483, 96.434, 63.004];
  const {p, R} = solver.pose(q);
  const [roll, pitch, yaw] = ik.rpy(R);
  check("position matches the controller's reported pose", distance(p, [0.27399, -0.06348, 0.63598]) < 0.02, p);
  check("roll, pitch and yaw match", Math.abs(roll - 79.714) < 0.002 && Math.abs(pitch + 62.079) < 0.002
    && Math.abs(yaw - 120.548) < 0.002, [roll, pitch, yaw]);
  const back = ik.rpy(ik.fromRpy(roll, pitch, yaw));
  check("fromRpy is rpy's inverse", Math.abs(back[0] - roll) < 1e-9 && Math.abs(back[1] - pitch) < 1e-9
    && Math.abs(back[2] - yaw) < 1e-9 && angleBetween(ik.fromRpy(roll, pitch, yaw), R) < 1e-6, back);
}

console.log("== solves poses the arm can reach, from a nearby seed");
{
  let worst = {pos: 0, rot: 0};
  let solved = 0;
  let tries = 0;
  let slowest = 0;
  for (let k = 0; k < 400; k += 1) {
    const truth = limits.map(([lo, hi]) => between(Math.max(lo, -170), Math.min(hi, 170)));
    if (Math.abs(truth[4]) < 8 || Math.abs(Math.abs(truth[4]) - 180) < 8) continue;  // keep clear of the wrist singularity
    if (Math.abs(truth[2]) < 8) continue;  // and of the straight elbow
    tries += 1;
    const target = solver.pose(truth);
    const start = truth.map((q, i) => Math.max(limits[i][0], Math.min(limits[i][1], q + between(-6, 6))));
    const t0 = performance.now();
    const result = solver.solve(start, target, limits);
    slowest = Math.max(slowest, performance.now() - t0);
    if (!result.ok) continue;
    solved += 1;
    const got = solver.pose(result.joints);
    worst.pos = Math.max(worst.pos, distance(got.p, target.p));
    worst.rot = Math.max(worst.rot, angleBetween(got.R, target.R));
  }
  check(`solves at least 99% of ${tries} reachable poses from a seed within 6 degrees`, solved >= tries * 0.99, `${solved}/${tries}`);
  check("lands within 0.05 mm and 0.011 degrees", worst.pos < 0.05 && worst.rot < 0.011, worst);
  check("each solve takes under 20 ms", slowest < 20, slowest.toFixed(2));
}

console.log("== follows a drag continuously, and stops at the edge of reach");
{
  // Folded, so there's room to slide: X slides out until the elbow straightens.
  const start = [0, -30, 110, -80, 90, 0];
  const home = solver.pose(start);
  let seedJoints = start;
  let reached = 0;
  let jump = 0;
  for (let mm = 1; mm <= 1500; mm += 1) {
    const target = {p: [home.p[0] + mm / 1000, home.p[1], home.p[2]], R: home.R};
    const result = solver.solve(seedJoints, target, limits);
    if (!result.ok) break;
    // Joints speed up near full stretch (a straight elbow), as 1/sin(J3); judge smoothness away from it.
    if (Math.abs(result.joints[2]) > 15) jump = Math.max(jump, ...result.joints.map((q, i) => Math.abs(q - seedJoints[i])));
    seedJoints = result.joints;
    reached = mm;
  }
  check("slides along X, 1 mm at a time, until the arm is stretched out", reached > 300 && Math.abs(seedJoints[2]) < 3,
    {reached, elbow: seedJoints[2]});
  check("no joint jumps more than 1 degree between 1 mm steps", jump < 1, jump.toFixed(2));
  let down = start;
  let lowered = 0;
  for (let mm = 1; mm <= 200; mm += 1) {
    const result = solver.solve(down, {p: [home.p[0], home.p[1], home.p[2] - mm / 1000], R: home.R}, limits);
    if (!result.ok) break;
    down = result.joints;
    lowered = mm;
  }
  check("lowers the tool 200 mm along Z", lowered === 200, lowered);
  const far = solver.solve(start, {p: [home.p[0] + 2, home.p[1], home.p[2]], R: home.R}, limits);
  check("a pose 2 m away is reported out of reach", !far.ok, far);

  let spun = start;
  let turned = 0;
  for (let deg = 1; deg <= 90; deg += 1) {
    const target = {p: home.p, R: ik.multiply(ik.rotation([0, 0, 1], deg * RAD), home.R)};
    const result = solver.solve(spun, target, limits);
    if (!result.ok) break;
    spun = result.joints;
    turned = deg;
  }
  const after = solver.pose(spun);
  check("turns the tool 90 degrees about Z with the flange held still", turned === 90 && distance(after.p, home.p) < 0.05,
    {turned, moved: distance(after.p, home.p)});
}

console.log("== respects the joint limits");
{
  const limited = limits.map(([lo, hi], i) => (i === 1 ? [-35, 35] : [lo, hi]));
  const start = [0, -30, 50, -10, 90, 0];
  const home = solver.pose(start);
  const result = solver.solve(start, solver.pose([0, -60, 80, -10, 90, 0]), limited);
  check("won't pass a joint's limit to reach a pose", !result.ok && result.atLimit === 1
    && result.joints[1] >= -35 - 1e-9, result);
  const inside = solver.solve(start, home, limited);
  check("solves in place when the pose is already reached", inside.ok && inside.joints.every((q, i) => Math.abs(q - start[i]) < 1e-6), inside);
}

console.log(`\n${passed}/${passed + failed} passed`);
process.exit(failed ? 1 : 0);
