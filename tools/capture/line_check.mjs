// Checks straight tool lines before the capture sends them: inverse kinematics along each line from the arm's
// pose, keeping the tool's orientation, and the joint path between samples against the arm itself, its cart and
// the floor, with the panel's own collision check. Usage: node tools/capture/line_check.mjs input.json
import { readFileSync } from "node:fs";

const input = JSON.parse(readFileSync(process.argv[2], "utf8"));
const ik = await import(new URL("../../scripts/web/ik.js", import.meta.url).href);
const { createChecker } = await import(new URL("../../scripts/web/collide.js", import.meta.url).href);
const solver = ik.createSolver(input.collision);
const checker = createChecker(input.collision);
const home = solver.pose(input.start);

const results = [];
for (const line of input.lines) {
  let q = input.start.slice();
  let worstJump = 0;
  let ok = true;
  let hit = null;
  let why = "";
  const n = Math.ceil(Math.abs(line.mm) / 5);
  for (let k = 1; k <= n; k += 1) {
    const p = home.p.slice();
    p[line.axis] += (line.mm / 1000) * (k / n);
    const sol = solver.solve(q, { p, R: home.R }, input.limits);
    if (!sol.ok) {
      ok = false;
      why = `no IK solution ${((Math.abs(line.mm) * k) / n).toFixed(0)} mm along`;
      break;
    }
    worstJump = Math.max(worstJump, ...sol.joints.map((v, i) => Math.abs(v - q[i])));
    const found = checker.checkPath(q, sol.joints);  // null when the path stays clear
    if (found) {
      ok = false;
      hit = { part: found.part, other: found.other, gapMm: +(found.gap * 1000).toFixed(1) };
      why = "collision";
      break;
    }
    q = sol.joints;
  }
  results.push({ axis: "XYZ"[line.axis], mm: line.mm, ok, why, worstJumpDegPer5mm: +worstJump.toFixed(3),
                 end: q.map((v) => +v.toFixed(3)), hit });
}
console.log(JSON.stringify({ homeMm: home.p.map((v) => +(v * 1000).toFixed(2)), results }, null, 1));
