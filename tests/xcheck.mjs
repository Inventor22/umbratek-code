// Cross-check: web/collide.js (the panel's in-browser check) must reach the same verdicts as utra_model.py (the
// server's). Usage: node tests/xcheck.mjs cases.json, where cases.json comes from tests/js_tests.py.
import fs from "node:fs";

const {createChecker} = await import(new URL("../scripts/web/collide.js", import.meta.url).href);
const {config, cases} = JSON.parse(fs.readFileSync(process.argv[2], "utf8"));
const checker = createChecker(config);
let bad = 0;
let worstAt = 0;
let worstGap = 0;
const t0 = performance.now();
for (const [n, c] of cases.entries()) {
  const hit = checker.checkPath(c.start, c.goal);
  const near = checker.clearances(c.start).slice(0, 3);
  near.forEach((x, i) => {
    if (Math.abs(x.gap - c.clear[i][0]) > 1e-9 || x.part !== c.clear[i][1]) {
      bad += 1;
      console.log("clearance mismatch", n, x, c.clear[i]);
    }
  });
  if (Boolean(hit) !== Boolean(c.hit)) {
    bad += 1;
    console.log("verdict mismatch", n, hit, c.hit);
    continue;
  }
  if (hit) {
    if (hit.part !== c.hit.part || hit.other !== c.hit.other || hit.obstacle !== c.hit.obstacle) {
      bad += 1;
      console.log("hit mismatch", n, hit, c.hit);
    }
    worstAt = Math.max(worstAt, Math.abs(hit.at - c.hit.at));
    worstGap = Math.max(worstGap, Math.abs(hit.gap - c.hit.gap));
  }
}
if (worstAt > 1e-6 || worstGap > 1e-9) bad += 1;
const ms = performance.now() - t0;
console.log(`  ${bad ? "FAIL" : "PASS"} collide.js agrees with utra_model.py on ${cases.length} paths (${(ms / cases.length).toFixed(2)} ms ` +
  `each): ${bad} mismatches; worst |at| diff ${worstAt.toExponential(2)}, |gap| diff ${worstGap.toExponential(2)}`);
process.exit(bad ? 1 : 0);
