// Inverse kinematics for the jog panel's tool gizmo: the joint angles that put the flange at a given pose.
// Damped least squares (Levenberg-Marquardt) from a nearby solution, so the arm keeps its elbow and wrist
// configuration as the pose moves. The forward kinematics are collide.js's, the model the collision check and
// the server use; its flange frame is the controller's TCP frame (checked against the controller's reported
// pose to 0.01 mm and 0.001 degrees). Poses are {p: [x, y, z] in metres, R: 3x3 rotation as rows} in the arm's
// base frame; joint angles are degrees.
import { createChecker } from "./collide.js";

const RAD = Math.PI / 180;
const RHO = 0.3;  // metres per radian: how much a rotation error weighs against a position error
const TOL_POS = 1e-5;  // stop iterating within 0.01 mm...
const TOL_ROT = 1e-4;  // ...and 0.006 degrees
const OK_POS = 5e-5;  // a solution counts within 0.05 mm...
const OK_ROT = 2e-4;  // ...and 0.011 degrees
const MAX_STEP = 0.2;  // radians any joint may turn in one iteration

const clamp = (x, low, high) => (x < low ? low : x > high ? high : x);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// Rotation by `angle` radians about the unit vector `axis`, as rows.
export function rotation(axis, angle) {
  const [x, y, z] = axis;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const v = 1 - c;
  return [[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
    [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
    [z * x * v - y * s, z * y * v + x * s, c + z * z * v]];
}

export function multiply(a, b) {
  return [0, 1, 2].map((i) => [0, 1, 2].map((j) => a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]));
}

// Roll, pitch and yaw in degrees, as the controller reports them: R = Rz(yaw) Ry(pitch) Rx(roll).
export function rpy(R) {
  const pitch = Math.asin(clamp(-R[2][0], -1, 1));
  if (Math.abs(R[2][0]) > 1 - 1e-9) {  // pitch at +-90 degrees: roll and yaw share an axis; put it all in yaw
    return [0, pitch / RAD, Math.atan2(-R[0][1], R[1][1]) / RAD];
  }
  return [Math.atan2(R[2][1], R[2][2]) / RAD, pitch / RAD, Math.atan2(R[1][0], R[0][0]) / RAD];
}

// The rotation for roll, pitch and yaw in degrees, the controller's way round.
export function fromRpy(roll, pitch, yaw) {
  return multiply(rotation([0, 0, 1], yaw * RAD), multiply(rotation([0, 1, 0], pitch * RAD), rotation([1, 0, 0], roll * RAD)));
}

// The rotation vector w (radians, base frame) that turns R into Rt: Rt = exp([w]) R.
function rotationError(Rt, R) {
  const E = [0, 1, 2].map((i) => [0, 1, 2].map((j) => Rt[i][0] * R[j][0] + Rt[i][1] * R[j][1] + Rt[i][2] * R[j][2]));
  const angle = Math.acos(clamp((E[0][0] + E[1][1] + E[2][2] - 1) / 2, -1, 1));
  const v = [E[2][1] - E[1][2], E[0][2] - E[2][0], E[1][0] - E[0][1]];
  if (angle < 1e-7) return v.map((x) => x / 2);
  if (Math.PI - angle < 1e-4) {
    // Half a turn: the axis is the column of (E + I) / 2 with the largest diagonal.
    const k = [0, 1, 2].reduce((best, i) => (E[i][i] > E[best][best] ? i : best), 0);
    const axis = [0, 1, 2].map((i) => (E[i][k] + (i === k ? 1 : 0)) / 2);
    const length = Math.hypot(...axis) || 1;
    return axis.map((x) => (x / length) * angle);
  }
  const scale = angle / (2 * Math.sin(angle));
  return v.map((x) => x * scale);
}

// Solves the 6x6 system A x = b by Gaussian elimination with partial pivoting; null when it's singular.
function solveLinear(A, b) {
  const n = b.length;
  const m = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c += 1) {
    let pivot = c;
    for (let r = c + 1; r < n; r += 1) if (Math.abs(m[r][c]) > Math.abs(m[pivot][c])) pivot = r;
    if (Math.abs(m[pivot][c]) < 1e-14) return null;
    [m[c], m[pivot]] = [m[pivot], m[c]];
    for (let r = c + 1; r < n; r += 1) {
      const f = m[r][c] / m[c][c];
      for (let k = c; k <= n; k += 1) m[r][k] -= f * m[c][k];
    }
  }
  const x = Array(n).fill(0);
  for (let r = n - 1; r >= 0; r -= 1) {
    let sum = m[r][n];
    for (let k = r + 1; k < n; k += 1) sum -= m[r][k] * x[k];
    x[r] = sum / m[r][r];
  }
  return x;
}

export function createSolver(config) {
  const {frames} = createChecker(config);
  const axes = config.joints.map((joint) => joint.axis);
  const n = axes.length;

  const flangeOf = (F) => {
    const t = F[n - 1];
    return {p: [t[0][3], t[1][3], t[2][3]], R: [0, 1, 2].map((i) => [t[i][0], t[i][1], t[i][2]])};
  };

  // The flange's pose for the given joint angles (degrees).
  function pose(degrees) {
    return flangeOf(frames(degrees));
  }

  function evaluate(q, target) {
    const F = frames(q.map((v) => v / RAD));
    const {p, R} = flangeOf(F);
    const ep = [target.p[0] - p[0], target.p[1] - p[1], target.p[2] - p[2]];
    const er = rotationError(target.R, R);
    const e = [...ep, er[0] * RHO, er[1] * RHO, er[2] * RHO];
    return {F, p, e, norm: Math.hypot(...e), posErr: Math.hypot(...ep), rotErr: Math.hypot(...er)};
  }

  // Rows: the flange's linear velocity, then its angular velocity (weighted by RHO), per joint's turn.
  function jacobian(F, p) {
    const J = Array.from({length: 6}, () => Array(n).fill(0));
    for (let i = 0; i < n; i += 1) {
      const t = F[i];
      const a = axes[i];
      const z = [0, 1, 2].map((r) => t[r][0] * a[0] + t[r][1] * a[1] + t[r][2] * a[2]);
      const v = cross(z, [p[0] - t[0][3], p[1] - t[1][3], p[2] - t[2][3]]);
      for (let r = 0; r < 3; r += 1) {
        J[r][i] = v[r];
        J[r + 3][i] = RHO * z[r];
      }
    }
    return J;
  }

  // Joint angles (degrees) that put the flange at `target`, starting from `seed` and staying within `limits`
  // ([low, high] degrees per joint). `ok` is false when the solver can't get there from the seed: the pose is
  // out of reach, needs a joint past its limit, or sits at a singularity. `joints` is then the closest it got,
  // and `atLimit` the first joint pressed against a limit (or -1).
  function solve(seed, target, limits, maxIterations = 100) {
    const low = limits.map(([l]) => l * RAD);
    const high = limits.map(([, h]) => h * RAD);
    let q = seed.map((d, i) => clamp(d * RAD, low[i], high[i]));
    let state = evaluate(q, target);
    let lambda = 1e-3;
    for (let k = 0; k < maxIterations && !(state.posErr < TOL_POS && state.rotErr < TOL_ROT); k += 1) {
      const J = jacobian(state.F, state.p);
      const A = J.map((row, r) => J.map((other, c) => row.reduce((sum, x, i) => sum + x * other[i], 0)
        + (r === c ? lambda * lambda : 0)));
      const y = solveLinear(A, state.e);
      if (!y) {
        lambda *= 10;
        continue;
      }
      let dq = Array.from({length: n}, (_, i) => J.reduce((sum, row, r) => sum + row[i] * y[r], 0));
      const biggest = Math.max(...dq.map(Math.abs));
      if (biggest > MAX_STEP) dq = dq.map((v) => (v * MAX_STEP) / biggest);
      const next = q.map((v, i) => clamp(v + dq[i], low[i], high[i]));
      const trial = evaluate(next, target);
      if (trial.norm < state.norm) {
        q = next;
        state = trial;
        lambda = Math.max(lambda / 3, 1e-7);
      } else {
        lambda *= 8;
        if (lambda > 100) break;
      }
    }
    const ok = state.posErr < OK_POS && state.rotErr < OK_ROT;
    const atLimit = ok ? -1 : q.findIndex((v, i) => v <= low[i] + 1e-6 || v >= high[i] - 1e-6);
    return {joints: q.map((v) => v / RAD), ok, atLimit, posErr: state.posErr, rotErr: state.rotErr};
  }

  return {pose, solve};
}
