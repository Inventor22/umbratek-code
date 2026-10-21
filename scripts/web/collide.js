// Collision checks for the jog panel's 3D view: a line-for-line port of utra_model.py's check_path, run on
// the data from ArmModel.collision_config() so the preview gives the server's verdict. No dependencies.
const EPS = 1e-12;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const lerp = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
const clamp = (x, low, high) => (x < low ? low : x > high ? high : x);
const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

// Closest points between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection, 5.1.9).
export function segmentSegment(p1, q1, p2, q2) {
  const d1 = sub(q1, p1);
  const d2 = sub(q2, p2);
  const r = sub(p1, p2);
  const a = dot(d1, d1);
  const e = dot(d2, d2);
  const f = dot(d2, r);
  let s;
  let t;
  if (a <= EPS && e <= EPS) {
    s = 0;
    t = 0;
  } else if (a <= EPS) {
    s = 0;
    t = clamp(f / e, 0, 1);
  } else {
    const c = dot(d1, r);
    if (e <= EPS) {
      s = clamp(-c / a, 0, 1);
      t = 0;
    } else {
      const b = dot(d1, d2);
      const denom = a * e - b * b;
      s = denom > EPS ? clamp((b * f - c * e) / denom, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        s = clamp(-c / a, 0, 1);
        t = 0;
      } else if (t > 1) {
        s = clamp((b - c) / a, 0, 1);
        t = 1;
      }
    }
  }
  return [lerp(p1, q1, s), lerp(p2, q2, t)];
}

// The point of an obstacle (box or cylinder, as in collision_config) nearest to x.
export function nearest(o, x) {
  const v = sub(x, o.centre);
  if (o.kind === "box") {
    const out = [...o.centre];
    for (let k = 0; k < 3; k += 1) {
      const d = clamp(dot(v, o.axes[k]), -o.half[k], o.half[k]);
      for (let i = 0; i < 3; i += 1) out[i] += o.axes[k][i] * d;
    }
    return out;
  }
  let h = dot(v, o.axis);
  const radial = [v[0] - o.axis[0] * h, v[1] - o.axis[1] * h, v[2] - o.axis[2] * h];
  const rho = Math.hypot(...radial);
  const scale = rho > 0 ? Math.min(1, o.radius / rho) : 0;
  h = clamp(h, -o.halfLength, o.halfLength);
  return [0, 1, 2].map((i) => o.centre[i] + o.axis[i] * h + radial[i] * scale);
}

function boxSegment(o, p, q) {
  const a = o.axes.map((axis) => dot(sub(p, o.centre), axis));
  const d = o.axes.map((axis) => dot(sub(q, p), axis));
  const cuts = [0, 1];
  for (let k = 0; k < 3; k += 1) {
    if (d[k] === 0) continue;
    for (const side of [-o.half[k], o.half[k]]) {
      const t = (side - a[k]) / d[k];
      if (t > 0 && t < 1) cuts.push(t);
    }
  }
  cuts.sort((x, y) => x - y);
  let best = null;
  for (let n = 0; n + 1 < cuts.length; n += 1) {
    // Between cuts each coordinate stays below, inside or above the box: the squared distance is a quadratic.
    const t0 = cuts[n];
    const t1 = cuts[n + 1];
    const middle = 0.5 * (t0 + t1);
    let num = 0;
    let den = 0;
    for (let k = 0; k < 3; k += 1) {
      const x = a[k] + d[k] * middle;
      let side;
      if (x > o.half[k]) side = o.half[k];
      else if (x < -o.half[k]) side = -o.half[k];
      else continue;
      num -= d[k] * (a[k] - side);
      den += d[k] * d[k];
    }
    const t = den <= 0 ? t0 : clamp(num / den, t0, t1);
    const local = [0, 1, 2].map((k) => a[k] + d[k] * t);
    const near = [0, 1, 2].map((k) => clamp(local[k], -o.half[k], o.half[k]));
    const gap = (local[0] - near[0]) ** 2 + (local[1] - near[1]) ** 2 + (local[2] - near[2]) ** 2;
    if (!best || gap < best.gap) best = {gap, t, near};
  }
  const point = [0, 1, 2].map((i) => o.centre[i] + best.near[0] * o.axes[0][i] + best.near[1] * o.axes[1][i]
    + best.near[2] * o.axes[2][i]);
  return [lerp(p, q, best.t), point];
}

// Closest points (on the segment p-q, on the obstacle).
export function nearestToSegment(o, p, q) {
  if (o.kind === "box") return boxSegment(o, p, q);
  // The distance from a convex shape is convex along a segment: a golden-section search finds its minimum.
  const gap = (t) => {
    const x = lerp(p, q, t);
    return distance(x, nearest(o, x));
  };
  let low = 0;
  let high = 1;
  const ratio = (Math.sqrt(5) - 1) / 2;
  let t1 = high - ratio * (high - low);
  let t2 = low + ratio * (high - low);
  let g1 = gap(t1);
  let g2 = gap(t2);
  for (let n = 0; n < 40; n += 1) {
    if (g1 <= g2) {
      high = t2;
      t2 = t1;
      g2 = g1;
      t1 = high - ratio * (high - low);
      g1 = gap(t1);
    } else {
      low = t1;
      t1 = t2;
      g1 = g2;
      t2 = low + ratio * (high - low);
      g2 = gap(t2);
    }
  }
  let best = 0;
  let bestGap = gap(0);
  for (const t of [1, 0.5 * (low + high)]) {
    const g = gap(t);
    if (g < bestGap) {
      best = t;
      bestGap = g;
    }
  }
  const x = lerp(p, q, best);
  return [x, nearest(o, x)];
}

function rotation(axis, angle) {
  const [x, y, z] = axis;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const v = 1 - c;
  return [[c + x * x * v, x * y * v - z * s, x * z * v + y * s, 0],
    [y * x * v + z * s, c + y * y * v, y * z * v - x * s, 0],
    [z * x * v - y * s, z * y * v + x * s, c + z * z * v, 0],
    [0, 0, 0, 1]];
}

function matmul(a, b) {
  const out = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i += 1) {
    for (let j = 0; j < 4; j += 1) {
      out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j];
    }
  }
  return out;
}

const apply = (t, p) => [0, 1, 2].map((i) => t[i][0] * p[0] + t[i][1] * p[1] + t[i][2] * p[2] + t[i][3]);

export function createChecker(config) {
  const {capsules, obstacles, pairs, reach, joints, labels} = config;
  const halfLengths = capsules.map((c) => distance(c.a, c.b) / 2);

  function frames(pose) {
    let t = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    return joints.map((joint, i) => {
      t = matmul(t, matmul(joint.fixed, rotation(joint.axis, (pose[i] * Math.PI) / 180)));
      return t;
    });
  }

  // Each capsule's ends in the base frame, for the given joint angles (degrees).
  function placed(pose) {
    const f = frames(pose);
    return capsules.map((c) => [apply(f[c.body - 1], c.a), apply(f[c.body - 1], c.b)]);
  }

  function lowerBound(at, [c, o, fixed]) {
    const middle = lerp(at[c][0], at[c][1], 0.5);
    if (fixed) return distance(middle, nearest(obstacles[o], middle)) - halfLengths[c] - capsules[c].radius;
    const other = lerp(at[o][0], at[o][1], 0.5);
    return distance(middle, other) - halfLengths[c] - capsules[c].radius - halfLengths[o] - capsules[o].radius;
  }

  function gapOf(at, [c, o, fixed]) {
    const [a, b] = at[c];
    if (fixed) {
      const points = nearestToSegment(obstacles[o], a, b);
      return {gap: distance(...points) - capsules[c].radius, points};
    }
    const points = segmentSegment(a, b, at[o][0], at[o][1]);
    return {gap: distance(...points) - capsules[c].radius - capsules[o].radius, points};
  }

  const names = ([c, o, fixed]) => [capsules[c].label || labels[capsules[c].body],
    fixed ? obstacles[o].label : capsules[o].label || labels[capsules[o].body]];

  // Where a straight joint-space move first brings two things closer than the margin, or null; see
  // ArmModel.check_path for how the continuous check works.
  function checkPath(start, goal, margin = config.margin) {
    const turn = start.map((s, i) => Math.abs(((goal[i] - s) * Math.PI) / 180));
    const speeds = pairs.map(([c, , , low]) => {
      let sum = 0;
      for (let j = low; j < reach[c].length; j += 1) sum += turn[j] * reach[c][j];
      return sum;
    });
    const fastest = Math.max(0, ...speeds);
    const shortest = fastest > 0 ? config.advance / fastest : 1;
    const limits = pairs.map(() => margin);
    let t = 0;
    for (;;) {
      const pose = start.map((s, i) => s + (goal[i] - s) * t);
      const at = placed(pose);
      let step = Infinity;
      for (let p = 0; p < pairs.length; p += 1) {
        const speed = speeds[p];
        if (speed === 0 && t > 0) continue;  // nothing on this path moves these two relative to each other
        const lower = lowerBound(at, pairs[p]);
        if (lower >= limits[p] && (speed === 0 || lower - limits[p] >= step * speed)) continue;
        const {gap, points} = gapOf(at, pairs[p]);
        if (gap < limits[p]) {
          if (t > 0) {
            const [part, other] = names(pairs[p]);
            return {part, other, obstacle: pairs[p][2] ? pairs[p][1] : null, gap, at: t, pose, points};
          }
          limits[p] = gap - config.slack;  // already this close: may move apart, not closer
        }
        if (speed > 0) step = Math.min(step, (gap - limits[p]) / speed);
      }
      if (t >= 1) return null;
      t = Math.min(1, t + Math.max(step, shortest));
    }
  }

  // Every checked pair's gap (m) at one pose, nearest first.
  function clearances(pose) {
    const at = placed(pose);
    return pairs.map((pair) => {
      const [part, other] = names(pair);
      return {gap: gapOf(at, pair).gap, part, other};
    }).sort((x, y) => x.gap - y.gap);
  }

  return {checkPath, clearances, placed, frames};
}
