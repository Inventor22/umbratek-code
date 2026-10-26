// Collision checks for the jog panel's 3D view: a line-for-line port of utra_model.py's check_path, run on
// the data from ArmModel.collision_config() so the preview gives the server's verdict. No dependencies.
const EPS = 1e-12;
const TOUCH = 1e-12;  // a segment closer than this to a box is taken to touch it (utra_model.TOUCH_M)

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

// How far x is from an obstacle (m): its distance outside, or minus its depth inside (how far it is from the nearest
// point of the surface).
export function signedDistance(o, x) {
  const v = sub(x, o.centre);
  if (o.kind === "box") {
    const q = o.axes.map((axis, k) => Math.abs(dot(v, axis)) - o.half[k]);
    const most = Math.max(q[0], q[1], q[2]);
    if (most <= 0) return most;
    return Math.sqrt(q.reduce((sum, c) => (c > 0 ? sum + c * c : sum), 0));
  }
  const h = dot(v, o.axis);
  const radial = [v[0] - o.axis[0] * h, v[1] - o.axis[1] * h, v[2] - o.axis[2] * h];
  const dr = Math.sqrt(dot(radial, radial)) - o.radius;
  const dh = Math.abs(h) - o.halfLength;
  if (dr <= 0 && dh <= 0) return Math.max(dr, dh);
  return Math.sqrt(Math.max(dr, 0) ** 2 + Math.max(dh, 0) ** 2);
}

// The point of an obstacle's surface nearest to x: nearest() outside, and inside the nearest face's point.
export function surfacePoint(o, x) {
  const v = sub(x, o.centre);
  if (o.kind === "box") {
    const local = o.axes.map((axis) => dot(v, axis));
    if (!local.every((c, k) => Math.abs(c) <= o.half[k])) return nearest(o, x);
    let k = 0;
    for (let i = 1; i < 3; i += 1) if (o.half[i] - Math.abs(local[i]) < o.half[k] - Math.abs(local[k])) k = i;
    local[k] = local[k] >= 0 ? o.half[k] : -o.half[k];
    return [0, 1, 2].map((i) => o.centre[i] + local[0] * o.axes[0][i] + local[1] * o.axes[1][i] + local[2] * o.axes[2][i]);
  }
  let h = dot(v, o.axis);
  let radial = [v[0] - o.axis[0] * h, v[1] - o.axis[1] * h, v[2] - o.axis[2] * h];
  let rho = Math.hypot(...radial);
  if (!(rho <= o.radius && Math.abs(h) <= o.halfLength)) return nearest(o, x);
  if (o.halfLength - Math.abs(h) < o.radius - rho) {  // nearer a cap
    h = h >= 0 ? o.halfLength : -o.halfLength;
    return [0, 1, 2].map((i) => o.centre[i] + o.axis[i] * h + radial[i]);
  }
  if (rho <= 0) {  // on the axis: any way out sideways is as near
    const side = Math.abs(o.axis[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    const along = dot(side, o.axis);
    radial = [side[0] - o.axis[0] * along, side[1] - o.axis[1] * along, side[2] - o.axis[2] * along];
    rho = Math.hypot(...radial);
  }
  return [0, 1, 2].map((i) => o.centre[i] + o.axis[i] * h + (radial[i] * o.radius) / rho);
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
  return {gap2: best.gap, x: lerp(p, q, best.t), point};
}

// For a segment reaching into a box: minus the depth of its deepest point, with that point and the nearest point of
// the box's surface. The depth along the segment is concave and piecewise linear, so its most is at an end, where a
// coordinate crosses the centre, or where two of its terms meet: every such place is tried (as utra_model.py does).
function boxDepth(o, p, q) {
  const a = o.axes.map((axis) => dot(sub(p, o.centre), axis));
  const d = o.axes.map((axis) => dot(sub(q, p), axis));
  const h = o.half;
  const candidates = [0, 1];
  for (let k = 0; k < 3; k += 1) {
    if (d[k] !== 0) candidates.push(-a[k] / d[k]);
    for (let j = 0; j < k; j += 1) {
      for (const sj of [1, -1]) {
        for (const sk of [1, -1]) {
          const den = sk * d[k] - sj * d[j];
          if (den !== 0) candidates.push(((h[k] - sk * a[k]) - (h[j] - sj * a[j])) / den);
        }
      }
    }
  }
  const depth = (t) => Math.min(h[0] - Math.abs(a[0] + d[0] * t), h[1] - Math.abs(a[1] + d[1] * t),
    h[2] - Math.abs(a[2] + d[2] * t));
  let best = null;
  let most = -Infinity;
  for (const t of candidates) {
    if (t < 0 || t > 1) continue;
    const value = depth(t);
    if (value > most) {
      best = t;
      most = value;
    }
  }
  const local = [0, 1, 2].map((k) => a[k] + d[k] * best);
  let k = 0;
  for (let i = 1; i < 3; i += 1) if (h[i] - Math.abs(local[i]) < h[k] - Math.abs(local[k])) k = i;
  const face = [...local];
  face[k] = local[k] >= 0 ? h[k] : -h[k];
  const point = [0, 1, 2].map((i) => o.centre[i] + face[0] * o.axes[0][i] + face[1] * o.axes[1][i]
    + face[2] * o.axes[2][i]);
  return {gap: -most, points: [lerp(p, q, best), point]};
}

// The point of the segment p-q where gap() is least. The distance from a convex shape, signed or not, is convex along
// a segment: a golden-section search finds its minimum.
function cylinderSegment(p, q, gap) {
  const along = (t) => gap(lerp(p, q, t));
  let low = 0;
  let high = 1;
  const ratio = (Math.sqrt(5) - 1) / 2;
  let t1 = high - ratio * (high - low);
  let t2 = low + ratio * (high - low);
  let g1 = along(t1);
  let g2 = along(t2);
  for (let n = 0; n < 40; n += 1) {
    if (g1 <= g2) {
      high = t2;
      t2 = t1;
      g2 = g1;
      t1 = high - ratio * (high - low);
      g1 = along(t1);
    } else {
      low = t1;
      t1 = t2;
      g1 = g2;
      t2 = low + ratio * (high - low);
      g2 = along(t2);
    }
  }
  let best = 0;
  let bestGap = along(0);
  for (const t of [1, 0.5 * (low + high)]) {
    const g = along(t);
    if (g < bestGap) {
      best = t;
      bestGap = g;
    }
  }
  return lerp(p, q, best);
}

// Closest points (on the segment p-q, on the obstacle).
export function nearestToSegment(o, p, q) {
  if (o.kind === "box") {
    const {x, point} = boxSegment(o, p, q);
    return [x, point];
  }
  const x = cylinderSegment(p, q, (y) => distance(y, nearest(o, y)));
  return [x, nearest(o, x)];
}

// The segment p-q's signed distance from an obstacle (m): how far apart they are, or, when the segment reaches inside,
// minus the depth of its deepest point; with the closest points (on the segment, on the obstacle's surface).
export function segmentGap(o, p, q, touch = TOUCH) {
  if (o.kind === "box") {
    const {gap2, x, point} = boxSegment(o, p, q);
    if (gap2 > touch * touch) return {gap: distance(x, point), points: [x, point]};
    return boxDepth(o, p, q);  // touching it, or reaching inside
  }
  const x = cylinderSegment(p, q, (y) => signedDistance(o, y));
  return {gap: signedDistance(o, x), points: [x, surfacePoint(o, x)]};
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
  const touch = config.touch ?? TOUCH;
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

  // The signed distance is 1-Lipschitz: no point of the segment is nearer than its middle's less half its length.
  function lowerBound(at, [c, o, fixed]) {
    const middle = lerp(at[c][0], at[c][1], 0.5);
    if (fixed) return signedDistance(obstacles[o], middle) - halfLengths[c] - capsules[c].radius;
    const other = lerp(at[o][0], at[o][1], 0.5);
    return distance(middle, other) - halfLengths[c] - capsules[c].radius - halfLengths[o] - capsules[o].radius;
  }

  function gapOf(at, [c, o, fixed]) {
    const [a, b] = at[c];
    if (fixed) {
      const {gap, points} = segmentGap(obstacles[o], a, b, touch);
      return {gap: gap - capsules[c].radius, points};
    }
    const points = segmentSegment(a, b, at[o][0], at[o][1]);
    return {gap: distance(...points) - capsules[c].radius - capsules[o].radius, points};
  }

  const names = ([c, o, fixed]) => [capsules[c].label || labels[capsules[c].body],
    fixed ? obstacles[o].label : capsules[o].label || labels[capsules[o].body]];

  // How close a pair already that near where a move begins may come: config.slack closer at most, but a part outside
  // a guarded zone (closed: an open one's pairs are left out) no further than its face (ArmModel._close_limit).
  function closeLimit([, o, fixed], gap) {
    const limit = gap - config.slack;
    return gap >= 0 && fixed && obstacles[o].guarded ? Math.max(limit, 0) : limit;
  }

  // Where a straight joint-space move first brings two things closer than the margin, or null; see
  // ArmModel.check_path for how the continuous check works (the page checks whole moves, never one in parts).
  function checkPath(start, goal, margin = config.margin) {
    const turn = start.map((s, i) => Math.abs(((goal[i] - s) * Math.PI) / 180));
    const speeds = pairs.map(([c, , , low]) => {
      let sum = 0;
      for (let j = low; j < reach[c].length; j += 1) sum += turn[j] * reach[c][j];
      return sum;
    });
    const fastest = Math.max(0, ...speeds);
    const shortest = fastest > 0 ? config.advance / fastest : 1;
    const limit = pairs.map(() => margin);
    const inside = pairs.map(() => false);  // a part already inside an obstacle at the start: a hit takes it deeper
    let t = 0;
    for (;;) {
      const pose = start.map((s, i) => s + (goal[i] - s) * t);
      const at = placed(pose);
      let step = Infinity;
      for (let p = 0; p < pairs.length; p += 1) {
        const speed = speeds[p];
        if (speed === 0 && t > 0) continue;  // nothing on this path moves these two relative to each other
        const lower = lowerBound(at, pairs[p]);
        if (limit[p] < margin && lower >= margin) {
          limit[p] = margin;  // out of the tight spot it began in: the margin holds again
          inside[p] = false;
        }
        if (lower >= limit[p] && (speed === 0 || lower - limit[p] >= step * speed)) continue;
        const {gap, points} = gapOf(at, pairs[p]);
        if (limit[p] < margin && gap >= margin) {
          limit[p] = margin;
          inside[p] = false;
        }
        if (gap < limit[p]) {
          if (t > 0) {
            const [part, other] = names(pairs[p]);
            return {part, other, obstacle: pairs[p][2] ? pairs[p][1] : null, gap, at: t, pose, points,
              inside: inside[p]};
          }
          limit[p] = closeLimit(pairs[p], gap);  // already this close: may move apart, not closer
          inside[p] = gap < 0;
        }
        if (speed > 0) step = Math.min(step, (gap - limit[p]) / speed);
      }
      if (t >= 1) return null;
      t = Math.min(1, t + Math.max(step, shortest));
    }
  }

  // The arm's gap (m) to each of `indices` (a Set of obstacle indices) at one pose, with the part of it nearest and
  // the closest points: a Map of index -> {gap, part, points}, a negative gap meaning that part is inside it.
  function gapsTo(pose, indices) {
    const at = placed(pose);
    const out = new Map();
    for (const pair of pairs) {
      const [, o, fixed] = pair;
      if (!fixed || !indices.has(o)) continue;
      const {gap, points} = gapOf(at, pair);
      if (!out.has(o) || gap < out.get(o).gap) out.set(o, {gap, part: names(pair)[0], points});
    }
    return out;
  }

  // Every checked pair's gap (m) at one pose, nearest first.
  function clearances(pose) {
    const at = placed(pose);
    return pairs.map((pair) => {
      const [part, other] = names(pair);
      return {gap: gapOf(at, pair).gap, part, other};
    }).sort((x, y) => x.gap - y.gap);
  }

  return {checkPath, clearances, gapsTo, placed, frames};
}
