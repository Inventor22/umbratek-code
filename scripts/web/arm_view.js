// 3D monitor for the jog panel: the UTRA arm built from Umbratek's URDF, standing on its cart; a shadow arm
// showing the planned pose; and, by mode, a ring at each joint that turns that joint when dragged, or a gizmo at
// the flange whose arrows and rings move the tool, the joints following by inverse kinematics (ik.js). Plans
// are checked with collide.js, the same collision check the server makes. Loaded by jog_panel.html, whose import
// map points "three" at three.js (MIT), which the jog panel server downloads on its first run and checks.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { createChecker } from "./collide.js";
import { createSolver, multiply, rotation } from "./ik.js";

const RAD = Math.PI / 180;
// Where each joint's ring sits, in that joint's frame: radius, and offset along the joint axis (metres),
// placed around each joint's own housing (from the mesh extents) so neighbouring rings don't cross.
const RINGS = [
  {radius: 0.104, offset: 0.03},
  {radius: 0.084, offset: 0.155},
  {radius: 0.074, offset: -0.03},
  {radius: 0.064, offset: 0.132},
  {radius: 0.064, offset: -0.01},
  {radius: 0.06, offset: -0.016},
];
const RING_TUBE = 0.0075;
const RING_PICK_PX = 14;  // how close the pointer must be to a ring's curve to grab it
const RING_FACING_MIN = 0.5;  // below this, the ring is too side-on to track the pointer around its plane
const RING_PX_PER_DEG = 2;  // side-on rings turn this many pixels of drag into one degree
const SNAP_DEG = 0.5;
const SNAP_COARSE_DEG = 5;  // with Shift held
const MIN_MOVE_DEG = 0.25;  // a smaller drag is a click, not a move
const SMOOTH_S = 0.08;  // the solid arm eases toward each new status frame
const PATH_STEP_DEG = 2;  // spacing of the points on the flange's path line
const VIEW = {target: [0.12, 0, 0.05], distance: 3.6, azimuth: -55, elevation: 20};
const COLOR = {
  well: 0x0e0f10, grid: 0x24272b, gridMain: 0x3a3e44, arm: 0xd9d4ca, base: 0x8f8b84,
  amber: 0xf2aa3c, alarm: 0xff8073, ink: 0xece8df, lit: 0xffffff,
  axes: [0xff3352, 0x8bdc00, 0x2890ff],  // the tool gizmo's X, Y and Z: red, green and blue, as 3D tools colour them
};
// The tool gizmo, authored in pixels and scaled to stay that size on screen at any zoom: arrows along the base
// frame's X, Y and Z from just off the flange, and a ring about each.
const GIZMO = {shaft: [16, 74], shaftRadius: 1.5, head: [74, 94], headRadius: 5.5, ring: 58, tube: 1.4,
               rim: 1.7, label: 108, letterPx: 18, ringName: 76, ringNamePx: 13, pick: 12, far: 0.35, reach: 122};
const TOOL_SNAP = {move: [1, 10], turn: [0.5, 5]};  // mm or degrees; the second with Shift held
const TOOL_CHUNK = {move: 10, turn: 3};  // the solver follows a drag in steps no longer than this
const BASE_AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

function vector(text, fallback = "0 0 0") {
  const [x, y, z] = (text || fallback).trim().split(/\s+/).map(Number);
  return new THREE.Vector3(x || 0, y || 0, z || 0);
}

function origin(element) {
  const rpy = vector(element && element.getAttribute("rpy"));
  return {
    position: vector(element && element.getAttribute("xyz")),
    // URDF roll-pitch-yaw are fixed-axis X, Y, Z rotations: R = Rz(yaw) Ry(pitch) Rx(roll).
    quaternion: new THREE.Quaternion().setFromEuler(new THREE.Euler(rpy.x, rpy.y, rpy.z, "ZYX")),
  };
}

function parseUrdf(text) {
  const doc = new DOMParser().parseFromString(text, "application/xml");
  if (doc.querySelector("parsererror")) throw new Error("the URDF isn't valid XML");
  const links = new Map();
  for (const link of doc.querySelectorAll("robot > link")) {
    const mesh = link.querySelector("visual > geometry > mesh");
    links.set(link.getAttribute("name"), mesh && {
      file: mesh.getAttribute("filename").replace(/\\/g, "/").split("/").pop(),
      scale: vector(mesh.getAttribute("scale"), "1 1 1"),
      origin: origin(link.querySelector("visual > origin")),
    });
  }
  const joints = [];
  let base = null;
  for (const joint of doc.querySelectorAll("robot > joint")) {
    if (!["revolute", "continuous"].includes(joint.getAttribute("type"))) continue;
    const parent = joint.querySelector("parent").getAttribute("link");
    if (joints.length && parent !== joints[joints.length - 1].child) throw new Error("the URDF's joints don't form one chain");
    base = base || parent;
    const axis = joint.querySelector("axis");
    joints.push({
      child: joint.querySelector("child").getAttribute("link"),
      origin: origin(joint.querySelector("origin")),
      axis: vector(axis && axis.getAttribute("xyz"), "1 0 0").normalize(),
    });
  }
  return {links, joints, base};
}

// One copy of the arm: a frame per joint (fixed to its parent link) holding a rotor that turns. Each mesh
// records which joint moves it (-1 for the base).
function buildArm(model, geometries, linkMaterial, baseMaterial) {
  const root = new THREE.Group();
  const meshes = [];
  const attach = (parent, linkName, material, joint) => {
    const visual = model.links.get(linkName);
    if (!visual || !material) return;
    const mesh = new THREE.Mesh(geometries.get(visual.file), material);
    mesh.position.copy(visual.origin.position);
    mesh.quaternion.copy(visual.origin.quaternion);
    mesh.scale.copy(visual.scale);
    mesh.userData.joint = joint;
    parent.add(mesh);
    meshes.push(mesh);
  };
  attach(root, model.base, baseMaterial, -1);
  const frames = [];
  const rotors = [];
  let parent = root;
  model.joints.forEach((joint, i) => {
    const frame = new THREE.Object3D();
    frame.position.copy(joint.origin.position);
    frame.quaternion.copy(joint.origin.quaternion);
    const rotor = new THREE.Object3D();
    frame.add(rotor);
    parent.add(frame);
    attach(rotor, joint.child, linkMaterial, i);
    frames.push(frame);
    rotors.push(rotor);
    parent = rotor;
  });
  const pose = (degrees) => {
    degrees.forEach((q, i) => rotors[i].quaternion.setFromAxisAngle(model.joints[i].axis, q * RAD));
    root.updateMatrixWorld(true);
  };
  return {root, frames, rotors, meshes, pose};
}

export async function createArmView(host, options) {
  const {modelBase, urdfFile, tapes, limits, cart, floorZ = 0} = options;
  let {collision} = options;
  const noop = () => {};
  const onPreview = options.onPreview || noop;
  const onRelease = options.onRelease || noop;
  let checker = createChecker(collision);
  let solver = createSolver(collision);
  let mode = options.mode === "tool" ? "tool" : "joints";  // what dragging in the view moves

  const urdfResponse = await fetch(modelBase + urdfFile, {cache: "no-store"});
  if (!urdfResponse.ok) throw new Error(`couldn't load the URDF (HTTP ${urdfResponse.status})`);
  const model = parseUrdf(await urdfResponse.text());
  const loader = new STLLoader();
  const files = [...new Set([...model.links.values()].filter(Boolean).map((v) => v.file))];
  const [geometries, cartParts] = await Promise.all([
    Promise.all(files.map(async (file) => {
      const geometry = await loader.loadAsync(modelBase + "meshes/" + file);
      geometry.computeBoundingBox();
      return [file, geometry];
    })).then((entries) => new Map(entries)),
    Promise.all((cart ? cart.visuals : []).map(async (visual) => ({
      geometry: await loader.loadAsync(cart.base + visual.file), color: visual.color,
    }))),
  ]);

  // ---- Scene ----
  const renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: "high-performance"});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.domElement.className = "screen-canvas";
  host.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(COLOR.well);
  scene.fog = new THREE.Fog(COLOR.well, 5, 11);
  const camera = new THREE.PerspectiveCamera(34, 1, 0.02, 40);
  camera.up.set(0, 0, 1);  // the URDF is z-up; keep it that way

  scene.add(new THREE.HemisphereLight(0xf3efe6, 0x1b1d20, 1.5));
  const key = new THREE.DirectionalLight(0xffffff, 2.4);
  key.position.set(1.4, -1.1, 2.6);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, {left: -1.5, right: 1.5, top: 1.5, bottom: -1.5, near: 0.5, far: 7});
  key.shadow.camera.updateProjectionMatrix();
  key.shadow.bias = -0.0004;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xffffff, 0.7);
  rim.position.set(-1.6, 1.8, 1.2);
  scene.add(rim);

  // The floor the cart stands on: a grid and soft contact shadows. The shadow catcher doesn't write depth,
  // so a shadow arm that would dip into the floor still shows.
  const grid = new THREE.GridHelper(3.2, 32, COLOR.gridMain, COLOR.grid);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = floorZ;
  scene.add(grid);
  const shadowFloor = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.ShadowMaterial({opacity: 0.38, depthWrite: false}));
  shadowFloor.position.z = floorZ + 0.0005;
  shadowFloor.receiveShadow = true;
  scene.add(shadowFloor);

  // The cart, in the colours its URDF gives, drawn closer to a mid grey: the frame recedes so the arm stays
  // the subject, and the dark controller and casters still read against the dark well.
  const cartGroup = new THREE.Group();
  const cartTone = new THREE.Color(0x5c6068);
  for (const part of cartParts) {
    const [r, g, b] = part.color;
    const color = new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace).lerp(cartTone, 0.4);
    const mesh = new THREE.Mesh(part.geometry, new THREE.MeshStandardMaterial({color, roughness: 0.6, metalness: 0.2}));
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    cartGroup.add(mesh);
  }
  scene.add(cartGroup);

  // Where a blocked plan first comes too close: a flat mark of fixed size on screen, an Alarm Red core in a hard
  // Well Black rim, drawn over everything. The part it would come too close to gets a faint red overlay.
  const MARK_PX = 14;
  const markCanvas = document.createElement("canvas");
  markCanvas.width = markCanvas.height = 64;
  const pen = markCanvas.getContext("2d");
  pen.fillStyle = "#0e0f10";
  pen.beginPath();
  pen.arc(32, 32, 31, 0, Math.PI * 2);
  pen.fill();
  pen.fillStyle = "#ff8073";
  pen.beginPath();
  pen.arc(32, 32, 31 - (64 * 2) / MARK_PX, 0, Math.PI * 2);
  pen.fill();
  const markTexture = new THREE.CanvasTexture(markCanvas);
  markTexture.colorSpace = THREE.SRGBColorSpace;
  const contactMark = new THREE.Sprite(new THREE.SpriteMaterial({
    map: markTexture, sizeAttenuation: false, depthTest: false, depthWrite: false, transparent: true, fog: false,
  }));
  contactMark.renderOrder = 12;
  contactMark.visible = false;
  scene.add(contactMark);
  const hitFill = new THREE.MeshBasicMaterial({color: COLOR.alarm, transparent: true, opacity: 0.15, depthWrite: false});
  const hitEdge = new THREE.LineBasicMaterial({color: COLOR.alarm, transparent: true, opacity: 0.9});
  const highlights = new Map();  // obstacle index -> its red overlay, made on first use
  function highlight(index) {
    const o = Number.isInteger(index) ? collision.obstacles[index] : null;
    // The floor is a 6 m slab: the contact mark alone says enough there.
    const show = o && (o.kind !== "box" || Math.max(...o.half) < 1) ? index : null;
    if (show !== null && !highlights.has(show)) {
      const geometry = o.kind === "box"
        ? new THREE.BoxGeometry(o.half[0] * 2, o.half[1] * 2, o.half[2] * 2)
        : new THREE.CylinderGeometry(o.radius, o.radius, o.halfLength * 2, 48).rotateX(Math.PI / 2);  // along z
      const overlay = new THREE.Mesh(geometry, hitFill);
      overlay.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry, 30), hitEdge));
      overlay.position.set(...o.centre);
      if (o.kind === "box") {
        const [x, y, z] = o.axes.map((axis) => new THREE.Vector3(...axis));
        overlay.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
      } else {
        overlay.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(...o.axis));
      }
      overlay.scale.setScalar(1.002);  // just outside the part, so it doesn't flicker against it
      overlay.renderOrder = 3;
      scene.add(overlay);
      highlights.set(show, overlay);
    }
    for (const [i, overlay] of highlights) overlay.visible = i === show;
  }

  const armMaterial = new THREE.MeshStandardMaterial({color: COLOR.arm, roughness: 0.5, metalness: 0.08});
  const baseMaterial = new THREE.MeshStandardMaterial({color: COLOR.base, roughness: 0.6, metalness: 0.1});
  const ghostMaterial = (color) => new THREE.MeshStandardMaterial({
    color, emissive: color, emissiveIntensity: 0.6, roughness: 0.8, metalness: 0,
    transparent: true, opacity: 0.42, depthWrite: false,
  });
  const ghostOk = ghostMaterial(COLOR.amber);
  const ghostBlocked = ghostMaterial(COLOR.alarm);
  // Depth-only pass first, so the translucent shadow arm shows just its front skin, not every inner face.
  const ghostDepth = new THREE.MeshBasicMaterial({colorWrite: false, transparent: true});

  const actual = buildArm(model, geometries, armMaterial, baseMaterial);
  for (const mesh of actual.meshes) mesh.castShadow = true;
  actual.root.visible = false;  // until the first status arrives
  scene.add(actual.root);
  const ghost = buildArm(model, geometries, ghostOk, null);
  for (const mesh of ghost.meshes) {
    mesh.renderOrder = 2;
    const depth = new THREE.Mesh(mesh.geometry, ghostDepth);
    depth.renderOrder = 1;
    mesh.add(depth);
  }
  ghost.root.visible = false;
  scene.add(ghost.root);
  const probe = buildArm(model, geometries, null, null);  // for the flange's path; never drawn

  // The cell's zones, frames and targets (setCell), drawn in neutral tones.
  const cellGroup = new THREE.Group();
  scene.add(cellGroup);
  const zoneFill = new THREE.MeshBasicMaterial({color: 0xaaa69d, transparent: true, opacity: 0.07, depthWrite: false});
  const zoneEdge = new THREE.LineBasicMaterial({color: 0x8f8b83, transparent: true, opacity: 0.8});
  const frameInk = new THREE.LineBasicMaterial({color: COLOR.ink, depthTest: false, transparent: true});
  const frameGrey = new THREE.LineBasicMaterial({color: 0x8f8b83, depthTest: false, transparent: true});
  const dotCanvas = document.createElement("canvas");
  dotCanvas.width = dotCanvas.height = 32;
  const dotPen = dotCanvas.getContext("2d");
  dotPen.fillStyle = "#0e0f10";
  dotPen.beginPath();
  dotPen.arc(16, 16, 15, 0, Math.PI * 2);
  dotPen.fill();
  dotPen.fillStyle = "#ece8df";
  dotPen.beginPath();
  dotPen.arc(16, 16, 10, 0, Math.PI * 2);
  dotPen.fill();
  const dotTexture = new THREE.CanvasTexture(dotCanvas);
  dotTexture.colorSpace = THREE.SRGBColorSpace;
  const targetDot = new THREE.SpriteMaterial({map: dotTexture, depthTest: false, depthWrite: false, transparent: true,
                                              fog: false, sizeAttenuation: true});
  // A guarded zone (one its interlocks open) is edged with dashes, like a door's outline; while open it loses its fill.
  const zoneDash = new THREE.LineDashedMaterial({color: 0x8f8b83, transparent: true, opacity: 0.8, dashSize: 0.012,
                                                 gapSize: 0.008});
  const zoneOpen = new THREE.MeshBasicMaterial({visible: false});
  // Where a touched-up target was: a hollow ring in Scale ink, joined to where it is now by a faint line.
  const ringCanvas = document.createElement("canvas");
  ringCanvas.width = ringCanvas.height = 32;
  const ringPen = ringCanvas.getContext("2d");
  ringPen.lineWidth = 5;
  ringPen.strokeStyle = "#0e0f10";
  ringPen.beginPath();
  ringPen.arc(16, 16, 11, 0, Math.PI * 2);
  ringPen.stroke();
  ringPen.lineWidth = 2.5;
  ringPen.strokeStyle = "#8f8b83";
  ringPen.stroke();
  const ringTexture = new THREE.CanvasTexture(ringCanvas);
  ringTexture.colorSpace = THREE.SRGBColorSpace;
  const wasRing = new THREE.SpriteMaterial({map: ringTexture, depthTest: false, depthWrite: false, transparent: true,
                                            fog: false, sizeAttenuation: true});
  const wasLine = new THREE.LineBasicMaterial({color: 0x8f8b83, depthTest: false, transparent: true, opacity: 0.7});
  // A pattern's slots: smaller dots in Scale ink.
  const slotDot = new THREE.SpriteMaterial({map: dotTexture, color: 0x8f8b83, depthTest: false, depthWrite: false,
                                            transparent: true, opacity: 0.75, fog: false, sizeAttenuation: true});
  // A frame's name, printed small in Printed Grey over a Well Black outline, a constant size on screen.
  function textSprite(text, color) {
    const canvas = document.createElement("canvas");
    const pen = canvas.getContext("2d");
    const font = "600 26px 'Segoe UI Variable Text', 'Segoe UI', system-ui, sans-serif";
    pen.font = font;
    canvas.width = Math.ceil(pen.measureText(text.toUpperCase()).width) + 12;
    canvas.height = 40;
    pen.font = font;
    pen.textBaseline = "middle";
    pen.lineWidth = 6;
    pen.strokeStyle = "#0e0f10";
    pen.strokeText(text.toUpperCase(), 6, 20);
    pen.fillStyle = color;
    pen.fillText(text.toUpperCase(), 6, 20);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map: texture, depthTest: false, depthWrite: false,
                                                              transparent: true, fog: false, sizeAttenuation: false}));
    sprite.scale.set((canvas.width / canvas.height) * 0.022, 0.022, 1);
    sprite.center.set(0, 0.5);
    sprite.renderOrder = 10;
    return sprite;
  }

  // A straight joint-space move from `from` to `to`, checked for collisions as the server checks it, with
  // the flange's path for drawing, and how far the flange travels for each degree the lead joint turns, at its
  // most (mm/deg): the move turns every joint in step, so that times the lead joint's speed is the flange's.
  function checkPath(from, to) {
    const hit = inClosed(from) || checker.checkPath(from, to);
    const span = Math.max(...from.map((q, i) => Math.abs(to[i] - q)));
    const steps = Math.max(1, Math.ceil(span / PATH_STEP_DEG));
    const points = [];
    for (let k = 0; k <= steps; k += 1) {
      probe.pose(from.map((start, i) => start + ((to[i] - start) * k) / steps));
      points.push(new THREE.Vector3().setFromMatrixPosition(probe.rotors[probe.rotors.length - 1].matrixWorld));
    }
    let flangePerDeg = 0;
    for (let k = 1; k < points.length && span > 0; k += 1) {
      flangePerDeg = Math.max(flangePerDeg, (points[k].distanceTo(points[k - 1]) * 1000 * steps) / span);
    }
    return {clear: !hit, hit, points, flangePerDeg};
  }

  // As the server has it: with the arm in a guarded zone that's closed (its pairs are in the check; an open one's
  // aren't), nothing moves it there, so every plan from there is blocked (the zone opens, or the arm is guided out
  // by hand in freedrive).
  function inClosed(from) {
    const closed = new Set(collision.obstacles.flatMap((o, i) => (o.guard ? [i] : [])));
    if (!closed.size) return null;
    for (const [i, g] of checker.gapsTo(from, closed)) {
      if (g.gap < 0) {
        return {part: g.part, other: collision.obstacles[i].label, obstacle: i, gap: g.gap, at: 0, pose: from,
          points: g.points, staying: true};
      }
    }
    return null;
  }

  // ---- Joint rings ----
  const zAxis = new THREE.Vector3(0, 0, 1);
  const rings = model.joints.map((joint, i) => {
    const spec = RINGS[i] || {radius: 0.07, offset: 0};
    const group = new THREE.Group();
    group.quaternion.setFromUnitVectors(zAxis, joint.axis);
    group.position.copy(joint.axis).multiplyScalar(spec.offset);
    const color = new THREE.Color(tapes[i] || "#dad5c9");
    const geometry = new THREE.TorusGeometry(spec.radius, RING_TUBE, 10, 120);
    const front = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({color, transparent: true, opacity: 1}));
    const behind = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
      color, transparent: true, opacity: 0.24, depthTest: false, depthWrite: false,
    }));
    behind.renderOrder = 10;
    const band = new THREE.Mesh(new THREE.TorusGeometry(spec.radius, 0.016, 8, 64), new THREE.MeshBasicMaterial({
      color, transparent: true, opacity: 0.2, depthWrite: false,
    }));
    band.visible = false;  // hover highlight
    const sweep = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
      color: COLOR.amber, transparent: true, opacity: 0.95, depthTest: false, depthWrite: false,
    }));
    sweep.renderOrder = 12;
    sweep.visible = false;
    group.add(front, behind, band, sweep);
    ghost.frames[i].add(group);  // the rings ride on the shadow arm, so dragging sculpts the plan
    return {group, front, behind, band, sweep, color, radius: spec.radius, joint: i};
  });

  const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({
    color: COLOR.amber, transparent: true, opacity: 0.9, depthTest: false,
  }));
  pathLine.renderOrder = 11;
  pathLine.visible = false;
  scene.add(pathLine);
  for (const ring of rings) ring.group.visible = mode === "joints";

  // ---- Tool gizmo ----
  // Each axis in its colour, X red, Y green and Z blue, as in other 3D tools, and still named by printed letters, so
  // colour is never the only cue. A Well Black rim makes it read over the light arm and the dark screen alike, and
  // it's drawn over everything, like the path. A dragged arrow lights amber, like a held key, and a turned ring
  // shows its turn in amber.
  const gizmo = new THREE.Group();
  gizmo.visible = false;
  scene.add(gizmo);
  const flat = (color, opacity) => new THREE.MeshBasicMaterial({
    color, transparent: true, opacity, depthTest: false, depthWrite: false, fog: false,
  });
  const yAxis = new THREE.Vector3(0, 1, 0);
  // A printed legend that faces the camera: bold, in `fill`, with a 2px Well Black outline, `px` tall on screen.
  function letter(text, px, fill) {
    const scale = 4;  // canvas pixels per screen pixel, crisp on 2x screens
    const font = `700 ${px * scale}px 'Segoe UI Variable Text', 'Segoe UI', system-ui, sans-serif`;
    const canvas = document.createElement("canvas");
    let pen = canvas.getContext("2d");
    pen.font = font;
    canvas.width = Math.ceil(pen.measureText(text).width + px * scale * 0.7);
    canvas.height = Math.ceil(px * scale * 1.5);
    pen = canvas.getContext("2d");  // resizing resets the context's state
    pen.font = font;
    pen.textAlign = "center";
    pen.textBaseline = "middle";
    pen.lineJoin = "round";
    pen.lineWidth = 4 * scale;
    pen.strokeStyle = "#0e0f10";
    pen.strokeText(text, canvas.width / 2, canvas.height / 2);
    pen.fillStyle = fill;
    pen.fillText(text, canvas.width / 2, canvas.height / 2);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: texture, transparent: true, depthTest: false, depthWrite: false, fog: false,
    }));
    sprite.scale.set(canvas.width / scale, canvas.height / scale, 1);
    sprite.renderOrder = 16;
    return sprite;
  }
  const handles = [];
  BASE_AXES.forEach((direction, k) => {
    const axis = new THREE.Vector3(...direction);
    const name = ["X", "Y", "Z"][k];
    const tone = COLOR.axes[k];
    const print = `#${tone.toString(16).padStart(6, "0")}`;
    // Arrow: a shaft and a head along +axis, each over a slightly larger rim.
    const arrow = new THREE.Group();
    arrow.quaternion.setFromUnitVectors(yAxis, axis);
    const core = flat(tone, 1);
    const rim = flat(COLOR.well, 0.9);
    const band = flat(tone, 0.25);
    band.visible = false;  // hover highlight
    const [s0, s1] = GIZMO.shaft;
    const [h0, h1] = GIZMO.head;
    const shaft = (r) => new THREE.CylinderGeometry(r, r, s1 - s0, 12).translate(0, (s0 + s1) / 2, 0);
    const head = (r, grow) => new THREE.ConeGeometry(r, h1 - h0 + 2 * grow, 24).translate(0, (h0 + h1) / 2, 0);
    for (const [geometry, material, order] of [[shaft(GIZMO.shaftRadius + GIZMO.rim), rim, 13],
      [head(GIZMO.headRadius + GIZMO.rim, GIZMO.rim), rim, 13], [shaft(GIZMO.shaftRadius), core, 14],
      [head(GIZMO.headRadius, 0), core, 14], [shaft(6), band, 12]]) {
      const mesh = new THREE.Mesh(geometry, material);
      mesh.renderOrder = order;
      arrow.add(mesh);
    }
    const label = letter(name, GIZMO.letterPx, print);
    label.position.copy(axis).multiplyScalar(GIZMO.label);
    gizmo.add(arrow, label);
    handles.push({type: "move", axis: k, name, tone, core, rim, band, label});

    // Ring about the axis, in the plane of the next two axes (u x v = axis, so positive angles turn
    // right-handed about it). It's drawn as two halves so the half facing away from the camera can be dimmed,
    // which gives the cage its near and far. A drag shows its turn as a translucent amber wedge edged by an arc.
    const u = new THREE.Vector3(...BASE_AXES[(k + 1) % 3]);
    const v = new THREE.Vector3(...BASE_AXES[(k + 2) % 3]);
    const group = new THREE.Group();
    group.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(u, v, axis));
    const ringCore = flat(tone, 1);
    const ringRim = flat(COLOR.well, 0.9);
    const farCore = flat(tone, GIZMO.far);
    const farRim = flat(COLOR.well, 0.9 * GIZMO.far);
    const ringBand = flat(tone, 0.25);
    ringBand.visible = false;
    const half = (tube) => new THREE.TorusGeometry(GIZMO.ring, tube, 8, 80, Math.PI);
    const near = new THREE.Group();
    const far = new THREE.Group();
    for (const [side, coreMaterial, rimMaterial] of [[near, ringCore, ringRim], [far, farCore, farRim]]) {
      const rimMesh = new THREE.Mesh(half(GIZMO.tube + GIZMO.rim), rimMaterial);
      const coreMesh = new THREE.Mesh(half(GIZMO.tube), coreMaterial);
      rimMesh.renderOrder = 13;
      coreMesh.renderOrder = 14;
      side.add(rimMesh, coreMesh);
    }
    const bandMesh = new THREE.Mesh(new THREE.TorusGeometry(GIZMO.ring, 6, 8, 160), ringBand);
    bandMesh.renderOrder = 12;
    const wedge = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
      color: COLOR.amber, transparent: true, opacity: 0.45, depthTest: false, depthWrite: false, fog: false,
      side: THREE.DoubleSide,
    }));
    wedge.renderOrder = 12;
    wedge.visible = false;
    const sweep = new THREE.Mesh(new THREE.BufferGeometry(), flat(COLOR.amber, 0.95));
    sweep.renderOrder = 15;
    sweep.visible = false;
    group.add(near, far, bandMesh, wedge, sweep);
    // Its printed name, just outside the ring at one of its diagonals (orientRings picks which, frame by frame).
    const ringName = letter(`R${name.toLowerCase()}`, GIZMO.ringNamePx, print);
    ringName.position.copy(u).add(v).normalize().multiplyScalar(GIZMO.ringName);
    gizmo.add(group, ringName);
    handles.push({type: "turn", axis: k, name: `R${name.toLowerCase()}`, tone, core: ringCore, rim: ringRim, band: ringBand,
                  farCore, farRim, near, far, group, wedge, sweep, u, v, label: ringName});
  });

  // Turns each ring's dimmed half to face away from the camera (a ring seen face-on has no far half, so it's drawn
  // whole), and sets each ring's name at whichever of its four diagonals reads best from here: on the ring's near
  // side, clear of the arrows, their letters and the names already set. A name holds its place until another is
  // clearly better, so the names don't flicker as the view turns. Needs the gizmo's world matrix up to date.
  const NAME_SPOTS = [[1, 1], [-1, 1], [-1, -1], [1, -1]];
  function orientRings() {
    const toCamera = camera.position.clone().sub(gizmo.position).normalize();
    const width = renderer.domElement.clientWidth || host.clientWidth;
    const height = renderer.domElement.clientHeight || host.clientHeight;
    const pixel = (local) => {
      const p = local.clone().applyMatrix4(gizmo.matrixWorld).project(camera);
      return new THREE.Vector2(((p.x + 1) / 2) * width, ((1 - p.y) / 2) * height);
    };
    const centre = pixel(new THREE.Vector3());
    const taken = [];  // each arrow's shaft, head and letter, then each name as it's set
    for (const direction of BASE_AXES) {
      for (const out of [45, 84, GIZMO.label]) taken.push(pixel(new THREE.Vector3(...direction).multiplyScalar(out)));
    }
    for (const k of [2, 0, 1]) {
      const handle = handles.find((h) => h.type === "turn" && h.axis === k);
      const a = toCamera.dot(handle.u);
      const b = toCamera.dot(handle.v);
      const away = Math.atan2(-b, -a);
      handle.far.rotation.z = away - Math.PI / 2;
      handle.near.rotation.z = away + Math.PI / 2;
      const fade = 1 - (1 - GIZMO.far) * Math.min(1, Math.hypot(a, b) / 0.35);
      handle.farCore.color.copy(handle.core.color);
      handle.farCore.opacity = handle.core.opacity * fade;
      handle.farRim.opacity = handle.rim.opacity * fade;
      let best = null;
      NAME_SPOTS.forEach(([s, t], spot) => {
        const direction = handle.u.clone().multiplyScalar(s).addScaledVector(handle.v, t).normalize();
        const at = pixel(direction.clone().multiplyScalar(GIZMO.ringName));
        let score = Math.min(48, ...taken.map((q) => q.distanceTo(at)));  // px of room, enough at 48
        if (direction.dot(toCamera) > 0.05) score += 12;  // against its own bright near arc
        if (at.distanceTo(centre) < GIZMO.ring) score -= 24;  // seen edge-on: inside the cage, over the other rings
        if (spot === handle.spot) score += 8;
        if (!best || score > best.score) best = {spot, score, direction, at};
      });
      handle.spot = best.spot;
      handle.label.position.copy(best.direction).multiplyScalar(GIZMO.ringName);
      taken.push(best.at);
    }
  }

  // Keeps the gizmo the same size on screen: one unit of its geometry is one pixel at its depth. Also turns the
  // rings' dimmed halves away from the camera and sets their names.
  function sizeGizmo() {
    const height = Math.max(1, renderer.domElement.clientHeight || host.clientHeight);
    const depth = gizmo.position.clone().sub(camera.position).dot(camera.getWorldDirection(new THREE.Vector3()));
    gizmo.scale.setScalar((2 * Math.max(depth, camera.near) * Math.tan((camera.fov * RAD) / 2)) / height);
    gizmo.updateMatrixWorld(true);
    orientRings();
    gizmo.updateMatrixWorld(true);
  }

  // ---- Camera ----
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.minDistance = 0.5;
  controls.maxDistance = 7;
  controls.maxPolarAngle = Math.PI * 0.56;
  let userMoved = false;  // once the user turns or zooms the view, a resize keeps their framing
  controls.addEventListener("start", () => { userMoved = true; });
  // Margins (px) the arm and cart are framed within: clear of the edges and of the plan bar, so the picture
  // doesn't jump when a plan appears. On a tall screen the bar docks along the bottom (a short one gives up
  // at most 30% of itself to it, or the bar's own height when it's showing and taller, up to half); on a wide,
  // short one, like the stacked side panel, the page docks it at the right (see WIDE) and the picture takes the
  // full height.
  const INSET = {side: 14, top: 14, bottom: 80, wideRight: 272};
  const WIDE = {ratio: 1.6, minWidth: 640};
  let wide = false;
  const planBar = () => host.querySelector(".plan");
  function frameWindow() {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    const bar = planBar();
    const barSpace = bar && !bar.hidden && !wide ? bar.offsetHeight + 18 : 0;  // docked 10px up, plus air
    const bottom = wide ? INSET.top : Math.max(Math.min(INSET.bottom, height * 0.3), Math.min(barSpace, height * 0.5));
    const right = wide ? INSET.wideRight : INSET.side;
    return {left: -1 + (2 * INSET.side) / width, right: 1 - (2 * right) / width,
            bottom: -1 + (2 * bottom) / height, top: 1 - (2 * INSET.top) / height};
  }
  // A thinned sample of the arm's and cart's mesh vertices in world space: framing fits the real shapes,
  // not their boxes (the cart's box corners are empty floor).
  function framePoints() {
    cartGroup.updateMatrixWorld(true);
    const points = [];
    for (const mesh of [...actual.meshes, ...cartGroup.children]) {
      const position = mesh.geometry.attributes.position;
      const stride = Math.max(1, Math.floor(position.count / 1500));
      for (let i = 0; i < position.count; i += stride) {
        points.push(new THREE.Vector3().fromBufferAttribute(position, i).applyMatrix4(mesh.matrixWorld));
      }
    }
    return points;
  }
  function resetView() {
    const az = VIEW.azimuth * RAD;
    const el = VIEW.elevation * RAD;
    const toward = new THREE.Vector3(Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el));
    let centre = new THREE.Vector3(...VIEW.target);
    let distance = VIEW.distance / Math.min(1, camera.aspect || 1) ** 0.75;
    if (actual.root.visible) {
      // Frame the arm as it stands now, on its cart, so it fills the screen whatever the screen's shape; in Tool
      // mode, with room for the gizmo around the flange of the shadow. The gizmo is a fixed size on screen, so its
      // margin is in pixels, and never more than 45% of the frame, so the fit always settles.
      const points = framePoints();
      const FRAME = frameWindow();
      const flange = mode === "tool" ? new THREE.Vector3().setFromMatrixPosition(ghost.rotors[ghost.rotors.length - 1].matrixWorld) : null;
      const margin = [Math.min((2 * GIZMO.reach) / Math.max(1, host.clientWidth), 0.45 * (FRAME.right - FRAME.left)),
                      Math.min((2 * GIZMO.reach) / Math.max(1, host.clientHeight), 0.45 * (FRAME.top - FRAME.bottom))];
      centre = new THREE.Box3().setFromPoints(points).getCenter(new THREE.Vector3());
      const p = new THREE.Vector3();
      const right = new THREE.Vector3();
      const up = new THREE.Vector3();
      for (let k = 0; k < 8; k += 1) {
        camera.position.copy(centre).addScaledVector(toward, distance);
        camera.lookAt(centre);
        camera.updateMatrixWorld(true);
        let [x0, x1, y0, y1] = [Infinity, -Infinity, Infinity, -Infinity];
        for (const point of points) {
          p.copy(point).project(camera);
          x0 = Math.min(x0, p.x);
          x1 = Math.max(x1, p.x);
          y0 = Math.min(y0, p.y);
          y1 = Math.max(y1, p.y);
        }
        if (flange) {
          p.copy(flange).project(camera);
          x0 = Math.min(x0, p.x - margin[0]);
          x1 = Math.max(x1, p.x + margin[0]);
          y0 = Math.min(y0, p.y - margin[1]);
          y1 = Math.max(y1, p.y + margin[1]);
        }
        // Slide the view so the picture sits in the middle of FRAME, then zoom so it fills it.
        const halfHeight = distance * Math.tan((camera.fov * RAD) / 2);
        right.setFromMatrixColumn(camera.matrixWorld, 0);
        up.setFromMatrixColumn(camera.matrixWorld, 1);
        centre.addScaledVector(right, ((x0 + x1 - FRAME.left - FRAME.right) / 2) * halfHeight * camera.aspect)
          .addScaledVector(up, ((y0 + y1 - FRAME.bottom - FRAME.top) / 2) * halfHeight);
        const scale = Math.max((x1 - x0) / (FRAME.right - FRAME.left), (y1 - y0) / (FRAME.top - FRAME.bottom));
        distance = Math.max(0.6, distance * scale);
      }
    }
    controls.target.copy(centre);
    camera.position.copy(centre).addScaledVector(toward, distance);
    controls.update();
    userMoved = false;
    requestRender();
  }

  // ---- State ----
  const count = model.joints.length;
  let target = null;  // latest measured joints (deg)
  let shown = null;  // what the solid arm shows, easing toward `target`
  let goal = null;  // the server's commanded goal while the arm is moving
  let plan = null;  // staged targets per joint (deg, or null to stay with the real arm), built by dragging
  let fault = null;  // where a faulted program step would have collided (setFault)
  let planCheck = {clear: true, hit: null, points: null, flangePerDeg: null};
  let planKey = "";  // what planCheck was worked out for
  let lastPlan = "";  // the staged plan last checked against the plan bar
  let drag = null;  // a ring or gizmo drag in progress
  let hover = null;  // the joint whose ring the pointer is over
  let hoverHandle = null;  // the gizmo handle the pointer is over
  let interactive = false;
  let dirty = true;
  let disposed = false;
  const requestRender = () => { dirty = true; };
  controls.addEventListener("change", requestRender);

  const staged = (targets) => Boolean(targets) && targets.some((q) => Number.isFinite(q));
  // The real arm with the staged joints swapped in.
  const withPlan = (base, targets) => base.map((q, i) => (targets && Number.isFinite(targets[i]) ? targets[i] : q));

  let pathPoints = null;
  function showPath(points, clear) {
    pathLine.visible = Boolean(points);
    if (points && points !== pathPoints) {
      pathLine.geometry.dispose();
      pathLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
    }
    pathPoints = points;
    pathLine.material.color.setHex(clear ? COLOR.amber : COLOR.alarm);
  }

  // Pose the shadow arm, and the rings riding on it: a drag in progress, else the staged plan, else the
  // goal of a move under way, else the real arm. Only links that differ from the real arm are drawn.
  function updateGhost() {
    if (!shown) return;
    let pose = shown;
    let check = null;
    if (drag) {
      pose = drag.pose;
      check = drag.check;
    } else if (plan) {
      pose = withPlan(shown, plan);
      check = planCheck;
    } else if (fault) {
      pose = fault.pose;
      check = {clear: false, hit: fault, points: null};
    } else if (goal) {
      pose = goal;
    }
    const clear = !check || check.clear;
    let first = pose.findIndex((q, i) => Math.abs(q - shown[i]) > 0.05);
    if (first < 0) first = pose.length;
    for (const mesh of ghost.meshes) {
      mesh.material = clear ? ghostOk : ghostBlocked;
      mesh.visible = mesh.userData.joint >= first;
    }
    ghost.pose(pose);
    gizmo.position.setFromMatrixPosition(ghost.rotors[ghost.rotors.length - 1].matrixWorld);
    gizmo.visible = mode === "tool";
    const hit = check && !check.clear ? check.hit : null;
    contactMark.visible = Boolean(hit);
    if (hit) {
      const [a, b] = hit.points;
      contactMark.position.set((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2);
    }
    highlight(hit ? hit.obstacle : null);
    showPath(check && check.points, clear);
    requestRender();
  }

  function styleRings() {
    for (const ring of rings) {
      const active = drag ? drag.joint === ring.joint : hover === ring.joint;
      const dim = drag ? drag.joint !== ring.joint : !interactive;
      ring.front.material.opacity = dim ? 0.3 : 1;
      ring.behind.material.opacity = dim ? 0.08 : 0.24;
      ring.front.material.color.copy(ring.color).lerp(new THREE.Color(0xffffff), active ? 0.35 : 0);
      ring.band.visible = (interactive || Boolean(drag)) && active;
    }
    // The gizmo: each handle in its axis colour, the hovered one brightened toward white, the rest dimmed while a drag
    // is on. A dragged arrow lights amber, like a held key; a dragged ring lights white, so the amber wedge and arc of
    // its turn stand out on it.
    for (const handle of handles) {
      const dragging = Boolean(drag && drag.kind === "tool");
      const active = dragging ? drag.handle === handle : hoverHandle === handle;
      const dim = dragging ? !active : !interactive;
      if (dragging && active) handle.core.color.setHex(handle.type === "move" ? COLOR.amber : COLOR.lit);
      else handle.core.color.setHex(handle.tone).lerp(new THREE.Color(COLOR.lit), active ? 0.35 : 0);
      handle.core.opacity = dim ? 0.3 : 1;
      handle.rim.opacity = dim ? 0.3 : 0.9;
      handle.band.visible = active && (interactive || dragging);
      if (handle.label) handle.label.material.opacity = dim ? 0.35 : 1;
    }
    const over = mode === "tool" ? hoverHandle !== null : hover !== null;
    renderer.domElement.style.cursor = drag ? "grabbing" : over && interactive ? "grab" : "";
    requestRender();
  }

  // ---- Ring dragging ----
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  function aim(event) {
    // A drag keeps the canvas's place from its start, so a layout shift under a still pointer can't move it.
    const rect = drag && drag.rect ? drag.rect : renderer.domElement.getBoundingClientRect();
    pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
    raycaster.setFromCamera(pointer, camera);
    return rect;
  }
  function ringUnder(event) {
    // Pick the ring whose drawn curve passes nearest the pointer on screen; overlapping hit volumes
    // would otherwise let a nearer ring steal the grab. Returns the ring and the angle grabbed on it.
    const rect = aim(event);
    const px = event.clientX - rect.left;
    const py = event.clientY - rect.top;
    let best = null;
    const point = new THREE.Vector3();
    for (const ring of rings) {
      ring.group.updateMatrixWorld(true);
      for (let k = 0; k < 96; k += 1) {
        const a = (k / 96) * Math.PI * 2;
        point.set(Math.cos(a) * ring.radius, Math.sin(a) * ring.radius, 0).applyMatrix4(ring.group.matrixWorld).project(camera);
        if (point.z > 1) continue;  // behind the camera
        const gap = Math.hypot(((point.x + 1) / 2) * rect.width - px, ((1 - point.y) / 2) * rect.height - py);
        if (gap <= RING_PICK_PX && (!best || gap < best.gap)) best = {ring, angle: a, gap};
      }
    }
    return best;
  }

  // Where a point on a ring lands on screen, in pixels from the canvas corner.
  function ringPixel(ring, angle, rect) {
    const p = new THREE.Vector3(Math.cos(angle) * ring.radius, Math.sin(angle) * ring.radius, 0)
      .applyMatrix4(ring.group.matrixWorld).project(camera);
    return new THREE.Vector2(((p.x + 1) / 2) * rect.width, ((1 - p.y) / 2) * rect.height);
  }
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

  function ringAngle(ring, event) {
    // Angle of the pointer around the ring, in the ring's own plane.
    aim(event);
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(drag.normal, drag.center);
    const hit = raycaster.ray.intersectPlane(plane, new THREE.Vector3());
    if (!hit) return null;
    const local = ring.group.worldToLocal(hit);
    return Math.atan2(local.y, local.x);
  }

  // ---- Gizmo dragging: slide the flange along an arrow, or turn it about a ring's axis ----
  const toScreen = (local, rect) => {
    const p = local.clone().applyMatrix4(gizmo.matrixWorld).project(camera);
    return p.z > 1 ? null : new THREE.Vector2(((p.x + 1) / 2) * rect.width, ((1 - p.y) / 2) * rect.height);
  };
  const ringLocal = (handle, angle) => handle.u.clone().multiplyScalar(Math.cos(angle) * GIZMO.ring)
    .addScaledVector(handle.v, Math.sin(angle) * GIZMO.ring);
  function segmentGap(px, py, a, b) {
    const ab = b.clone().sub(a);
    const t = Math.max(0, Math.min(1, ((px - a.x) * ab.x + (py - a.y) * ab.y) / Math.max(ab.lengthSq(), 1e-9)));
    return Math.hypot(a.x + ab.x * t - px, a.y + ab.y * t - py);
  }
  // The handle whose drawn shape passes nearest the pointer on screen, and where on a ring it was grabbed.
  function gizmoUnder(event) {
    if (!gizmo.visible) return null;
    sizeGizmo();
    const rect = aim(event);
    const px = event.clientX - rect.left;
    const py = event.clientY - rect.top;
    let best = null;
    for (const handle of handles) {
      if (handle.type === "move") {
        const axis = new THREE.Vector3(...BASE_AXES[handle.axis]);
        const a = toScreen(axis.clone().multiplyScalar(GIZMO.shaft[0]), rect);
        const b = toScreen(axis.clone().multiplyScalar(GIZMO.head[1]), rect);
        if (!a || !b) continue;
        const gap = segmentGap(px, py, a, b);
        if (gap <= GIZMO.pick && (!best || gap < best.gap)) best = {handle, gap};
      } else {
        for (let k = 0; k < 120; k += 1) {
          const angle = (k / 120) * Math.PI * 2;
          const p = toScreen(ringLocal(handle, angle), rect);
          if (!p) continue;
          const gap = Math.hypot(p.x - px, p.y - py);
          if (gap <= GIZMO.pick && (!best || gap < best.gap)) best = {handle, gap, angle};
        }
      }
    }
    return best;
  }
  // Where along the dragged arrow's line the pointer's ray passes closest (metres from the flange), or null
  // when the arrow points nearly along the ray.
  function alongAxis(d) {
    const {origin, direction} = raycaster.ray;
    const w = origin.clone().sub(d.center);
    const b = d.axisVector.dot(direction);
    const denominator = 1 - b * b;
    if (denominator < 1e-4) return null;
    return (d.axisVector.dot(w) - b * direction.dot(w)) / denominator;
  }
  function turnAngle(d) {
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(d.axisVector, d.center);
    const hit = raycaster.ray.intersectPlane(plane, new THREE.Vector3());
    if (!hit) return null;
    hit.sub(d.center);
    return Math.atan2(hit.dot(d.handle.v), hit.dot(d.handle.u));
  }
  const gizmoCentre = (rect) => toScreen(new THREE.Vector3(), rect) || new THREE.Vector2(rect.width / 2, rect.height / 2);

  function startToolDrag(pick, event) {
    const handle = pick.handle;
    const rect = aim(event);
    // The drag edits the plan (or the real arm's pose when nothing is staged) as a whole.
    const base = plan ? withPlan(target, plan) : [...target];
    const from = solver.pose(base);
    drag = {
      kind: "tool", handle, type: handle.type, axis: handle.axis, pointerId: event.pointerId, start: [...target],
      base, from, seed: base, pose: base, applied: 0, reason: null, check: null, rect,
      center: new THREE.Vector3(...from.p), axisVector: new THREE.Vector3(...BASE_AXES[handle.axis]),
      x: event.clientX, y: event.clientY, total: 0, startAngle: pick.angle || 0,
    };
    // How far a pixel reaches at the gizmo's depth, for the on-screen fallbacks below.
    drag.metresPerPx = gizmo.scale.x;
    if (handle.type === "move") {
      const along = alongAxis(drag);
      const a = toScreen(new THREE.Vector3(), rect);
      const b = toScreen(drag.axisVector.clone().multiplyScalar(GIZMO.head[1]), rect);
      // Seen end-on, an arrow can't be followed along its line: slide it by dragging up and down instead.
      drag.planar = along !== null && a && b && a.distanceTo(b) > 24;
      drag.t0 = drag.planar ? along : 0;
    } else {
      const facing = Math.abs(raycaster.ray.direction.dot(drag.axisVector));
      drag.planar = facing > RING_FACING_MIN;
      if (drag.planar) {
        drag.last = turnAngle(drag);
        if (drag.last === null) drag.planar = false;
        else drag.startAngle = drag.last;
      }
      if (!drag.planar) {
        const here = toScreen(ringLocal(handle, drag.startAngle), rect);
        const ahead = toScreen(ringLocal(handle, drag.startAngle + 0.05), rect);
        const along = here && ahead ? ahead.sub(here) : new THREE.Vector2(1, 0);
        drag.tangent = along.length() > 0.05 ? along.normalize() : new THREE.Vector2(1, 0);
        drag.pxPerRad = RING_PX_PER_DEG / RAD;
      }
    }
    styleRings();
    updateToolDrag(event);
  }

  // The flange's pose `value` along the dragged handle from where the drag began: mm along the arrow, or
  // degrees about the ring's axis (the flange stays put).
  function toolTarget(d, value) {
    if (d.type === "move") {
      const p = [...d.from.p];
      p[d.axis] += value / 1000;
      return {p, R: d.from.R};
    }
    return {p: d.from.p, R: multiply(rotation(BASE_AXES[d.axis], value * RAD), d.from.R)};
  }

  // Follows the drag from the last pose reached toward `wanted` in short steps, each solved from the one
  // before so the arm keeps its configuration, and stops at the last value it can reach, on the `fine` grid.
  function followTool(d, wanted, fine) {
    const steps = Math.round((wanted - d.applied) / fine);
    const startValue = d.applied;
    const direction = Math.sign(steps);
    const per = Math.max(1, Math.round(TOOL_CHUNK[d.type] / fine));
    let done = 0;
    let failure = null;
    let failedAt = 0;
    const attempt = (n) => {
      const result = solver.solve(d.seed, toolTarget(d, startValue + n * fine), limits);
      if (result.ok) {
        d.seed = result.joints;
        d.applied = startValue + n * fine;
      }
      return result;
    };
    while (done !== steps) {
      const next = done + direction * Math.min(per, Math.abs(steps - done));
      const result = attempt(next);
      if (!result.ok) {
        failure = result;
        failedAt = next;
        break;
      }
      done = next;
    }
    if (failure) {
      // Home in on the edge between the last step reached and the one that failed.
      let good = done;
      let bad = failedAt;
      while (Math.abs(bad - good) > 1) {
        const middle = good + Math.trunc((bad - good) / 2);
        const result = attempt(middle);
        if (result.ok) good = middle;
        else {
          bad = middle;
          failure = result;
        }
      }
    }
    d.reason = !failure ? null : failure.atLimit >= 0 ? `J${failure.atLimit + 1} is at its limit` : "Out of reach";
  }

  function updateToolDrag(event) {
    const d = drag;
    const rect = aim(event);
    let raw;
    if (d.type === "move") {
      const along = d.planar ? alongAxis(d) : null;
      raw = along !== null ? (along - d.t0) * 1000 : d.planar ? d.lastRaw || 0 : (d.y - event.clientY) * d.metresPerPx * 1000;
      d.lastRaw = raw;
    } else {
      if (d.planar) {
        const angle = turnAngle(d);
        if (angle !== null) {
          d.total += wrap(angle - d.last);
          d.last = angle;
        }
      } else {
        d.total = ((event.clientX - d.x) * d.tangent.x + (event.clientY - d.y) * d.tangent.y) / d.pxPerRad;
      }
      raw = d.total / RAD;
    }
    const [fine, coarse] = TOOL_SNAP[d.type];
    const snap = event.shiftKey ? coarse : fine;
    followTool(d, Math.round(raw / snap) * snap, fine);
    d.pose = d.seed;
    d.check = checkPath(d.start, d.pose);
    updateGhost();
    sizeGizmo();
    if (d.type === "turn") {
      // A translucent amber wedge edged by an arc shows how far, and which way, this drag turns the tool.
      const turn = d.applied * RAD;
      const {sweep, wedge} = d.handle;
      const start = turn >= 0 ? d.startAngle : d.startAngle + turn;
      sweep.visible = wedge.visible = Math.abs(turn) > 0.001;
      if (sweep.visible) {
        sweep.geometry.dispose();
        sweep.geometry = new THREE.TorusGeometry(GIZMO.ring, GIZMO.tube * 2.2, 6, 120, Math.abs(turn));
        sweep.rotation.z = start;
        wedge.geometry.dispose();
        wedge.geometry = new THREE.CircleGeometry(GIZMO.ring, 120, start, Math.abs(turn));
        const tone = d.check.clear ? COLOR.amber : COLOR.alarm;
        sweep.material.color.setHex(tone);
        wedge.material.color.setHex(tone);
      }
    }
    const centre = gizmoCentre(rect);
    // Anchor the page's readout just outside the gizmo, toward the pointer, so it never covers a handle or letter.
    const toward = new THREE.Vector2(event.clientX - rect.left - centre.x, event.clientY - rect.top - centre.y);
    if (toward.length() < 1) toward.set(1, 0);
    toward.normalize().multiplyScalar(GIZMO.label + 12);
    onPreview({
      kind: "tool", type: d.type, name: d.handle.name, axis: d.axis, pose: d.pose,
      from: d.type === "move" ? d.from.p[d.axis] * 1000 : 0, amount: d.applied,
      clear: d.check.clear, hit: d.check.hit, limited: d.reason,
      x: rect.left + centre.x + toward.x, y: rect.top + centre.y + toward.y, cx: rect.left + centre.x, cy: rect.top + centre.y,
    });
  }

  function grab(event) {
    controls.enabled = false;  // this capture listener runs before OrbitControls', so the camera stays put
    event.preventDefault();
    event.stopImmediatePropagation();
    renderer.domElement.setPointerCapture(event.pointerId);
  }

  renderer.domElement.addEventListener("pointerdown", (event) => {
    if (event.button !== 0 || !interactive || !shown) return;
    if (mode === "tool") {
      const pick = gizmoUnder(event);
      if (!pick) return;
      grab(event);
      startToolDrag(pick, event);
      return;
    }
    const pick = ringUnder(event);
    if (!pick) return;
    const ring = pick.ring;
    grab(event);
    ring.group.updateMatrixWorld(true);
    const center = new THREE.Vector3().setFromMatrixPosition(ring.group.matrixWorld);
    const normal = zAxis.clone().transformDirection(ring.group.matrixWorld);
    const facing = Math.abs(raycaster.ray.direction.dot(normal));
    // The drag edits the plan (or the real arm's pose when nothing is staged) one joint at a time.
    const base = plan ? withPlan(target, plan) : [...target];
    drag = {
      kind: "ring", joint: ring.joint, ring, center, normal, start: [...target], base, pose: base, check: null,
      from: base[ring.joint], actualFrom: target[ring.joint], total: 0, to: base[ring.joint], pointerId: event.pointerId,
      rect: renderer.domElement.getBoundingClientRect(),
      // Seen nearly edge-on, the ring's plane is useless for tracking; follow the ring on screen instead.
      planar: facing > RING_FACING_MIN, x: event.clientX, y: event.clientY, startAngle: pick.angle,
    };
    if (drag.planar) {
      drag.last = ringAngle(ring, event);
      if (drag.last === null) drag.planar = false;
      else drag.startAngle = drag.last;
    }
    if (!drag.planar) {
      // Drag along the ring's on-screen direction at the grab point, at a fixed rate.
      const rect = renderer.domElement.getBoundingClientRect();
      const here = ringPixel(ring, pick.angle, rect);
      const along = ringPixel(ring, pick.angle + 0.05, rect).sub(here);
      drag.tangent = along.length() > 0.05 ? along.normalize() : new THREE.Vector2(1, 0);
      drag.pxPerRad = RING_PX_PER_DEG / RAD;
    }
    // The ring's centre on screen, so the page can keep its readout clear of the ring and its arc.
    const rect = renderer.domElement.getBoundingClientRect();
    const centre = ringPixel(ring, 0, rect).add(ringPixel(ring, Math.PI, rect)).multiplyScalar(0.5);
    drag.cx = rect.left + centre.x;
    drag.cy = rect.top + centre.y;
    styleRings();
    updateDrag(event);
  }, true);

  function updateDrag(event) {
    if (drag.kind === "tool") {
      updateToolDrag(event);
      return;
    }
    const ring = drag.ring;
    if (drag.planar) {
      const angle = ringAngle(ring, event);
      if (angle !== null) {
        drag.total += wrap(angle - drag.last);
        drag.last = angle;
      }
    } else {
      drag.total = ((event.clientX - drag.x) * drag.tangent.x + (event.clientY - drag.y) * drag.tangent.y) / drag.pxPerRad;
    }
    const [low, high] = limits[drag.joint] || [-180, 180];
    const snap = event.shiftKey ? SNAP_COARSE_DEG : SNAP_DEG;
    const raw = drag.from + drag.total / RAD;
    drag.to = Math.min(high, Math.max(low, Math.round(raw / snap) * snap));
    drag.pose = [...drag.base];
    drag.pose[drag.joint] = drag.to;
    drag.check = checkPath(drag.start, drag.pose);
    updateGhost();
    // The amber arc on the ring shows how far this drag turns the joint.
    const turn = (drag.to - drag.from) * RAD;
    ring.sweep.visible = Math.abs(turn) > 0.001;
    if (ring.sweep.visible) {
      ring.sweep.geometry.dispose();
      ring.sweep.geometry = new THREE.TorusGeometry(ring.radius, RING_TUBE * 1.9, 6, 96, Math.abs(turn));
      ring.sweep.rotation.z = turn >= 0 ? drag.startAngle : drag.startAngle + turn;
      ring.sweep.material.color.setHex(drag.check.clear ? COLOR.amber : COLOR.alarm);
    }
    onPreview({joint: drag.joint, from: drag.actualFrom, to: drag.to, clear: drag.check.clear, hit: drag.check.hit,
      x: event.clientX, y: event.clientY, cx: drag.cx, cy: drag.cy, limited: raw < low || raw > high});
  }

  function endDrag(cancelled) {
    const done = drag;
    drag = null;
    controls.enabled = true;
    if (done.kind === "tool") {
      if (done.handle.sweep) done.handle.sweep.visible = done.handle.wedge.visible = false;
      styleRings();
      updateGhost();
      onPreview(null);
      onRelease({kind: "tool", type: done.type, name: done.handle.name, axis: done.axis, amount: done.applied,
        from: done.type === "move" ? done.from.p[done.axis] * 1000 : 0, pose: done.pose, cancelled,
        changed: !cancelled && Math.abs(done.applied) > 1e-9});
      return;
    }
    done.ring.sweep.visible = false;
    styleRings();
    updateGhost();
    onPreview(null);
    onRelease({joint: done.joint, from: done.actualFrom, to: done.to, cancelled,
      changed: !cancelled && Math.abs(done.to - done.from) >= MIN_MOVE_DEG});
  }

  renderer.domElement.addEventListener("pointermove", (event) => {
    if (drag) {
      if (event.pointerId === drag.pointerId) updateDrag(event);
      return;
    }
    if (event.buttons) return;  // orbiting
    if (mode === "tool") {
      const pick = interactive ? gizmoUnder(event) : null;
      const handle = pick ? pick.handle : null;
      if (handle !== hoverHandle) {
        hoverHandle = handle;
        styleRings();
      }
      return;
    }
    const pick = interactive ? ringUnder(event) : null;
    const joint = pick ? pick.ring.joint : null;
    if (joint !== hover) {
      hover = joint;
      styleRings();
    }
  });
  renderer.domElement.addEventListener("pointerleave", () => {
    if (!drag && (hover !== null || hoverHandle !== null)) {
      hover = null;
      hoverHandle = null;
      styleRings();
    }
  });
  for (const type of ["pointerup", "pointercancel"]) {
    renderer.domElement.addEventListener(type, (event) => {
      if (drag && event.pointerId === drag.pointerId) endDrag(type === "pointercancel");
    });
  }

  // ---- Rendering ----
  function resize() {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    // The page docks the plan bar by this: along the bottom of a tall screen, at the right of a wide one.
    wide = width >= WIDE.minWidth && width / height > WIDE.ratio;
    host.dataset.shape = wide ? "wide" : "tall";
    // A sprite that ignores distance is sized in units of the view's height: keep the mark MARK_PX across.
    contactMark.scale.setScalar((2 * MARK_PX * Math.tan((camera.fov * RAD) / 2)) / height);
    if (!userMoved && !drag) resetView();  // never re-frame under a drag
    requestRender();
  }
  const observer = new ResizeObserver(resize);
  observer.observe(host);
  resize();
  resetView();

  // In Tool mode, keep the gizmo whole on screen and clear of the plan bar: when a new plan, or the bar
  // growing, leaves its arrows near an edge or under the bar, re-frame (unless the user has set the view, or a
  // drag is on). It's checked on the next frame, once the page has laid the bar out, so a new plan re-frames once.
  function gizmoCrowded() {
    if (!gizmo.visible) return false;
    sizeGizmo();
    const canvas = renderer.domElement.getBoundingClientRect();
    const centre = toScreen(new THREE.Vector3(), canvas);
    if (!centre) return true;
    const room = GIZMO.head[1] + 10;  // the arrow tips, and a little air
    if (Math.min(centre.x, centre.y, canvas.width - centre.x, canvas.height - centre.y) < room) return true;
    const bar = planBar();
    if (!bar || bar.hidden) return false;
    const box = bar.getBoundingClientRect();
    const x = canvas.left + centre.x;
    const y = canvas.top + centre.y;
    return Math.hypot(x - Math.max(box.left, Math.min(x, box.right)), y - Math.max(box.top, Math.min(y, box.bottom))) < room;
  }
  let clearCheck = 0;
  function keepGizmoClear() {
    if (clearCheck) return;
    clearCheck = requestAnimationFrame(() => {
      clearCheck = 0;
      if (!disposed && mode === "tool" && !userMoved && !drag && gizmoCrowded()) resetView();
    });
  }
  if (planBar()) new ResizeObserver(() => keepGizmoClear()).observe(planBar());

  let last = performance.now();
  function frame(now) {
    if (disposed) return;
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    if (target && shown) {
      const ease = 1 - Math.exp(-dt / SMOOTH_S);
      let moving = false;
      for (let i = 0; i < count; i += 1) {
        const gap = target[i] - shown[i];
        if (Math.abs(gap) > 0.002) {
          shown[i] += gap * ease;
          moving = true;
        } else {
          shown[i] = target[i];
        }
      }
      if (moving) {
        actual.pose(shown);
        updateGhost();  // the rings ride on the shadow, which follows the real arm when nothing is staged
      }
    }
    if (controls.update()) dirty = true;
    if (dirty) {
      dirty = false;
      if (gizmo.visible) sizeGizmo();
      renderer.render(scene, camera);
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);

  renderer.domElement.addEventListener("webglcontextlost", (event) => {
    event.preventDefault();
    if (options.onLost) options.onLost();
  });

  return {
    joints: count,
    setActual(degrees) {
      if (!degrees || degrees.length < count || degrees.some((q) => q === null || !Number.isFinite(q))) return;
      target = degrees.slice(0, count);
      if (!shown) {
        shown = [...target];
        actual.pose(shown);
        actual.root.visible = true;
        ghost.root.visible = true;
        updateGhost();
        resetView();
      }
    },
    setGoal(degrees) {
      const next = degrees && degrees.length >= count ? degrees.slice(0, count) : null;
      const same = next === goal || (next && goal && next.every((q, i) => Math.abs(q - goal[i]) < 1e-6));
      if (same) return;
      goal = next;
      updateGhost();
    },
    // Stage a plan: an absolute target per joint, or null to leave that joint with the real arm. Returns
    // whether the straight joint-space move from the arm to the plan stays clear, and if not, what it hits, and
    // the flange's most mm per degree of the lead joint along it (null with no plan).
    // The check reruns only when the plan or the arm's position changes.
    setPlan(targets) {
      const next = staged(targets) ? targets.slice(0, count) : null;
      const key = next && target ? JSON.stringify([next, target.map((q) => Math.round(q * 100))]) : "";
      if (key !== planKey || Boolean(next) !== Boolean(plan)) {
        planKey = key;
        plan = next;
        planCheck = plan && target ? checkPath(target, withPlan(target, plan))
          : {clear: true, hit: null, points: null, flangePerDeg: null};
        updateGhost();
      }
      // A new plan (not the arm moving under one) may leave the gizmo near an edge or under the plan bar.
      const planOnly = next ? JSON.stringify(next) : "";
      if (planOnly !== lastPlan) {
        lastPlan = planOnly;
        keepGizmoClear();
      }
      return {clear: planCheck.clear, hit: planCheck.hit, flangePerDeg: planCheck.flangePerDeg};
    },
    // Whether new ring drags may start. A drag already under way carries on; the page cancels it
    // explicitly when the arm can no longer move.
    setInteractive(on) {
      if (interactive === Boolean(on)) return;
      interactive = Boolean(on);
      if (!interactive) hover = null;
      styleRings();
    },
    checkPath,
    // The cell, from the server: its collision model (the active tool's shape and the keep-out zones join the
    // check, as on the server, but for guarded zones open now), and the zones, frames, targets and patterns' slots to
    // draw. Zones are faint boxes with grey edges (red only when a plan would hit one), dashed for a guarded zone,
    // which loses its fill while open; frames a grey x and y and their name, targets small light dots (a touched-up
    // one with a grey ring where it was), slots smaller grey dots; none wear the gizmo's axis colours.
    setCell(cell) {
      if (cell && cell.collision) {
        collision = cell.collision;
        checker = createChecker(collision);
        solver = createSolver(collision);
        for (const overlay of highlights.values()) scene.remove(overlay);
        highlights.clear();
        planKey = null;  // re-check any staged plan against the new model
      }
      for (const child of [...cellGroup.children]) {
        cellGroup.remove(child);
        child.traverse((node) => {
          if (node.geometry) node.geometry.dispose();
          if (node.material && node.material.map && node.material.map !== dotTexture
              && node.material.map !== ringTexture) node.material.map.dispose();
        });
      }
      for (const zone of (cell && cell.zones) || []) {
        const geometry = new THREE.BoxGeometry(zone.half[0] * 2, zone.half[1] * 2, zone.half[2] * 2);
        const guarded = Boolean(zone.interlocks && zone.interlocks.length);
        const box = new THREE.Mesh(geometry, guarded && zone.open ? zoneOpen : zoneFill);
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry), guarded ? zoneDash : zoneEdge);
        if (guarded) edges.computeLineDistances();
        box.add(edges);
        box.position.set(...zone.centre);
        const [x, y, z] = zone.axes.map((axis) => new THREE.Vector3(...axis));
        box.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
        box.renderOrder = 2;
        cellGroup.add(box);
      }
      for (const frame of (cell && cell.frames) || []) {
        const group = new THREE.Group();
        group.position.set(...frame.origin);
        const along = (axis, length, material) => new THREE.Line(new THREE.BufferGeometry().setFromPoints(
          [new THREE.Vector3(), new THREE.Vector3(...axis).multiplyScalar(length)]), material);
        group.add(along(frame.x, 0.07, frameInk), along(frame.y, 0.045, frameGrey));
        group.add(textSprite(frame.name, "#aaa69d"));
        group.children[2].position.set(...frame.x.map((v) => v * 0.085));
        cellGroup.add(group);
      }
      for (const target of (cell && cell.targets) || []) {
        const dot = new THREE.Sprite(targetDot);
        dot.scale.set(0.012, 0.012, 1);
        dot.position.set(...target.p);
        dot.renderOrder = 9;
        cellGroup.add(dot);
        if (target.was) {  // touched up: where it was, and the way it moved
          const ring = new THREE.Sprite(wasRing);
          ring.scale.set(0.014, 0.014, 1);
          ring.position.set(...target.was);
          ring.renderOrder = 9;
          const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(
            [new THREE.Vector3(...target.was), new THREE.Vector3(...target.p)]), wasLine);
          line.renderOrder = 8;
          cellGroup.add(ring, line);
        }
      }
      for (const slot of (cell && cell.slots) || []) {
        const dot = new THREE.Sprite(slotDot);
        dot.scale.set(0.007, 0.007, 1);
        dot.position.set(...slot.p);
        dot.renderOrder = 8;
        cellGroup.add(dot);
      }
      if (shown) updateGhost();
      dirty = true;
    },
    // Where a program's step would have collided (the server's hit: pose, closest points, obstacle index), drawn
    // as a blocked plan is: the red shadow arm at that pose, the contact mark and the red overlay. null clears it.
    setFault(hit) {
      const next = hit && Array.isArray(hit.pose) && hit.pose.length >= count ? hit : null;
      if (JSON.stringify(next) === JSON.stringify(fault)) return;
      fault = next;
      if (shown) updateGhost();
    },
    // What dragging in the view moves: "joints" (a ring per joint) or "tool" (the gizmo at the flange).
    setMode(next) {
      const wanted = next === "tool" ? "tool" : "joints";
      if (wanted === mode) return;
      if (drag) endDrag(true);
      mode = wanted;
      hover = null;
      hoverHandle = null;
      for (const ring of rings) ring.group.visible = mode === "joints";
      if (shown) updateGhost();
      else gizmo.visible = false;
      if (!userMoved) resetView();  // make room for the gizmo, or take it back
      styleRings();
    },
    // Screen points of a gizmo handle (used by automated checks): partway along an arrow and its tip, or a ring
    // point clear of the arrows; and the gizmo's centre.
    gizmoPoint(type, axis) {
      const handle = handles.find((h) => h.type === type && h.axis === axis);
      if (!handle || !gizmo.visible) return null;
      sizeGizmo();
      const rect = renderer.domElement.getBoundingClientRect();
      const centre = gizmoCentre(rect);
      const direction = new THREE.Vector3(...BASE_AXES[axis]);
      let point = null;
      let tip = null;
      if (type === "move") {
        point = toScreen(direction.clone().multiplyScalar((GIZMO.shaft[0] + GIZMO.shaft[1]) / 2), rect);
        tip = toScreen(direction.clone().multiplyScalar(GIZMO.head[1]), rect);
      } else {
        for (const angle of [0.25, 0.75, 1.25, 1.75].map((x) => x * Math.PI)) {
          const p = toScreen(ringLocal(handle, angle), rect);
          if (p && (!point || p.distanceTo(centre) > point.distanceTo(centre))) point = p;
        }
      }
      if (!point) return null;
      const page = (p) => p && {x: rect.left + p.x, y: rect.top + p.y};
      return {...page(point), tip: page(tip), cx: rect.left + centre.x, cy: rect.top + centre.y};
    },
    cancelDrag() {
      if (!drag) return false;
      endDrag(true);
      return true;
    },
    isDragging: () => Boolean(drag),
    // Screen position of a ring's outermost point and its centre (used by automated checks).
    ringPoint(joint) {
      const ring = rings[joint];
      if (!ring || !actual.root.visible) return null;
      const rect = renderer.domElement.getBoundingClientRect();
      const toScreen = (v) => {
        const p = v.project(camera);
        return {x: rect.left + ((p.x + 1) / 2) * rect.width, y: rect.top + ((1 - p.y) / 2) * rect.height};
      };
      ring.group.updateMatrixWorld(true);
      const centre = toScreen(new THREE.Vector3().setFromMatrixPosition(ring.group.matrixWorld));
      let best = null;
      for (let k = 0; k < 72; k += 1) {
        const a = (k / 72) * Math.PI * 2;
        const point = toScreen(new THREE.Vector3(Math.cos(a) * ring.radius, Math.sin(a) * ring.radius, 0)
          .applyMatrix4(ring.group.matrixWorld));
        const reach = Math.hypot(point.x - centre.x, point.y - centre.y);
        if (!best || reach > best.reach) best = {...point, reach};
      }
      return {x: best.x, y: best.y, cx: centre.x, cy: centre.y};
    },
    // Where to anchor a readout for a joint: just outside its ring, on the side away from the links that
    // follow it (for the last joint, away from the joint before), plus the ring's centre.
    labelPoint(joint) {
      const ring = rings[joint];
      if (!ring || !actual.root.visible) return null;
      const rect = renderer.domElement.getBoundingClientRect();
      const middle = (r) => {
        r.group.updateMatrixWorld(true);
        return ringPixel(r, 0, rect).add(ringPixel(r, Math.PI, rect)).multiplyScalar(0.5);
      };
      const centre = middle(ring);
      const flange = new THREE.Vector3().setFromMatrixPosition(ghost.rotors[ghost.rotors.length - 1].matrixWorld).project(camera);
      let away = centre.clone().sub(new THREE.Vector2(((flange.x + 1) / 2) * rect.width, ((1 - flange.y) / 2) * rect.height));
      if (away.length() < 12 && joint > 0) away = centre.clone().sub(middle(rings[joint - 1]));
      if (away.length() < 1) away.set(1, 0);
      away.normalize();
      let reach = 12;
      for (let k = 0; k < 36; k += 1) reach = Math.max(reach, ringPixel(ring, (k / 36) * Math.PI * 2, rect).distanceTo(centre));
      return {x: rect.left + centre.x + away.x * reach, y: rect.top + centre.y + away.y * reach,
              cx: rect.left + centre.x, cy: rect.top + centre.y};
    },
    resetView,
  };
}
