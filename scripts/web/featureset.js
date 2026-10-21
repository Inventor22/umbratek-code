// The jog panel's Program, Teach and Cell sections: running programs on the server (Run, Step, Dry run, Pause,
// Stop run, Reset and the speed override), editing their steps, teaching targets in freedrive, and setting up the
// cell's tools, frames, targets and keep-out zones. The server keeps and checks all of it (docs/capabilities.md);
// this module shows it and sends the operator's requests. It builds on the panel's own script through
// window.jogPanel and runs after it.
const P = window.jogPanel;
const {el, setText} = P;
const $ = (id) => document.getElementById(id);
const RUN_MOVING = ["running", "pausing", "stopping"];
const RUN_BUSY = [...RUN_MOVING, "paused"];
const RUN_ENDED = ["idle", "done", "stopped"];
const HOLD_MS = 700;
const CONFIRM_MS = 3000;
const OVERRIDES = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];
const SENSITIVITIES = [90, 100, 110];  // the teach sensitivity in % (Umbratek's current SDK); see jog_server.py

let status = null;
let cell = null;  // GET /api/cell's reply: the cell, its frames and targets in the base frame, and what to draw
let cellAsked = null;  // the revision being fetched
let drawnIn = null;  // the 3D view the cell was last drawn in
let programs = [];
let program = null;  // the program on screen, as the server saved it
let selected = null;  // the selected step's path (indexes from 0); new steps go after it
let shownRun = "";  // what the step lamps last showed
let lastEventKey = null;
let eventsBusy = false;

// ---- talking to the server ----
async function getJson(path) {
  const res = await fetch(path, {cache: "no-store"});
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return res.json();
}

// Sends a request and puts the server's answer on the message line. Returns the reply, or null if it refused.
async function send(path, body, key) {
  let reply;
  try {
    reply = await P.post(path, body || {});
  } catch (e) {
    P.say("The jog panel server isn't answering. Is jog_server.py still running?", "bad");
    if (key) P.refuse(key);
    return null;
  }
  if (reply.message) P.say(reply.message, reply.ok ? "ok" : "bad");
  if (!reply.ok) {
    if (key) P.refuse(key);
    return null;
  }
  if (reply.cell) takeCell(reply);
  return reply;
}

const revision = () => (cell ? cell.cell.revision : undefined);

// ---- small builders ----
const fix = (v, digits = 1) => {
  const text = Number(v).toFixed(digits);
  return /^-0\.?0*$/.test(text) ? text.slice(1) : text;
};
const triple = (values, digits = 1) => `(${values.map((v) => fix(v, digits)).join(", ")})`;
const pathText = (path) => path.map((i) => i + 1).join(".");
const samePath = (a, b) => Boolean(a && b) && a.length === b.length && a.every((v, i) => v === b[i]);
const num = (id) => {
  const value = Number($(id).value);
  return Number.isFinite(value) ? value : 0;
};

function option(value, label) {
  const node = el("option", "", label);
  node.value = value;
  return node;
}

// Refills a select, keeping its choice if it's still there.
function fillSelect(select, entries, empty) {
  const keep = select.value;
  const items = entries.map(([value, label]) => option(value, label));
  if (!items.length && empty) items.push(option("", empty));
  select.replaceChildren(...items);
  if (entries.some(([value]) => value === keep)) select.value = keep;
}

function miniKey(text, label, onPress, disabled) {
  const key = el("button", "key mini", text);
  key.type = "button";
  if (label) key.setAttribute("aria-label", label);
  key.disabled = Boolean(disabled);
  key.addEventListener("click", () => onPress(key));
  return key;
}

// Deleting takes two presses: the first arms the key, which says "Sure?" in red for a few seconds.
function twice(key, action) {
  if (key.dataset.armed) {
    clearTimeout(key.armTimer);
    delete key.dataset.armed;
    setText(key, key.dataset.text);
    action();
    return;
  }
  key.dataset.text = key.textContent;
  key.dataset.armed = "1";
  setText(key, "Sure?");
  P.say("Press again to delete.", "wait");
  key.armTimer = setTimeout(() => {
    delete key.dataset.armed;
    setText(key, key.dataset.text);
  }, CONFIRM_MS);
}

// A list row: the name in bold, its details, an optional cue (red if bad), and its keys.
function row(name, detail, cue, bad) {
  const li = el("li", "row");
  const what = el("span", "what");
  what.append(el("b", "", name));
  if (detail) what.append(` ${detail}`);
  if (cue) what.append(el("span", bad ? "cue bad" : "cue", cue));
  const acts = el("span", "acts");
  li.append(what, acts);
  return {li, acts};
}

// A detented fader like the panel's step and speed faders, for the speed override and the teach sensitivity.
function fader(holder, {name, label, values, unit, value, home, onChange}) {
  holder.classList.add("detent");
  const input = el("input");
  Object.assign(input, {type: "range", id: `${holder.id}Input`, min: "0", max: String(values.length - 1), step: "1"});
  input.setAttribute("aria-label", name);
  const legend = el("label", "legend", label);
  legend.htmlFor = input.id;
  const readout = el("output", "detent-value");
  readout.setAttribute("for", input.id);
  const ticks = el("span", "detent-ticks");
  values.forEach((v, i) => {
    const tick = el("i", v === home ? "home" : "");
    tick.style.left = `${(i / (values.length - 1)) * 100}%`;
    ticks.append(tick);
  });
  const ends = el("span", "detent-ends");
  ends.append(el("span", "", `${values[0]}`), el("span", "", `${values[values.length - 1]}`));
  ticks.setAttribute("aria-hidden", "true");
  ends.setAttribute("aria-hidden", "true");
  const track = el("span", "detent-track");
  track.append(ticks, ends, input);
  holder.append(legend, readout, track);
  const show = () => {
    const v = values[Number(input.value)];
    setText(readout, `${v}${unit}`);
    input.setAttribute("aria-valuetext", `${v}${unit}`);
  };
  input.value = String(Math.max(0, values.indexOf(value)));
  show();
  input.addEventListener("input", show);
  input.addEventListener("change", () => onChange(values[Number(input.value)]));
  return {
    input,
    get value() { return values[Number(input.value)]; },
    // Follows the server, but never under the user's hand.
    set(v) {
      const i = values.indexOf(v);
      if (i < 0 || document.activeElement === input || input.matches(":active")) return;
      if (input.value !== String(i)) {
        input.value = String(i);
        show();
      }
    },
  };
}

// A row of joined mode keys, the desk's two- or three-way switch (as Joints | Tool above the 3D view): the key
// pressed in is the choice.
function choice(id, onChange) {
  const keys = [...$(id).querySelectorAll("button")];
  const c = {
    get value() {
      const pressed = keys.find((k) => k.getAttribute("aria-pressed") === "true");
      return pressed ? pressed.dataset.value : null;
    },
    set(value) {
      for (const k of keys) k.setAttribute("aria-pressed", String(k.dataset.value === value));
    },
    key: (value) => keys.find((k) => k.dataset.value === value),
  };
  for (const k of keys) {
    k.addEventListener("click", () => {
      c.set(k.dataset.value);
      if (onChange) onChange(k.dataset.value);
    });
  }
  return c;
}

// ---- the arm's readiness, as the server judges it ----
function ready(s) {
  return Boolean(s && s.live && s.enabled && !s.err && !s.stopAlarm && !s.freedrive && [0, 2].includes(s.state));
}

function whyNotReady(s) {
  if (!s) return "The jog panel server isn't answering.";
  if (!s.live) return "There's no status from the controller.";
  if (s.stopAlarm) return "A stop wasn't confirmed: use the hardware E-stop.";
  if (s.err) return "The controller reports a fault: hold Enable to clear it.";
  if (!s.enabled) return "The motors are off: hold Enable.";
  if (s.freedrive) return "The arm is in freedrive: leave it in Teach.";
  if (s.state === 4 || s.state === 3) return "The arm is stopped: hold Enable.";
  return "The arm is still moving.";
}

const runOf = (s) => (s && s.run) || {state: "idle"};

// ---- Program: the run window, the run keys and the override ----
const override = fader($("overrideFader"), {
  name: "Speed override", label: "Override", values: OVERRIDES, unit: "%", value: 100, home: 100,
  onChange: (percent) => send("/api/run/override", {percent}),
});

function countSteps(steps) {
  return steps.reduce((n, s) => n + 1 + countSteps(s.steps || []) + countSteps(s.then || []) + countSteps(s.else || []), 0);
}

function runWords(s) {
  if (!s) return ["NO SERVER", "The jog panel server isn't answering.", "idle"];
  const r = runOf(s);
  const at = r.path ? `step ${pathText(r.path)}` : "";
  const ev = r.lastEvent || {};
  const speed = r.override < 100 ? ` · ${r.override}% speed` : "";
  const reason = (r.reason || "").replace(/\.+$/, "");
  switch (r.state) {
    case "running":
      if (r.mode === "dry") return ["DRY RUN", `Planning ${r.program}; nothing moves. ${at}: ${r.step}`, "idle"];
      if (r.manual) return ["MOVING", `${r.step}${speed}. STOP or Stop run ends it.`, "moving"];
      return [r.mode === "step" ? "STEPPING" : "RUNNING", `${r.program}, ${at}: ${r.step}${speed}`, "moving"];
    case "pausing":
      return ["PAUSING", "Bringing the arm to a stop; the run waits there.", "moving"];
    case "stopping":
      return ["STOPPING", "Stopping the arm.", "moving"];
    case "paused":
      if (r.halt) return ["HALTED", `${r.program}, ${at}: ${r.halt} Resume to carry on, or Stop run.`, "stopped"];
      if (r.reason === "stepped") return ["PAUSED", `${r.program}: ${at} done. Step runs the next; Resume runs on.`, "stopped"];
      return ["PAUSED", `${r.program}, ${at}: ${reason}. Resume re-plans from where the arm is.`, "stopped"];
    case "stopped":
      return ["STOPPED", `${r.manual ? "The move" : r.program} stopped: ${reason || "stopped"}. `
        + (r.manual ? "Plan it again to carry on." : "Run starts it again from the top, or Run from on a step; Reset clears this."),
        "stopped"];
    case "faulted":
      return ["FAULTED", r.hit
        ? `${reason}. The 3D view shows where. Move the target or the zone, or change the step, then press Reset.`
        : `${reason}. Press Reset once that's dealt with.`, "stopped"];
    case "done":
      if (r.manual) return ["ARRIVED", ev.seconds !== undefined ? `At the target in ${ev.seconds} s.` : "At the target.", "ready"];
      if (r.mode === "dry") {
        return ["PLANNED", `${r.program} would take about ${ev.estimateS} s at ${r.override}%. Nothing moved.`, "idle"];
      }
      return ["DONE", `${r.program} finished${ev.seconds !== undefined ? ` in ${ev.seconds} s` : ""}.`, "ready"];
    default:
      if (!program) return ["NO RUN", programs.length ? "Pick a program." : "Name a new program and press Create.", "idle"];
      if (!program.steps.length) return ["NO RUN", `${program.name} has no steps yet.`, "idle"];
      return ["NO RUN", `${program.name}: ${countSteps(program.steps)} steps. Run or Step moves the arm; Dry run only plans it.`,
        "idle"];
  }
}

// Why Run (or Step, or Dry run) can't start now, or null when it can.
function whyNotStart(s, mode) {
  const r = runOf(s);
  if (!program) return programs.length ? "Pick a program first." : "Create a program first.";
  if (!program.steps.length) return `${program.name} has no steps yet: add some, or record targets into it.`;
  if (r.state === "paused") return `${r.program} is paused: Resume or Step it, or Stop run to start again.`;
  if (!RUN_ENDED.includes(r.state)) return r.state === "faulted" ? "The last run faulted: press Reset first."
    : `${r.manual ? "A move" : r.program} is under way: wait, or Stop run.`;
  if (mode === "dry") return s && s.live ? null : "There's no status from the controller.";
  return ready(s) ? null : whyNotReady(s);
}

// Run, Step and Dry run stay pressable when they can't start, unlit, and a press is refused with the reason.
function gate(key, why) {
  key.disabled = false;
  key.setAttribute("aria-disabled", String(Boolean(why)));
  key.dataset.why = why || "";
}

function refused(key) {
  if (!key.dataset.why) return false;
  P.refuse(key);
  P.say(key.dataset.why, "bad");
  return true;
}

function renderRun(s) {
  const r = runOf(s);
  let [word, sentence, tone] = runWords(s);
  const have = Boolean(program && program.steps.length);
  if (have && RUN_ENDED.includes(r.state) && !ready(s)) sentence += ` ${whyNotReady(s)}`;
  setText($("runWord"), word);
  setText($("runSentence"), sentence);
  $("runWindow").dataset.tone = tone;

  const live = Boolean(s && s.live);
  const paused = r.state === "paused";
  gate($("runStart"), whyNotStart(s, "run"));
  gate($("runStepKey"), paused ? (live ? null : "There's no status from the controller.") : whyNotStart(s, "step"));
  gate($("runDry"), whyNotStart(s, "dry"));
  setText($("runPause"), paused ? "Resume" : "Pause");
  $("runPause").disabled = !((paused && live) || (r.state === "running" && !r.manual));
  $("runStop").disabled = !RUN_BUSY.includes(r.state);
  $("runReset").disabled = !["faulted", "stopped", "done"].includes(r.state);
  $("programDelete").disabled = !program || (RUN_BUSY.includes(r.state) && r.program === program.name);
  if (s && s.run) override.set(s.run.override);
}

$("runStart").addEventListener("click", (e) => {
  if (!refused(e.currentTarget)) send("/api/run/start", {name: program.name, mode: "run"}, e.currentTarget);
});
$("runDry").addEventListener("click", (e) => {
  if (!refused(e.currentTarget)) send("/api/run/start", {name: program.name, mode: "dry"}, e.currentTarget);
});
$("runStepKey").addEventListener("click", (e) => {
  if (refused(e.currentTarget)) return;
  if (runOf(status).state === "paused") send("/api/run/step", {}, e.currentTarget);
  else send("/api/run/start", {name: program.name, mode: "step", from: selected}, e.currentTarget);
});
$("runPause").addEventListener("click", (e) => send(runOf(status).state === "paused" ? "/api/run/resume" : "/api/run/pause",
  {}, e.currentTarget));
$("runStop").addEventListener("click", (e) => send("/api/run/stop", {}, e.currentTarget));
$("runReset").addEventListener("click", (e) => send("/api/run/reset", {}, e.currentTarget));

// ---- Program: choosing, creating and deleting programs ----
async function loadPrograms(want) {
  try {
    programs = (await getJson("/api/programs")).programs;
  } catch (e) {
    return;
  }
  const select = $("programSelect");
  fillSelect(select, programs.map((n) => [n, n]), "No programs yet");
  if (want && programs.includes(want)) select.value = want;
  select.disabled = !programs.length;
  const name = select.value;
  if (!name) {
    program = null;
    selected = null;
    renderSteps();
  } else if (!program || program.name !== name) {
    await loadProgram(name);
  }
}

async function loadProgram(name) {
  try {
    const reply = await getJson(`/api/program?name=${encodeURIComponent(name)}`);
    if (!reply.ok) throw new Error(reply.message);
    if (!program || program.name !== name) selected = null;
    program = reply.program;
  } catch (e) {
    P.say(`Couldn't load ${name}: ${e.message}`, "bad");
    program = null;
  }
  renderSteps();
  renderRun(status);
}

$("programSelect").addEventListener("change", () => loadProgram($("programSelect").value));
$("programCreate").addEventListener("click", async (event) => {
  const key = event.currentTarget;
  const name = $("programName").value.trim();
  if (programs.includes(name)) {
    P.say(`There's already a program called ${name}.`, "bad");
    P.refuse(key);
    return;
  }
  const reply = await send("/api/program", {program: {name, steps: []}}, key);
  if (!reply) return;
  $("programName").value = "";
  program = null;
  await loadPrograms(reply.program.name);
});
$("programName").addEventListener("keydown", (event) => { if (event.key === "Enter") $("programCreate").click(); });
$("programDelete").addEventListener("click", (event) => {
  if (!program) return;
  twice(event.currentTarget, async () => {
    if (await send("/api/program/delete", {name: program.name}, $("programDelete"))) {
      program = null;
      await loadPrograms();
    }
  });
});
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) loadPrograms(program && program.name);
});

// ---- Program: the steps ----
function describeStep(s) {
  switch (s.type) {
    case "move": {
      let text = `${s.kind === "line" ? "Straight move" : "Joint move"} to ${s.target}`;
      const o = s.offset;
      if (o && o.frame === "tool" && !o.xyz[0] && !o.xyz[1] && o.xyz[2] < 0 && !(o.rpy || []).some(Boolean)) {
        text += `, ${fix(-o.xyz[2], 0)} mm back along the tool`;
      } else if (o) {
        text += `, offset ${triple(o.xyz, 0)} mm in the ${o.frame === "tool" ? "tool's" : "target's"} frame`;
      }
      if (s.speed !== undefined) text += ` at ${fix(s.speed, 0)} ${s.kind === "line" ? "mm/s" : "°/s"}`;
      return text;
    }
    case "wait":
      return s.input !== undefined ? `Wait for ${s.input} = ${s.value} (up to ${s.timeout} s)` : `Wait ${s.seconds} s`;
    case "set": return `Set ${s.output} to ${s.value}`;
    case "pulse": return `Pulse ${s.output} for ${s.seconds} s`;
    case "if": return `If ${s.input} = ${s.value}`;
    case "repeat": return `Repeat ${s.count} times`;
    case "call": return `Run ${s.program}`;
    case "halt": return `Halt: ${s.message}`;
    case "comment": return s.text || "(comment)";
    default: return s.type;
  }
}

// Every step as a row: its path, depth, whether this page can edit it (the top level and repeats' steps; an if's
// branches are edited in its JSON file), and the label rows "Else" and "Then" for if steps.
function flatten(steps, path = [], depth = 0, editable = true, out = []) {
  steps.forEach((s, i) => {
    const p = [...path, i];
    out.push({s, path: p, depth, editable});
    if (s.type === "repeat") flatten(s.steps, p, depth + 1, editable, out);
    if (s.type === "if") {
      flatten(s.then, p, depth + 1, false, out);
      if (s.else.length) {
        out.push({label: "Else", depth});
        flatten(s.else, p, depth + 1, false, out);
      }
    }
  });
  return out;
}

function stepProblem(s) {
  if (s.type !== "move" || !cell) return "";
  const target = cell.cell.targets[s.target];
  if (!target) return "no such target";
  if (target.unreachable) return "out of reach since its frame moved";
  return "";
}

function renderSteps() {
  const list = $("steps");
  const r = runOf(status);
  const busy = Boolean(program) && RUN_BUSY.includes(r.state) && r.program === program.name;
  const rows = program ? flatten(program.steps) : [];
  if (selected && !rows.some((x) => x.s && samePath(x.path, selected))) selected = null;
  list.replaceChildren(...rows.map((x) => {
    const li = el("li", "row");
    li.dataset.depth = String(Math.min(x.depth, 3));
    if (x.label) {
      li.classList.add("comment");
      li.append(el("span"), el("span", "pick text", x.label), el("span"));
      return li;
    }
    li.dataset.path = pathText(x.path);
    if (x.s.type === "comment") li.classList.add("comment");
    const lamp = el("span", "led");
    const pick = el("button", "pick");
    pick.type = "button";
    pick.append(el("span", "num", pathText(x.path)), el("span", "text", describeStep(x.s)));
    const problem = stepProblem(x.s);
    if (problem) pick.lastChild.append(el("span", "cue bad", ` ${problem}`));
    const isSelected = samePath(x.path, selected);
    li.classList.toggle("selected", isSelected);
    pick.setAttribute("aria-pressed", String(isSelected));
    pick.addEventListener("click", () => {
      selected = samePath(x.path, selected) ? null : x.path;
      renderSteps();
    });
    const acts = el("span", "acts");
    const where = `step ${pathText(x.path)}`;
    const start = miniKey("Run from", `Run from ${where}`, (key) => send("/api/run/start",
      {name: program.name, mode: "run", from: x.path}, key), busy || !ready(status));
    acts.append(start);
    if (x.editable) {
      const container = containerOf(program.steps, x.path);
      const i = x.path[x.path.length - 1];
      acts.append(
        miniKey("\u2191", `Move ${where} up`, () => moveStep(x.path, -1), busy || i === 0),
        miniKey("\u2193", `Move ${where} down`, () => moveStep(x.path, 1), busy || i === container.length - 1),
        miniKey("\u00d7", `Delete ${where}`, (key) => twice(key, () => deleteStep(x.path)), busy),
      );
    }
    li.append(lamp, pick, acts);
    return li;
  }));
  shownRun = "";
  markCurrent();
  renderComposer();
}

// The lamp of the step the run is on: amber while it moves, a hollow amber ring while the run waits there (paused,
// halted or stepped), red if it failed; a dry run's step is grey, since nothing moves.
function markCurrent() {
  const r = runOf(status);
  const mine = Boolean(program) && r.program === program.name && !r.manual;
  const key = mine ? `${r.state}|${r.mode}|${r.path ? pathText(r.path) : ""}` : "";
  if (key === shownRun) return;
  shownRun = key;
  const on = !mine || !r.path ? null : r.state === "faulted" ? "dead"
    : r.mode === "dry" ? (RUN_MOVING.includes(r.state) ? "slow" : null)
    : RUN_MOVING.includes(r.state) ? "moving" : r.state === "paused" ? "held" : null;
  const at = mine && r.path ? pathText(r.path) : null;
  for (const li of $("steps").children) {
    const lamp = li.querySelector(".led");
    if (!lamp) continue;
    if (on && li.dataset.path === at) {
      lamp.dataset.on = on;
      li.dataset.current = on;
      li.setAttribute("aria-current", "step");
    } else {
      delete lamp.dataset.on;
      delete li.dataset.current;
      li.removeAttribute("aria-current");
    }
  }
  // Keep the step the run is on in sight, scrolling the list only (never the page).
  if (on) {
    const list = $("steps");
    const current = list.querySelector('[aria-current="step"]');
    if (current) {
      const box = list.getBoundingClientRect();
      const it = current.getBoundingClientRect();
      if (it.top < box.top) list.scrollTop -= box.top - it.top + 6;
      else if (it.bottom > box.bottom) list.scrollTop += it.bottom - box.bottom + 6;
    }
  }
}

function containerOf(steps, path) {
  let list = steps;
  for (const i of path.slice(0, -1)) list = list[i].steps || list[i].then || [];
  return list;
}

async function saveProgram(next, select) {
  const reply = await send("/api/program", {program: next, revision: program.revision});
  if (!reply) return false;
  program = reply.program;
  selected = select;
  renderSteps();
  return true;
}

function moveStep(path, by) {
  const next = structuredClone(program);
  const list = containerOf(next.steps, path);
  const i = path[path.length - 1];
  [list[i], list[i + by]] = [list[i + by], list[i]];
  saveProgram(next, [...path.slice(0, -1), i + by]);
}

function deleteStep(path) {
  const next = structuredClone(program);
  containerOf(next.steps, path).splice(path[path.length - 1], 1);
  saveProgram(next, null);
}

function insertStep(step) {
  const next = structuredClone(program);
  if (selected) {
    const i = selected[selected.length - 1] + 1;
    containerOf(next.steps, selected).splice(i, 0, step);
    return saveProgram(next, [...selected.slice(0, -1), i]);
  }
  next.steps.push(step);
  return saveProgram(next, [next.steps.length - 1]);
}

// ---- Program: adding a step ----
const SHOWN = {
  joint: ["stepTarget", "move"], line: ["stepTarget", "move"], wait: ["seconds"],
  waitInput: ["stepIoLine", "stepIoBit", "stepValue", "timeout"], set: ["stepIoLine", "stepIoBit", "stepValue"],
  pulse: ["stepIoLine", "stepIoBit", "seconds"], halt: ["stepText"], comment: ["stepText"],
};

function renderComposer() {
  const type = $("stepType").value;
  const shown = SHOWN[type];
  for (const id of ["stepTarget", "stepIoLine", "stepIoBit", "stepValue", "stepText"]) $(id).hidden = !shown.includes(id);
  for (const node of $("composer").querySelectorAll("[data-for]")) node.hidden = !shown.includes(node.dataset.for);
  setText($("stepSpeedUnit"), type === "line" ? "mm/s" : "°/s");
  $("stepSpeed").max = type === "line" ? "1000" : "120";
  $("stepSpeed").placeholder = program ? fix(program.speed[type === "line" ? "tcp" : "joint"], 0) : "";
  $("stepText").placeholder = type === "halt" ? "what the operator should do" : "note";
  const targets = cell ? Object.entries(cell.cell.targets) : [];
  fillSelect($("stepTarget"), targets.filter(([, t]) => type !== "line" || !t.joints)
    .map(([name, t]) => [name, t.joints ? `${name} (joints)` : name]), type === "line" ? "No tool targets yet" : "No targets yet");
  const r = runOf(status);
  const busy = Boolean(program) && RUN_BUSY.includes(r.state) && r.program === program.name;
  $("stepAdd").disabled = !program || busy;
  setText($("stepAdd"), selected ? `Add after ${pathText(selected)}` : "Add at the end");
}

function composedStep() {
  const type = $("stepType").value;
  const io = `${ioLine.value}.${Math.round(num("stepIoBit"))}`;
  const value = Number(ioValue.value);
  switch (type) {
    case "joint":
    case "line": {
      if (!$("stepTarget").value) return null;
      const step = {type: "move", kind: type, target: $("stepTarget").value};
      const back = num("stepBack");
      if (back > 0) step.offset = {frame: "tool", xyz: [0, 0, -back]};
      if ($("stepSpeed").value.trim()) step.speed = num("stepSpeed");
      return step;
    }
    case "wait": return {type: "wait", seconds: num("stepSeconds")};
    case "waitInput": return {type: "wait", input: io, value, timeout: num("stepTimeout")};
    case "set": return {type: "set", output: io, value};
    case "pulse": return {type: "pulse", output: io, seconds: num("stepSeconds")};
    case "halt": return {type: "halt", message: $("stepText").value.trim() || "Halted."};
    default: return {type: "comment", text: $("stepText").value.trim()};
  }
}

$("stepType").addEventListener("change", renderComposer);
const ioLine = choice("stepIoLine");
const ioValue = choice("stepValue");
$("stepAdd").addEventListener("click", (event) => {
  const step = composedStep();
  if (!program) return;
  if (!step) {
    P.say("Record a target first (Teach), then add a move to it.", "bad");
    P.refuse(event.currentTarget);
    return;
  }
  insertStep(step);
});

// ---- Program: the run's log ----
function eventText(e) {
  const at = e.path || "";
  switch (e.event) {
    case "run.start": {
      const how = e.mode === "dry" ? " as a dry run" : e.mode === "step" ? " step by step" : "";
      return `${e.manual ? "Move" : `Run ${e.program}`}${how}${e.from ? ` from step ${e.from}` : ""} at ${e.override}%`;
    }
    case "step.start": return `${at}: ${e.text}`;
    case "step.done": return `${at}: done in ${e.seconds} s`;
    case "step.interrupted": return `${at}: interrupted: ${e.reason}`;
    case "move.planned": return `${at}: ${e.text}`;
    case "move.sent": return `${at}: sent${e.of > 1 ? ` (part ${e.segment} of ${e.of})` : ""}`;
    case "move.arrived": return `${at}: arrived${e.of > 1 ? ` (part ${e.segment} of ${e.of})` : ""} after ${e.seconds} s`;
    case "move.regain": return `${at}: back to the path from ${e.offMm} mm off`;
    case "speed.capped": return `${at}: speed capped at ${e.speed} mm/s (${e.reason})`;
    case "io.wait": return `${at}: waiting for ${e.input} = ${e.value}`;
    case "io.waited": return `${at}: ${e.input} came after ${e.seconds} s`;
    case "io.set": return `${at}: ${e.output} set to ${e.value}`;
    case "dry.assumed": return `${at}: assumed ${e.input} = ${e.value} (dry run)`;
    case "dry.would-set": return `${at}: would set ${e.output} to ${e.value} (dry run)`;
    case "dry.would-pulse": return `${at}: would pulse ${e.output} for ${e.seconds} s (dry run)`;
    case "dry.halt": return `${at}: would halt: ${e.message} (dry run)`;
    case "repeat": return `${at}: repeat ${e.iteration} of ${e.of}`;
    case "if": return `${at}: ${e.input} read ${e.read}, so ${e.branch}`;
    case "call": return `${at}: run ${e.program}`;
    case "run.paused": return `Paused: ${e.reason}`;
    case "run.resumed": return `Resumed (${e.how})`;
    case "run.done": return `Done in ${e.seconds} s${e.estimateS !== undefined ? `; would take about ${e.estimateS} s` : ""}`;
    case "run.stopped": return `Stopped: ${e.reason}`;
    case "run.faulted": return `Faulted: ${e.reason}`;
    case "operator": return `Operator: ${e.action}${e.percent !== undefined ? ` ${e.percent}%` : ""}${e.cause ? ` (${e.cause})` : ""}`;
    default: return e.event;
  }
}

const BAD_EVENTS = ["run.faulted", "run.stopped", "step.interrupted"];

async function refreshEvents(s) {
  if (!s || !s.run) return;
  const last = s.run.lastEvent;
  const key = last ? `${last.t}|${last.event}` : "";
  if (key === lastEventKey || eventsBusy) return;
  eventsBusy = true;
  try {
    const reply = await getJson("/api/run");
    const list = $("events");
    const atBottom = list.scrollTop + list.clientHeight >= list.scrollHeight - 4;
    list.replaceChildren(...reply.run.events.map((e) => {
      const li = el("li", BAD_EVENTS.includes(e.event) ? "bad" : "");
      const time = el("time", "", (e.t || "").slice(11, 19));
      time.dateTime = e.t || "";
      li.append(time, eventText(e));
      return li;
    }));
    if (atBottom) list.scrollTop = list.scrollHeight;
    lastEventKey = key;
  } catch (e) {
    // try again on the next status
  } finally {
    eventsBusy = false;
  }
}

// ---- Teach: freedrive and recording targets ----
// Set before entering freedrive: the controller stops the arm when the sensitivity is written.
const sensitivity = fader($("sensitivityFader"), {
  name: "Teach sensitivity for freedrive", label: "Sensitivity", values: SENSITIVITIES, unit: "%", value: 90,
  home: 90, onChange: () => {},
});

const freedriveKey = $("freedriveKey");
let holdTimer = 0;
function beginFreedrive() {
  if (freedriveKey.disabled || holdTimer) return;
  if (status && status.freedrive) {
    P.say("Leaving freedrive…", "wait");
    send("/api/freedrive", {on: false}, freedriveKey);
    return;
  }
  freedriveKey.classList.add("charging");
  holdTimer = setTimeout(() => {
    holdTimer = 0;
    freedriveKey.classList.remove("charging");
    P.say("Starting freedrive…", "wait");
    send("/api/freedrive", {on: true, sensitivity: sensitivity.value}, freedriveKey);
  }, HOLD_MS);
}
function endFreedriveHold() {
  if (holdTimer) {
    clearTimeout(holdTimer);
    holdTimer = 0;
    P.say("Keep holding until the amber bar fills to start freedrive.", "wait");
  }
  freedriveKey.classList.remove("charging");
}
freedriveKey.addEventListener("pointerdown", (event) => {
  if (event.button !== 0) return;
  event.preventDefault();
  freedriveKey.setPointerCapture(event.pointerId);
  beginFreedrive();
});
for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) freedriveKey.addEventListener(type, endFreedriveHold);
freedriveKey.addEventListener("keydown", (event) => {
  if (event.key !== "Enter") return;
  event.preventDefault();
  if (!event.repeat) beginFreedrive();
});
freedriveKey.addEventListener("keyup", (event) => { if (event.key === "Enter") endFreedriveHold(); });

function renderTeach(s) {
  const live = Boolean(s && s.live);
  const on = Boolean(live && s.freedrive);
  const r = runOf(s);
  freedriveKey.classList.toggle("on", on);
  setText($("freedriveLabel"), on ? "Freedrive on" : "Hold for freedrive");
  freedriveKey.disabled = !(on || (ready(s) && !RUN_BUSY.includes(r.state)));
  const locked = !live ? "Locked: no status" : RUN_BUSY.includes(r.state) ? "Locked: a program is under way"
    : s.stopAlarm ? "Locked: use the E-stop" : s.err ? "Locked: clear the fault first" : !s.enabled ? "Locked: motors off"
    : "Locked: wait for the arm to stop";
  setText($("freedriveSub"), on ? "Press to leave" : freedriveKey.disabled ? locked : "Hold for 0.7 s");
  sensitivity.input.disabled = on;
  if (on && s.teachSensitivity) sensitivity.set(s.teachSensitivity);
  const moving = RUN_MOVING.includes(r.state);
  $("recordKey").disabled = !live || moving;
  const intoOk = Boolean(program) && !(RUN_BUSY.includes(r.state) && r.program === program.name);
  const joint = recordAs.value === "joint";
  $("recordFrame").disabled = joint;
  recordAdd.key("line").disabled = joint || !intoOk;
  recordAdd.key("joint").disabled = !intoOk;
  if (!intoOk && recordAdd.value !== "none") recordAdd.set("none");
  if (joint && recordAdd.value === "line") recordAdd.set("joint");  // a straight line needs a tool pose
  setText($("recordIntoText"), !program ? "(pick a program to add moves to it)"
    : `to it in ${program.name}, ${selected ? `after step ${pathText(selected)}` : "at the end"}`);
  $("recordIntoText").hidden = Boolean(program) && recordAdd.value === "none";
  for (const id of ["toolTouch", "touchOrigin", "touchX", "touchXY"]) $(id).disabled = !live || moving;
}

const recordAs = choice("recordAs", () => renderTeach(status));
const recordAdd = choice("recordAdd", () => renderTeach(status));

// "p1" -> "p2", "pick_09" -> "pick_10", "home" -> "home2".
function nextName(name) {
  const m = /^(.*?)(\d+)$/.exec(name);
  return m ? m[1] + String(Number(m[2]) + 1).padStart(m[2].length, "0") : `${name}2`;
}

$("recordKey").addEventListener("click", async (event) => {
  const key = event.currentTarget;
  const name = $("recordName").value.trim();
  const body = {here: true, name, frame: $("recordFrame").value || "base", kind: recordAs.value, revision: revision()};
  const into = recordAdd.value !== "none" && program;
  if (into) Object.assign(body, {program: program.name, at: selected, move: recordAdd.value});
  const before = into ? program : null;
  const reply = await send("/api/cell/target", body, key);
  if (!reply) return;
  $("recordName").value = nextName(name);
  if (into) {
    const at = selected ? [...selected.slice(0, -1), selected[selected.length - 1] + 1] : [before.steps.length];
    await loadProgram(program.name);
    selected = at;
    renderSteps();
  }
});
$("recordName").addEventListener("keydown", (event) => { if (event.key === "Enter") $("recordKey").click(); });

// ---- Cell: tools, frames, targets and zones ----
function takeCell(reply) {
  cell = reply;
  drawnIn = null;
  renderCell();
  if (program) renderSteps();
  else renderComposer();
}

function frameChoices() {
  return [["base", "base"], ...Object.keys(cell.cell.frames).map((n) => [n, n])];
}

function renderCell() {
  if (!cell) return;
  const data = cell.cell;
  const r = runOf(status);
  const moving = RUN_MOVING.includes(r.state);
  for (const id of ["recordFrame", "frameParent", "zoneFrame"]) fillSelect($(id), frameChoices());
  const counted = (n, one, many) => `${n} ${n === 1 ? one : many}`;
  const tools = Object.keys(data.tools).length;
  setText($("cellSummary"), `${counted(tools, "tool", "tools")} (${data.active.tool} in use), `
    + `${counted(Object.keys(data.frames).length, "frame", "frames")}, `
    + `${counted(Object.keys(data.targets).length, "target", "targets")}, `
    + `${counted(Object.keys(data.zones).length, "keep-out zone", "keep-out zones")} · cell/cell.json, revision ${data.revision}`);

  $("tools").replaceChildren(...Object.entries(data.tools).map(([name, t]) => {
    const bits = [`point ${triple(t.tcp.xyz)} mm`];
    if (t.mass) bits.push(`${fix(t.mass, 2)} kg`);
    if (t.capsules.length) bits.push(`${t.capsules.length === 1 ? "a capsule" : `${t.capsules.length} capsules`} for its shape`);
    if (t.calibration) bits.push(`touch-fit to ${t.calibration.rmsMm} mm RMS`);
    const active = name === data.active.tool;
    const {li, acts} = row(name, bits.join(" · "), active ? "in use" : "");
    acts.append(
      miniKey("Use", `Use tool ${name}`, (key) => send("/api/cell/tool/activate", {name, revision: revision()}, key), active || moving),
      miniKey("Edit", `Edit tool ${name}`, () => editTool(name)),
      miniKey("\u00d7", `Delete tool ${name}`, (key) => twice(key, () => send("/api/cell/tool/delete", {name, revision: revision()}, key)), active),
    );
    return li;
  }));
  const toolName = $("toolName").value.trim();
  const touches = toolName && cell.pending.tools[toolName] || 0;
  setText($("toolTouchCue"), touches
    ? `${touches} touch${touches === 1 ? "" : "es"} for ${toolName}${touches < 4 ? `; ${4 - touches} more to go` : "; press Fit"}. Turn the tool well between them.`
    : "Name the tool, then rest its point on one fixed spike from 4 or more angles, pressing Touch at each.");
  $("toolFit").disabled = touches < 4;
  $("toolTouchClear").disabled = !touches;

  $("frames").replaceChildren(...Object.entries(data.frames).map(([name, f]) => {
    const bits = [`in ${f.parent} · origin ${triple(f.pose.xyz)} mm · turned ${triple(f.pose.rpy)}°`];
    const cue = f.calibration ? `touched ${f.calibration.at.replace("T", " ")}` : "";
    const {li, acts} = row(name, bits.join(""), cue);
    acts.append(
      miniKey("Re-touch", `Re-touch frame ${name}`, () => {
        $("frameName").value = name;
        $("frameParent").value = f.parent;
        renderCell();
        $("touchOrigin").focus();
      }),
      miniKey("\u00d7", `Delete frame ${name}`, (key) => twice(key, () => send("/api/cell/frame/delete", {name, revision: revision()}, key))),
    );
    return li;
  }));
  const frameName = $("frameName").value.trim();
  const pending = (frameName && cell.pending.frames[frameName]) || {};
  const got = ["origin", "x", "xy"].filter((k) => pending[k]);
  const names = {origin: "Origin", x: "+X", xy: "XY"};
  const existing = frameName && data.frames[frameName];
  setText($("frameTouchCue"), got.length
    ? `${frameName}: touched ${got.map((k) => names[k]).join(", ")}${got.length < 3 ? `; still to touch: ${["origin", "x", "xy"].filter((k) => !pending[k]).map((k) => names[k]).join(", ")}` : "; press Fit"}.`
      + (existing ? " Fitting moves its targets and zones with it." : "")
    : existing ? `Re-touching ${frameName}: touch its origin, +X edge and +Y side again, then Fit. Its targets and zones move with it.`
    : "With the tool point, touch the fixture's origin, a point along its +X edge, and a point on its +Y side; then Fit. "
      + "Re-fitting a frame later moves its targets and zones with it.");
  $("frameFit").disabled = got.length < 3;

  const ok = ready(status) && !RUN_MOVING.includes(r.state);
  $("targets").replaceChildren(...Object.entries(data.targets).map(([name, t]) => {
    let detail;
    if (t.joints) {
      detail = `joints ${triple(t.joints)}°`;
    } else {
      detail = `in ${t.frame} · ${triple(t.pose.xyz)} mm · ${triple(t.pose.rpy)}°`;
      if (t.tool !== data.active.tool) detail += ` · taught with ${t.tool}`;
    }
    const {li, acts} = row(name, detail, t.unreachable ? "out of reach since its frame moved" : "", t.unreachable);
    acts.append(
      miniKey("Plan", `Plan a joint move to ${name}`, (key) => planTarget(name, t, key), !ok || t.unreachable),
      miniKey("\u00d7", `Delete target ${name}`, (key) => twice(key, () => send("/api/cell/target/delete", {name, revision: revision()}, key))),
    );
    return li;
  }));

  $("zones").replaceChildren(...Object.entries(data.zones).map(([name, z]) => {
    const {li, acts} = row(name, `in ${z.frame} · centre ${triple(z.pose.xyz, 0)} mm · ${z.size.map((v) => fix(v, 0)).join(" × ")} mm`);
    acts.append(miniKey("\u00d7", `Delete zone ${name}`, (key) => twice(key, () => send("/api/cell/zone/delete", {name, revision: revision()}, key))));
    return li;
  }));
}

// A target's joint move, staged as the panel's plan (the shadow arm, the path check and the plan bar); Move sends it.
// A tool target's joint angles come from the server, solved from the angles it was taught at.
async function planTarget(name, t, key) {
  let joints = t.joints;
  if (!joints) {
    const speeds = {joint: P.settings().jointSpeed, tcp: P.settings().toolSpeed};
    const reply = await send("/api/plan", {target: name, kind: "joint", defaults: speeds}, key);
    if (!reply) return;
    joints = reply.plan.goal;
  }
  if (!P.stage(joints, `Joint move to ${name} planned. Press Move to go there, or Esc to clear the plan.`)) P.refuse(key);
}

function editTool(name) {
  const t = cell.cell.tools[name];
  $("toolName").value = name;
  ["toolX", "toolY", "toolZ"].forEach((id, i) => { $(id).value = fix(t.tcp.xyz[i], 3); });
  $("toolMass").value = fix(t.mass, 3);
  ["toolCx", "toolCy", "toolCz"].forEach((id, i) => { $(id).value = fix(t.cog[i], 1); });
  const c = t.capsules[0];
  $("toolLength").value = c ? fix(Math.hypot(...c.b.map((v, i) => v - c.a[i])), 1) : "0";
  $("toolRadius").value = c ? fix(c.r, 1) : "0";
  renderCell();
  $("toolName").focus();
}

// A tool's shape from the form: one capsule from the flange towards the tool point (or along z, for a tool point
// at the flange). A tool given more capsules through the API keeps them unless the form's shape changes.
function shapeFromForm(old) {
  const length = num("toolLength");
  const radius = num("toolRadius");
  if (old && old.capsules.length > 1) {
    const c = old.capsules[0];
    const same = Math.abs(Math.hypot(...c.b.map((v, i) => v - c.a[i])) - length) < 0.05 && Math.abs(c.r - radius) < 0.05;
    if (same) return old.capsules;
  }
  if (!(length > 0 && radius > 0)) return [];
  const point = [num("toolX"), num("toolY"), num("toolZ")];
  const reach = Math.hypot(...point);
  const along = reach > 1 ? point.map((v) => v / reach) : [0, 0, 1];
  return [{a: [0, 0, 0], b: along.map((v) => Math.round(v * length * 10) / 10), r: radius}];
}

$("toolSave").addEventListener("click", (event) => {
  const name = $("toolName").value.trim();
  const old = cell && cell.cell.tools[name];
  send("/api/cell/tool", {
    name, tcp: {xyz: [num("toolX"), num("toolY"), num("toolZ")], rpy: old ? old.tcp.rpy : [0, 0, 0]},
    mass: num("toolMass"), cog: [num("toolCx"), num("toolCy"), num("toolCz")], capsules: shapeFromForm(old),
    revision: revision(),
  }, event.currentTarget);
});
$("toolTouch").addEventListener("click", (e) => send("/api/cell/tool/touch", {name: $("toolName").value.trim()}, e.currentTarget));
$("toolFit").addEventListener("click", (e) => send("/api/cell/tool/calibrate", {name: $("toolName").value.trim(), revision: revision()}, e.currentTarget));
$("toolTouchClear").addEventListener("click", (e) => send("/api/cell/tool/touch/clear", {name: $("toolName").value.trim()}, e.currentTarget));
$("toolName").addEventListener("input", renderCell);

for (const [id, which] of [["touchOrigin", "origin"], ["touchX", "x"], ["touchXY", "xy"]]) {
  $(id).addEventListener("click", (e) => send("/api/cell/frame/touch",
    {name: $("frameName").value.trim(), which, parent: $("frameParent").value || "base"}, e.currentTarget));
}
$("frameFit").addEventListener("click", (e) => send("/api/cell/frame/calibrate",
  {name: $("frameName").value.trim(), revision: revision()}, e.currentTarget));
$("frameName").addEventListener("input", renderCell);

$("zoneSave").addEventListener("click", (e) => send("/api/cell/zone", {
  name: $("zoneName").value.trim(), frame: $("zoneFrame").value || "base",
  pose: {xyz: [num("zoneX"), num("zoneY"), num("zoneZ")], rpy: [0, 0, 0]},
  size: [num("zoneW"), num("zoneD"), num("zoneH")], revision: revision(),
}, e.currentTarget));

// ---- every status ----
let keysShown = "";
let runSeen = null;  // the run's state and step at the last status, to say on the message line when they change

// When a run stops, pauses, halts, faults or finishes, the message line says so (it may still say "Running ...").
function sayRunChange(r) {
  const key = `${r.state}|${r.path ? pathText(r.path) : ""}|${r.halt || ""}`;
  if (runSeen === null || key === runSeen) {
    runSeen = key;
    return;
  }
  runSeen = key;
  const what = r.manual ? "The move" : r.program;
  const at = r.path ? ` at step ${pathText(r.path)}` : "";
  const reason = (r.reason || "").replace(/\.+$/, "");
  if (r.state === "paused" && r.halt) P.say(`${what} halted${at}: ${r.halt}`, "wait");
  else if (r.state === "paused" && r.reason !== "stepped") P.say(`${what} paused${at}: ${reason}.`, "wait");
  else if (r.state === "faulted") P.say(`${what} faulted: ${reason}.`, "bad");
  else if (r.state === "stopped") P.say(`${what} stopped: ${reason || "stopped"}.`, "wait");
  else if (r.state === "done" && !r.manual) P.say(`${what} finished${r.mode === "dry" ? " planning; nothing moved" : ""}.`, "ok");
}

function onStatus(s) {
  status = s;
  if (s && s.cellRevision !== undefined && (!cell || cell.cell.revision !== s.cellRevision) && cellAsked !== s.cellRevision) {
    cellAsked = s.cellRevision;
    getJson("/api/cell").then(takeCell).catch(() => { cellAsked = null; });
  }
  const r = runOf(s);
  const view = P.view();
  if (view && view.setCell && cell && drawnIn !== view) {
    view.setCell({collision: cell.collision, ...cell.scene});
    drawnIn = view;
  }
  if (view && view.setFault) view.setFault(r.state === "faulted" ? r.hit : null);
  if (s && s.run) sayRunChange(r);
  renderRun(s);
  renderTeach(s);
  markCurrent();
  refreshEvents(s);
  // Keys whose state follows the run and the arm's readiness: redraw the lists when either changes.
  const keys = `${r.state}|${r.program}|${ready(s)}|${Boolean(s && s.live)}`;
  if (keys !== keysShown) {
    keysShown = keys;
    if (program) renderSteps();
    renderCell();
    renderComposer();
  }
}

P.listeners.push(onStatus);
renderComposer();
loadPrograms();
