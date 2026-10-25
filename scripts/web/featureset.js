// The jog panel's Program, Teach and Cell sections: running programs on the server (Run, Step, Dry run, Pause,
// Stop run, Reset, the speed override and safe home), editing their steps (retries and fallbacks included), the calls
// a run makes to the owner, teaching targets in freedrive and touching them up, and setting up the cell's tools,
// grippers, frames, targets, patterns, interlocks and zones. The server keeps and checks all of it
// (docs/programs.md); this module shows it and sends the operator's requests. It builds on the panel's own script
// through window.jogPanel and runs after it.
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
let io = null;  // GET /api/io's last reply: the digital lines, and the interlocks and grippers as they read
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
  const brain = s && s.brain;
  if (brain && brain.ok === false && needsBrain(program.steps)) {
    return brain.url ? `${program.name} needs the brain, which isn't answering at ${brain.url}: start it `
      + "(python brain/stub/stub_brain.py)." : `${program.name} needs the brain, which is off (--brain off).`;
  }
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
  const why = have && RUN_ENDED.includes(r.state) ? whyNotStart(s, "dry") || (ready(s) ? null : whyNotReady(s)) : null;
  if (why) sentence += ` ${why}`;
  setText($("runWord"), word);
  setText($("runSentence"), sentence);
  $("runWindow").dataset.tone = tone;

  const live = Boolean(s && s.live);
  const paused = r.state === "paused";
  gate($("runStart"), whyNotStart(s, "run"));
  gate($("runStepKey"), paused ? (live ? null : "There's no status from the controller.") : whyNotStart(s, "step"));
  gate($("runDry"), whyNotStart(s, "dry"));
  gate($("homeKey"), whyNotHome(s));
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
  else send("/api/run/start", {name: program.name, mode: "step", from: selected && !selected.into ? selected.path : null},
    e.currentTarget);
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
const GRIP_WORDS = {open: "Open", close: "Close", grasp: "Grasp with"};

function moveTo(s) {
  if (s.pattern === undefined) return s.target;
  return s.slot !== undefined ? `${s.pattern} slot ${s.slot}` : `${s.pattern}'s next slot`;
}

function describeStep(s) {
  switch (s.type) {
    case "move": {
      let text = `${s.kind === "line" ? "Straight move" : s.kind === "planned" ? "Planned move" : "Joint move"} to ${moveTo(s)}`;
      const o = s.offset;
      if (o && o.frame === "tool" && !o.xyz[0] && !o.xyz[1] && o.xyz[2] < 0 && !(o.rpy || []).some(Boolean)) {
        text += `, ${fix(-o.xyz[2], 0)} mm back along the tool`;
      } else if (o) {
        text += `, offset ${triple(o.xyz, 0)} mm in the ${o.frame === "tool" ? "tool's" : "target's"} frame`;
      }
      if (s.speed !== undefined) text += ` at ${fix(s.speed, 0)} ${s.kind === "line" ? "mm/s" : "°/s"}`;
      return text;
    }
    case "learned": {
      let text = `Learned skill ${s.skill} near ${s.near}, within ${fix(s.within, 0)} mm, up to ${fix(s.seconds, 0)} s`;
      if (s.speed !== undefined) text += ` at ${fix(s.speed, 0)} °/s`;
      if (s.check) text += `, then ${s.check.input} = ${s.check.value}`;
      return text;
    }
    case "gripper": {
      const mass = s.part && s.part.mass;
      return `${GRIP_WORDS[s.action] || s.action} ${s.gripper}${s.action === "grasp" && mass ? `, a ${fix(mass, 2)} kg part` : ""}`;
    }
    case "count":
      if (s.to === undefined) return `Count ${s.pattern} on to its next slot`;
      return s.to === 1 ? `Count ${s.pattern} back to slot 1` : `Set ${s.pattern}'s counter to slot ${s.to}`;
    case "retry": return `Retry: up to ${s.tries} tries of the steps below`;
    case "fallback": return "Fallback: if a step below fails, the steps under If it fails run instead";
    case "home":
      return `Safe home to ${s.target}${s.retract ? `, first up to ${fix(s.retract, 0)} mm back along the tool` : ""}`;
    case "notify": return `Call the owner: ${s.message}`;
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

// Every step as a row: its path as the server numbers it (a fallback's If it fails steps carry on from its own),
// its depth, `at` (where it lives in the program: indexes and the keys of the lists between them, for editing) and
// whether this page edits it (the top level, repeats, retries and fallbacks; an if's branches are edited in its JSON
// file). Label rows: "Else" for an if; "If it fails" for a fallback, and an empty list's row, which can be selected
// to add steps into that list (`into`).
function flatten(steps, path = [], depth = 0, editable = true, out = [], at = [], offset = 0) {
  steps.forEach((s, i) => {
    const p = [...path, offset + i];
    const here = [...at, i];
    out.push({s, path: p, depth, editable, at: here});
    if (s.type === "repeat" || s.type === "retry") {
      flatten(s.steps, p, depth + 1, editable, out, [...here, "steps"]);
      if (!s.steps.length && editable) out.push({label: "No steps yet: select this, then add them", depth: depth + 1, into: [...here, "steps"]});
    }
    if (s.type === "fallback") {
      flatten(s.steps, p, depth + 1, editable, out, [...here, "steps"]);
      if (!s.steps.length && editable) out.push({label: "No steps yet: select this, then add them", depth: depth + 1, into: [...here, "steps"]});
      out.push({label: "If it fails", depth, into: editable ? [...here, "else"] : null});
      flatten(s.else, p, depth + 1, editable, out, [...here, "else"], s.steps.length);
    }
    if (s.type === "if") {
      flatten(s.then, p, depth + 1, false, out, [...here, "then"]);
      if (s.else.length) {
        out.push({label: "Else", depth});
        flatten(s.else, p, depth + 1, false, out, [...here, "else"]);
      }
    }
  });
  return out;
}

// The list a location's step lives in, and its index: `at` alternates indexes and list keys, ending with an index.
function listAt(steps, at) {
  let list = steps;
  for (let k = 0; k + 1 < at.length; k += 2) list = list[at[k]][at[k + 1]];
  return [list, at[at.length - 1]];
}

// The list named by `into` (indexes and keys, ending with a key).
function listInto(steps, into) {
  let list = steps;
  for (let k = 0; k < into.length; k += 2) list = list[into[k]][into[k + 1]];
  return list;
}

const sameAt = (a, b) => Boolean(a && b) && a.length === b.length && a.every((v, i) => v === b[i]);

// What the brain does for a step ("" for steps it has no part in).
function brainPart(s) {
  if (s.type === "learned") return "the brain acts";
  return s.type === "move" && s.kind === "planned" ? "the brain plans the path" : "";
}

// A step's problem, shown in Alarm Red: a target, gripper or pattern that's gone, a target out of reach, a pattern used
// up, or a brain it needs that isn't there.
function stepProblem(s) {
  if (!cell) return "";
  const data = cell.cell;
  if (s.type === "gripper" && !data.grippers[s.gripper]) return "no such gripper";
  if (s.pattern !== undefined && (s.type === "move" || s.type === "count")) {
    const info = cell.patterns[s.pattern];
    if (!info) return "no such pattern";
    if (info.error) return info.error;
    const next = (status && status.counters && status.counters[s.pattern]) || 1;
    if (s.type === "move" && s.slot === undefined && next > info.count) return `${s.pattern} is used up`;
    return "";
  }
  const name = s.type === "move" || s.type === "home" ? s.target : s.type === "learned" ? s.near : null;
  if (!name) return "";
  const target = data.targets[name];
  if (!target) return "no such target";
  if (target.unreachable) return "out of reach since its frame moved";
  const brain = status && status.brain;
  if (!brainPart(s) || !brain || brain.ok === null || brain.ok === undefined) return "";
  if (!brain.ok) return "brain not answering";
  if (s.type === "learned" && !(s.skill in (brain.skills || {}))) return `the brain has no skill ${s.skill}`;
  return "";
}

// A step's note in Printed Grey: the brain's part, or where a pattern's counter stands.
function stepCue(s) {
  if (brainPart(s)) return brainPart(s);
  if (s.type === "move" && s.pattern !== undefined && s.slot === undefined && cell && cell.patterns[s.pattern]) {
    const next = (status && status.counters && status.counters[s.pattern]) || 1;
    return `slot ${next} of ${cell.patterns[s.pattern].count} next`;
  }
  return "";
}

// Whether a program's own steps (not the programs it calls) need the brain.
function needsBrain(steps) {
  return steps.some((s) => brainPart(s) || needsBrain([...(s.steps || []), ...(s.then || []), ...(s.else || [])]));
}

function renderSteps() {
  const list = $("steps");
  const r = runOf(status);
  const busy = Boolean(program) && RUN_BUSY.includes(r.state) && r.program === program.name;
  const rows = program ? flatten(program.steps) : [];
  if (selected && !rows.some((x) => (selected.into ? sameAt(x.into, selected.into) : x.s && sameAt(x.at, selected.at)))) {
    selected = null;
  }
  list.replaceChildren(...rows.map((x) => {
    const li = el("li", "row");
    li.dataset.depth = String(Math.min(x.depth, 3));
    if (x.label) {
      li.classList.add("comment");
      if (!x.into) {
        li.append(el("span"), el("span", "pick text", x.label), el("span"));
        return li;
      }
      // A row that takes new steps: selecting it adds them at the start of that list.
      li.classList.add("slot");
      const pick = el("button", "pick");
      pick.type = "button";
      pick.append(el("span", "num", ""), el("span", "text", x.label));
      const isSelected = Boolean(selected && sameAt(selected.into, x.into));
      li.classList.toggle("selected", isSelected);
      pick.setAttribute("aria-pressed", String(isSelected));
      pick.addEventListener("click", () => {
        selected = isSelected ? null : {into: x.into};
        renderSteps();
      });
      li.append(el("span"), pick, el("span", "acts"));
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
    else if (stepCue(x.s)) pick.lastChild.append(el("span", "cue", ` · ${stepCue(x.s)}`));
    const isSelected = Boolean(selected && !selected.into && sameAt(x.at, selected.at));
    li.classList.toggle("selected", isSelected);
    pick.setAttribute("aria-pressed", String(isSelected));
    pick.addEventListener("click", () => {
      selected = isSelected ? null : {at: x.at, path: x.path};
      renderSteps();
    });
    const acts = el("span", "acts");
    const where = `step ${pathText(x.path)}`;
    const start = miniKey("Run from", `Run from ${where}`, (key) => send("/api/run/start",
      {name: program.name, mode: "run", from: x.path}, key), busy || !ready(status));
    acts.append(start);
    if (x.editable) {
      const [container, i] = listAt(program.steps, x.at);
      acts.append(
        miniKey("\u2191", `Move ${where} up`, () => moveStep(x.at, -1), busy || i === 0),
        miniKey("\u2193", `Move ${where} down`, () => moveStep(x.at, 1), busy || i === container.length - 1),
        miniKey("\u00d7", `Delete ${where}`, (key) => twice(key, () => deleteStep(x.at)), busy),
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

async function saveProgram(next, select) {
  const reply = await send("/api/program", {program: next, revision: program.revision});
  if (!reply) return false;
  program = reply.program;
  selected = select;
  renderSteps();
  return true;
}

// The selection after an edit: the step at `at`, with its path as the server numbers it.
function selectAt(at) {
  const row = flatten(program.steps).find((x) => x.s && sameAt(x.at, at));
  return row ? {at: row.at, path: row.path} : null;
}

async function saveAndSelect(next, at) {
  const reply = await send("/api/program", {program: next, revision: program.revision});
  if (!reply) return false;
  program = reply.program;
  selected = selectAt(at);
  renderSteps();
  return true;
}

function moveStep(at, by) {
  const next = structuredClone(program);
  const [list, i] = listAt(next.steps, at);
  [list[i], list[i + by]] = [list[i + by], list[i]];
  saveAndSelect(next, [...at.slice(0, -1), i + by]);
}

function deleteStep(at) {
  const next = structuredClone(program);
  const [list, i] = listAt(next.steps, at);
  list.splice(i, 1);
  saveProgram(next, null);
}

// Adds a step after the selected one, or at the start of the selected empty list or If it fails, or at the end.
function insertStep(step) {
  const next = structuredClone(program);
  if (selected && selected.into) {
    listInto(next.steps, selected.into).unshift(step);
    return saveAndSelect(next, [...selected.into, 0]);
  }
  if (selected) {
    const [list, i] = listAt(next.steps, selected.at);
    list.splice(i + 1, 0, step);
    return saveAndSelect(next, [...selected.at.slice(0, -1), i + 1]);
  }
  next.steps.push(step);
  return saveAndSelect(next, [next.steps.length - 1]);
}

// Where the next step goes, in words: "after 3", "into 4" (an empty list), "under If it fails of 4", "at the end".
function insertWhere() {
  if (!selected) return "at the end";
  if (selected.into) {
    const row = flatten(program.steps).find((x) => x.s && sameAt(x.at, selected.into.slice(0, -1)));
    const of = row ? pathText(row.path) : "";
    return selected.into[selected.into.length - 1] === "else" ? `under If it fails of ${of}` : `into ${of}`;
  }
  return `after ${pathText(selected.path)}`;
}

// ---- Program: adding a step ----
const SHOWN = {
  joint: ["stepTarget", "back", "speed"], line: ["stepTarget", "back", "speed"], planned: ["stepTarget", "back", "speed"],
  learned: ["near", "stepTarget", "stepSkill", "learned", "speed"], wait: ["seconds"],
  waitInput: ["stepIoLine", "stepIoBit", "stepValue", "timeout"], set: ["stepIoLine", "stepIoBit", "stepValue"],
  pulse: ["stepIoLine", "stepIoBit", "seconds"], halt: ["stepText"], comment: ["stepText"],
  gripper: ["stepGrip", "stepGripper", "part"], count: ["stepPattern", "stepCount"], retry: ["tries"], fallback: [],
  home: ["stepTarget", "back", "speed"], notify: ["stepText"],
};
const PATTERN = "pattern:";  // a move's target select lists patterns too, as "pattern:<name>"

function renderComposer() {
  const type = $("stepType").value;
  const shown = SHOWN[type];
  for (const id of ["stepTarget", "stepSkill", "stepIoLine", "stepIoBit", "stepValue", "stepText", "stepGrip",
    "stepGripper", "stepPattern", "stepCount"]) {
    $(id).hidden = !shown.includes(id);
  }
  for (const node of $("composer").querySelectorAll("[data-for]")) node.hidden = !shown.includes(node.dataset.for);
  $("stepPart").closest("label").hidden = !(type === "gripper" && grip.value === "grasp");
  setText($("stepSpeedUnit"), type === "line" ? "mm/s" : "°/s");
  $("stepSpeed").max = type === "line" ? "1000" : "120";
  $("stepSpeed").placeholder = program ? fix(program.speed[type === "line" ? "tcp" : "joint"], 0) : "";
  $("stepText").placeholder = type === "halt" ? "what the operator should do" : type === "notify" ? "what the owner should know" : "note";
  $("stepTarget").setAttribute("aria-label", type === "learned" ? "Near target" : type === "home" ? "Home target" : "Target");
  const data = cell ? cell.cell : {targets: {}, grippers: {}, patterns: {}};
  const targets = Object.entries(data.targets);
  const moves = ["joint", "line", "planned"].includes(type);
  fillSelect($("stepTarget"), [
    ...targets.filter(([, t]) => type !== "line" || !t.joints).map(([name, t]) => [name, t.joints ? `${name} (joints)` : name]),
    ...(moves ? Object.keys(data.patterns).map((name) => [PATTERN + name, `${name}'s next slot (pattern)`]) : []),
  ], type === "line" ? "No tool targets yet" : "No targets yet");
  fillSelect($("stepGripper"), Object.keys(data.grippers).map((name) => [name, name]), "No grippers yet");
  fillSelect($("stepPattern"), Object.keys(data.patterns).map((name) => [name, name]), "No patterns yet");
  const brain = status && status.brain;
  const skills = brain && brain.ok ? Object.keys(brain.skills || {}) : [];
  fillSelect($("stepSkill"), skills.map((name) => [name, name]), brain && brain.ok ? "The brain has no skills" : "No brain");
  const r = runOf(status);
  const busy = Boolean(program) && RUN_BUSY.includes(r.state) && r.program === program.name;
  $("stepAdd").disabled = !program || busy;
  setText($("stepAdd"), !program || !selected ? "Add at the end" : selected.into ? `Add ${insertWhere()}` : `Add after ${pathText(selected.path)}`);
}

// The step the add row describes, or {why} it can't be added yet.
function composedStep() {
  const type = $("stepType").value;
  const io = `${ioLine.value}.${Math.round(num("stepIoBit"))}`;
  const value = Number(ioValue.value);
  const speed = $("stepSpeed").value.trim() ? num("stepSpeed") : undefined;
  switch (type) {
    case "joint":
    case "line":
    case "planned": {
      const to = $("stepTarget").value;
      if (!to) return {why: "Record a target first (Teach), then add a move to it."};
      const step = {type: "move", kind: type};
      if (to.startsWith(PATTERN)) step.pattern = to.slice(PATTERN.length);
      else step.target = to;
      const back = num("stepBack");
      if (back > 0) step.offset = {frame: "tool", xyz: [0, 0, -back]};
      if (speed !== undefined) step.speed = speed;
      return step;
    }
    case "learned": {
      if (!$("stepTarget").value) return {why: "Record a target first (Teach): a learned step works near one."};
      if (!$("stepSkill").value) {
        return {why: "Start the brain (python brain/stub/stub_brain.py) to list its skills, then add a learned step."};
      }
      const step = {type: "learned", skill: $("stepSkill").value, near: $("stepTarget").value, within: num("stepWithin"),
        seconds: num("stepLimit")};
      if (speed !== undefined) step.speed = speed;
      return step;
    }
    case "gripper": {
      if (!$("stepGripper").value) return {why: "Add a gripper in the Cell section first: name it, and the output that closes it."};
      const step = {type: "gripper", gripper: $("stepGripper").value, action: grip.value};
      if (grip.value === "grasp") step.part = {mass: num("stepPart"), cog: [0, 0, 0]};
      return step;
    }
    case "count": {
      if (!$("stepPattern").value) return {why: "Add a pattern in the Cell section first, from three taught corners."};
      return countWay.value === "first" ? {type: "count", pattern: $("stepPattern").value, to: 1}
        : {type: "count", pattern: $("stepPattern").value};
    }
    case "retry": return {type: "retry", tries: Math.round(num("stepTries")), steps: []};
    case "fallback": return {type: "fallback", steps: [], else: []};
    case "home": {
      const to = $("stepTarget").value;
      if (!to || to.startsWith(PATTERN)) return {why: "Pick the target to go home to (record one in Teach first)."};
      const step = {type: "home", target: to, retract: num("stepBack")};
      if (speed !== undefined) step.speed = speed;
      return step;
    }
    case "notify": return {type: "notify", message: $("stepText").value.trim() || "The run needs you."};
    case "wait": return {type: "wait", seconds: num("stepSeconds")};
    case "waitInput": return {type: "wait", input: io, value, timeout: num("stepTimeout")};
    case "set": return {type: "set", output: io, value};
    case "pulse": return {type: "pulse", output: io, seconds: num("stepSeconds")};
    case "halt": return {type: "halt", message: $("stepText").value.trim() || "Halted."};
    default: return {type: "comment", text: $("stepText").value.trim()};
  }
}

$("stepType").addEventListener("change", () => {
  // Safe home backs off along the tool first: 50 mm unless asked otherwise; moves start without an approach offset.
  if ($("stepType").value === "home" && num("stepBack") === 0) $("stepBack").value = "50";
  renderComposer();
});
const ioLine = choice("stepIoLine");
const ioValue = choice("stepValue");
const grip = choice("stepGrip", () => renderComposer());
const countWay = choice("stepCount");
$("stepAdd").addEventListener("click", async (event) => {
  const step = composedStep();
  if (!program) return;
  if (step.why) {
    P.say(step.why, "bad");
    P.refuse(event.currentTarget);
    return;
  }
  const container = step.type === "retry" || step.type === "fallback";
  if (await insertStep(step) && container) {
    // Straight into the new container's list, so the next steps added go there.
    const row = flatten(program.steps).find((x) => x.s && selected && sameAt(x.at, selected.at));
    if (row) {
      selected = {into: [...row.at, "steps"]};
      renderSteps();
      P.say(`${describeStep(step)}: the steps added next go into it. Select a step outside it to add after that instead.`, "ok");
    }
  }
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
    case "dry.assumed":
      if (e.zone) return `${at}: assumed ${e.interlocks.join(" and ")} will let it into the ${e.zone} zone (dry run)`;
      return e.skill ? `${at}: assumed ${e.skill} ends where it began (dry run)` : `${at}: assumed ${e.input} = ${e.value} (dry run)`;
    case "brain.plan": return `${at}: ${e.text}${e.note ? ` (the brain: ${e.note})` : ""}`;
    case "brain.refused": return `${at}: ${e.reason}`;
    case "step.failed": return `${at}: failed: ${e.reason}`;
    case "retry": return `${at}: try ${e.attempt} of ${e.of}, after ${e.reason}`;
    case "retry.spent": return `${at}: no tries left of ${e.tries}`;
    case "fallback": return `${at}: a step failed, so the steps under If it fails run`;
    case "gripper.set": {
      const outs = Object.entries(e.outputs || {}).map(([k, v]) => `${k} ${v ? "on" : "off"}`).join(", ");
      return `${at}: ${e.action === "open" ? "opening" : e.action === "grasp" ? "grasping with" : "closing"} ${e.gripper} (${outs})`;
    }
    case "gripper.done":
      return `${at}: ${e.gripper} ${e.action === "open" ? "opened" : e.action === "grasp" ? "holds the part" : "closed"} in ${e.seconds} s`
        + `${e.payload !== undefined ? `; payload ${e.payload} kg` : ""}${e.note && /stopped/.test(e.note) ? `. ${e.note}` : ""}`;
    case "dry.would-grip": return `${at}: would ${e.action === "grasp" ? "grasp with" : e.action} ${e.gripper} (dry run; ${e.note})`;
    case "pattern.count":
      return `${at}: ${e.pattern} counted to ${e.slot > e.of ? `past its last slot (used up)` : `slot ${e.slot} of ${e.of}`}`;
    case "home.retract":
      return `${at}: ${e.gotMm >= e.wantedMm ? `back ${e.gotMm} mm along the tool` : e.gotMm > 0
        ? `back ${e.gotMm} of ${e.wantedMm} mm along the tool: ${e.short}` : `no room to back off: ${e.short}`}`;
    case "owner.called": return `${at}: called the owner: ${e.message}`;
    case "dry.would-notify": return `${at}: would call the owner: ${e.message} (dry run)`;
    case "skill.start": return `${at}: ${e.skill} starts, within ${e.within} mm of ${e.near}, for up to ${e.seconds} s`;
    case "skill.burst": return `${at}: burst ${e.tick + 1}: ${e.text}`;
    case "skill.hold": return `${at}: ${e.seconds} s still, as the skill asked`;
    case "skill.done": return `${at}: ${e.skill} done after ${e.bursts} burst${e.bursts === 1 ? "" : "s"}${e.note ? `: ${e.note}` : ""}`;
    case "skill.end": return `${at}: ${e.skill} ended (${e.outcome})`;
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

const BAD_EVENTS = ["run.faulted", "run.stopped", "step.interrupted", "brain.refused", "step.failed", "retry.spent"];

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
  const where = program ? insertWhere() : "";
  setText($("recordIntoText"), !program ? "(pick a program to add moves to it)"
    : `to it in ${program.name}, ${where.startsWith("after ") ? `after step ${where.slice(6)}` : where}`);
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
  const move = recordAdd.value;
  const reply = await send("/api/cell/target", body, key);
  if (!reply) return;
  $("recordName").value = nextName(name);
  if (into) {
    // The move goes where the add row's would: after the selected step, into a selected empty list, or at the end.
    await loadProgram(program.name);
    if (await insertStep({type: "move", kind: move, target: name})) {
      P.say(`${reply.message} Added a ${move === "line" ? "straight" : "joint"} move to it in ${program.name}.`, "ok");
    }
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
  pollIo();
}

function frameChoices() {
  return [["base", "base"], ...Object.keys(cell.cell.frames).map((n) => [n, n])];
}

// Every digital line, for a gripper's inputs: "none", then tool.0 to tool.15 and controller.0 to controller.15.
const LINES = ["tool", "controller"].flatMap((line) => [...Array(16).keys()].map((n) => `${line}.${n}`));
function inputChoices() {
  return [["", "none"], ...LINES.map((name) => [name, name])];
}

const signalText = (s, on = "on", off = "off") => `${s.input || s.output} ${s.value ? on : off}`;

function gripperDetail(g) {
  const close = g.close.map((o) => signalText(o)).join(" and ");
  const bits = [`closes with ${close}`];
  const inputs = [["closed", "closed"], ["opened", "open"], ["part", "a part"]].filter(([k]) => g[k])
    .map(([k, what]) => `${what} on ${signalText(g[k], "", "off").trim()}`);
  bits.push(inputs.length ? `confirms ${inputs.join(", ")} (within ${fix(g.timeout, 1)} s)` : `no inputs: waits ${fix(g.seconds, 1)} s`);
  return bits.join(" · ");
}

// What /api/io last said of a gripper: its outputs closing it, its inputs and the part it holds.
function gripperCue(name) {
  const g = io && io.grippers && io.grippers[name];
  if (!g) return "";
  const parts = [g.outputs === "closed" ? "closed" : g.outputs === "open" ? "open" : "outputs between"];
  if (g.held) parts.push(`holding ${fix(g.held.mass, 2)} kg`);
  else if (g.part) parts.push("a part in it");
  return parts.join(", ");
}

function patternDetail(name, p, info) {
  const size = `${p.rows} × ${p.columns}${p.layers > 1 ? ` × ${p.layers}` : ""}`;
  if (!info || info.error) return `${size} · from ${p.first}`;
  const [along, down, up] = info.pitchMm;
  const bits = [`${size} in ${info.frame}`];
  if (p.columns > 1) bits.push(`columns ${fix(along)} mm apart`);
  if (p.rows > 1) bits.push(`rows ${fix(down)} mm apart${info.angleDeg !== null && Math.abs(info.angleDeg - 90) >= 0.5 ? ` at ${fix(info.angleDeg)}°` : ""}`);
  if (p.layers > 1) bits.push(`layers ${fix(up)} mm apart`);
  return bits.join(" · ");
}

function renderCell() {
  if (!cell) return;
  const data = cell.cell;
  const r = runOf(status);
  const moving = RUN_MOVING.includes(r.state);
  const busy = RUN_BUSY.includes(r.state);
  for (const id of ["recordFrame", "frameParent", "zoneFrame"]) fillSelect($(id), frameChoices());
  const counted = (n, one, many) => `${n} ${n === 1 ? one : many}`;
  const tools = Object.keys(data.tools);
  const n = (key) => Object.keys(data[key]).length;
  setText($("cellSummary"), [`${counted(tools.length, "tool", "tools")} (${data.active.tool} in use)`,
    n("grippers") ? counted(n("grippers"), "gripper", "grippers") : "", counted(n("frames"), "frame", "frames"),
    counted(n("targets"), "target", "targets"), n("patterns") ? counted(n("patterns"), "pattern", "patterns") : "",
    n("interlocks") ? counted(n("interlocks"), "interlock", "interlocks") : "",
    counted(n("zones"), "zone", "zones")].filter(Boolean).join(", ")
    + ` · cell/cell.json, revision ${data.revision}`);

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

  // Grippers: what closes each and what confirms it, its state as last read, and the panel's own Open, Close and
  // Grasp (a grasp from here holds a part of unknown mass: the payload stays the tool's).
  const live = Boolean(status && status.live);
  $("grippers").replaceChildren(...Object.entries(data.grippers).map(([name, g]) => {
    const {li, acts} = row(name, gripperDetail(g), gripperCue(name));
    const act = (action) => (key) => send("/api/gripper", {name, action}, key).then(() => pollIo(true));
    acts.append(
      miniKey("Open", `Open ${name}`, act("open"), busy || !live),
      miniKey("Close", `Close ${name}`, act("close"), busy || !live),
      miniKey("Grasp", `Grasp with ${name}, checking for a part`, act("grasp"), busy || !live),
      miniKey("Edit", `Edit gripper ${name}`, () => editGripper(name)),
      miniKey("\u00d7", `Delete gripper ${name}`, (key) => twice(key, () => send("/api/cell/gripper/delete", {name, revision: revision()}, key))),
    );
    return li;
  }));
  for (const id of ["gripClosed", "gripOpened", "gripPart"]) fillSelect($(id), inputChoices());

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

  // Targets: Plan stages a joint move to one; Touch up re-teaches it where the arm is now, and the row then shows how
  // far it moved, with Undo.
  const ok = ready(status) && !RUN_MOVING.includes(r.state);
  $("targets").replaceChildren(...Object.entries(data.targets).map(([name, t]) => {
    let detail;
    if (t.joints) {
      detail = `joints ${triple(t.joints)}°`;
    } else {
      detail = `in ${t.frame} · ${triple(t.pose.xyz)} mm · ${triple(t.pose.rpy)}°`;
      if (t.tool !== data.active.tool) detail += ` · taught with ${t.tool}`;
    }
    const change = t.touchUp;
    const cue = t.unreachable ? "out of reach since its frame moved"
      : change ? `touched up ${change.at.slice(11, 16)}: ${change.movedMm !== undefined
        ? `moved ${fix(change.movedMm, 2)} mm, turned ${fix(change.turnedDeg, 2)}°` : `joints by up to ${fix(change.jointsMaxDeg, 2)}°`}`
      : "";
    const {li, acts} = row(name, detail, cue, t.unreachable);
    acts.append(miniKey("Plan", `Plan a joint move to ${name}`, (key) => planTarget(name, t, key), !ok || t.unreachable));
    acts.append(miniKey("Touch up", `Touch up ${name}: re-teach it where the arm is now`, (key) => send(
      "/api/cell/target/touchup", {name, revision: revision()}, key), !live || moving || (t.tool && t.tool !== data.active.tool)));
    if (t.previous) {
      acts.append(miniKey("Undo", `Undo ${name}'s touch-up`, (key) => send("/api/cell/target/undo", {name, revision: revision()}, key)));
    }
    acts.append(miniKey("\u00d7", `Delete target ${name}`, (key) => twice(key, () => send("/api/cell/target/delete", {name, revision: revision()}, key))));
    return li;
  }));

  // Patterns: their size and pitches, where their counter stands, and Plan (its next slot), Reset (back to slot 1).
  const counters = (status && status.counters) || cell.counters || {};
  $("patterns").replaceChildren(...Object.entries(data.patterns).map(([name, p]) => {
    const info = cell.patterns[name] || {};
    const next = counters[name] || 1;
    const usedUp = !info.error && next > info.count;
    const cue = info.error ? info.error : usedUp ? `used up: all ${info.count} slots done` : `next: slot ${next} of ${info.count}`;
    const {li, acts} = row(name, patternDetail(name, p, info), cue, Boolean(info.error) || usedUp);
    acts.append(
      miniKey("Plan", `Plan a joint move to ${name}'s next slot`, (key) => planSlot(name, key), !ok || usedUp || Boolean(info.error)),
      miniKey("Reset", `Count ${name} back to slot 1`, (key) => send("/api/cell/pattern/counter", {name, slot: 1}, key), moving || next === 1),
      miniKey("Edit", `Edit pattern ${name}`, () => editPattern(name)),
      miniKey("\u00d7", `Delete pattern ${name}`, (key) => twice(key, () => send("/api/cell/pattern/delete", {name, revision: revision()}, key))),
    );
    return li;
  }));
  const toolTargets = Object.entries(data.targets).filter(([, t]) => !t.joints).map(([name]) => [name, name]);
  fillSelect($("patFirst"), toolTargets, "No tool targets yet");
  fillSelect($("patRowEnd"), [["", "none (one column)"], ...toolTargets]);
  fillSelect($("patColumnEnd"), [["", "none (one row)"], ...toolTargets]);

  // Interlocks: the input each reads, lit green while it holds (from the last read of the I/O).
  $("interlocks").replaceChildren(...Object.entries(data.interlocks).map(([name, il]) => {
    const state = io && io.interlocks && io.interlocks[name];
    const guards = Object.entries(data.zones).filter(([, z]) => (z.interlocks || []).includes(name)).map(([z]) => z);
    const cue = !state ? "" : state.holds ? `holds${guards.length ? `: the arm may enter ${guards.join(", ")}` : ""}`
      : `doesn't hold${guards.length ? `: the arm keeps out of ${guards.join(", ")}` : ""}`;
    const {li, acts} = row(name, `holds while ${il.input} reads ${il.value ? "on" : "off"}`, cue);
    const lamp = el("span", "led");
    if (state && state.holds) lamp.dataset.on = "live";
    lamp.setAttribute("aria-hidden", "true");
    li.querySelector(".what").prepend(lamp);
    acts.append(miniKey("\u00d7", `Delete interlock ${name}`, (key) => twice(key, () => send("/api/cell/interlock/delete", {name, revision: revision()}, key))));
    return li;
  }));
  fillSelect($("zoneGuard"), [["", "none: always kept out"], ...Object.keys(data.interlocks).map((name) => [name, name])]);

  $("zones").replaceChildren(...Object.entries(data.zones).map(([name, z]) => {
    const guards = z.interlocks || [];
    let cue = "";
    if (guards.length) {
      const states = guards.map((g) => io && io.interlocks && io.interlocks[g]);
      const open = states.every((s) => s && s.holds);
      cue = `open while ${guards.join(" and ")} ${guards.length === 1 ? "holds" : "hold"}${states.every(Boolean) ? (open ? ": open" : ": closed") : ""}`;
    }
    const {li, acts} = row(name, `in ${z.frame} · centre ${triple(z.pose.xyz, 0)} mm · ${z.size.map((v) => fix(v, 0)).join(" × ")} mm`, cue);
    acts.append(miniKey("\u00d7", `Delete zone ${name}`, (key) => twice(key, () => send("/api/cell/zone/delete", {name, revision: revision()}, key))));
    return li;
  }));

  fillSelect($("homeTarget"), Object.entries(data.targets).map(([name, t]) => [name, t.joints ? `${name} (joints)` : name]),
    "No targets yet");
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

// A pattern's next slot, staged as the panel's plan, as a target's is.
async function planSlot(name, key) {
  const speeds = {joint: P.settings().jointSpeed, tcp: P.settings().toolSpeed};
  const reply = await send("/api/plan", {pattern: name, kind: "joint", defaults: speeds}, key);
  if (!reply) return;
  if (!P.stage(reply.plan.goal, `${reply.message.replace(/ at .*$/, "")} planned. Press Move to go there, or Esc to clear the plan.`)) {
    P.refuse(key);
  }
}

function editGripper(name) {
  const g = cell.cell.grippers[name];
  $("gripName").value = name;
  const [line, bit] = g.close[0].output.split(".");
  gripLine.set(line);
  $("gripBit").value = bit;
  gripValue.set(String(g.close[0].value));
  $("gripSeconds").value = fix(g.seconds, 1);
  $("gripTimeout").value = fix(g.timeout, 1);
  for (const [id, key] of [["gripClosed", "closed"], ["gripOpened", "opened"], ["gripPart", "part"]]) {
    $(id).value = g[key] ? g[key].input : "";
  }
  $("gripName").focus();
}

function editPattern(name) {
  const p = cell.cell.patterns[name];
  $("patName").value = name;
  $("patRows").value = p.rows;
  $("patColumns").value = p.columns;
  $("patLayers").value = p.layers;
  $("patPitch").value = fix(p.layerPitch, 1);
  $("patFirst").value = p.first;
  $("patRowEnd").value = p.rowEnd || "";
  $("patColumnEnd").value = p.columnEnd || "";
  $("patName").focus();
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
  size: [num("zoneW"), num("zoneD"), num("zoneH")], interlocks: $("zoneGuard").value ? [$("zoneGuard").value] : [],
  revision: revision(),
}, e.currentTarget));

// A gripper from its form: one output closes it (the other value opens it); inputs are read as on when they confirm.
// A gripper given more outputs through the API keeps them, unless the form's first output changes.
const gripLine = choice("gripLine");
const gripValue = choice("gripValue");
$("gripSave").addEventListener("click", (e) => {
  const name = $("gripName").value.trim();
  const output = `${gripLine.value}.${Math.round(num("gripBit"))}`;
  const value = Number(gripValue.value);
  const old = cell && cell.cell.grippers[name];
  const same = old && old.close[0].output === output && old.close[0].value === value;
  const input = (id) => ($(id).value ? {input: $(id).value, value: 1} : undefined);
  send("/api/cell/gripper", {
    name, close: same ? old.close : [{output, value}], open: same ? old.open : [{output, value: 1 - value}],
    seconds: num("gripSeconds"), timeout: num("gripTimeout"), closed: input("gripClosed"), opened: input("gripOpened"),
    part: input("gripPart"), revision: revision(),
  }, e.currentTarget);
});

$("patSave").addEventListener("click", (e) => send("/api/cell/pattern", {
  name: $("patName").value.trim(), rows: Math.round(num("patRows")), columns: Math.round(num("patColumns")),
  layers: Math.round(num("patLayers")), layerPitch: num("patPitch"), first: $("patFirst").value,
  rowEnd: $("patRowEnd").value || undefined, columnEnd: $("patColumnEnd").value || undefined, revision: revision(),
}, e.currentTarget));

const lockLine = choice("lockLine");
const lockValue = choice("lockValue");
$("lockSave").addEventListener("click", (e) => send("/api/cell/interlock", {
  name: $("lockName").value.trim(), input: `${lockLine.value}.${Math.round(num("lockBit"))}`, value: Number(lockValue.value),
  revision: revision(),
}, e.currentTarget));

// ---- Program: safe home, and the calls ----
function whyNotHome(s) {
  const r = runOf(s);
  if (!$("homeTarget").value) return "Record a target to go home to first (Teach).";
  if (RUN_BUSY.includes(r.state)) return `${r.manual ? "A move" : r.program} is under way: wait, or Stop run.`;
  if (r.state === "faulted") return "The last run faulted: press Reset first.";
  return ready(s) ? null : whyNotReady(s);
}

$("homeKey").addEventListener("click", (e) => {
  if (refused(e.currentTarget)) return;
  send("/api/home", {target: $("homeTarget").value, retract: num("homeBack"),
    defaults: {joint: P.settings().jointSpeed, tcp: P.settings().toolSpeed}}, e.currentTarget);
});

let callsShown = "";
let callsHeard = 0;  // the newest call said on the message line
function renderCalls(s) {
  const calls = (s && s.calls) || [];
  const key = calls.map((c) => c.id).join(",");
  if (key === callsShown) return;
  callsShown = key;
  $("callsGroup").hidden = !calls.length;
  setText($("callsCue"), calls.length === 1 ? "A run called you." : `Runs called you ${calls.length} times.`);
  $("calls").replaceChildren(...calls.slice().reverse().map((c) => {
    const li = el("li", "row");
    const time = el("time", "", c.at.slice(11, 19));
    time.dateTime = c.at;
    const what = el("span", `what${c.why === "step" ? "" : " bad"}`, c.message);
    if (c.program && c.path) what.append(el("span", "cue", `${c.program}, step ${c.path}`));
    const acts = el("span", "acts");
    acts.append(miniKey("Seen", `Seen: ${c.message}`, (k) => send("/api/calls/ack", {id: c.id}, k)));
    li.append(time, what, acts);
    return li;
  }));
  const newest = calls.length ? calls[calls.length - 1] : null;
  if (newest && newest.id > callsHeard) {
    callsHeard = newest.id;
    P.say(`The run called you: ${newest.message}`, newest.why === "step" ? "wait" : "bad");
  }
}
$("callsSeen").addEventListener("click", (e) => send("/api/calls/ack", {}, e.currentTarget));

// ---- the I/O, for the interlocks' lamps, the grippers' state and the guarded zones in the 3D view ----
let ioAsked = 0;
let ioKey = "";
async function pollIo(now) {
  if (!cell || document.hidden) return;
  const wanted = Object.keys(cell.cell.interlocks).length || Object.keys(cell.cell.grippers).length;
  if (!wanted || (!now && Date.now() - ioAsked < 1000)) return;
  ioAsked = Date.now();
  try {
    const reply = await getJson("/api/io");
    if (!reply.ok) return;
    io = reply;
  } catch (e) {
    return;
  }
  const key = JSON.stringify([io.interlocks, io.grippers]);
  if (key !== ioKey) {
    ioKey = key;
    drawnIn = null;  // guarded zones open or close in the 3D view and its check
    renderCell();
  }
}

// The guarded zones open now (their interlocks all hold, as last read), which the 3D view's check lets the arm enter,
// as the server does.
function openZones() {
  const zones = new Set();
  if (!cell || !io || !io.interlocks) return zones;
  for (const [name, z] of Object.entries(cell.cell.zones)) {
    const guards = z.interlocks || [];
    if (guards.length && guards.every((g) => io.interlocks[g] && io.interlocks[g].holds)) zones.add(name);
  }
  return zones;
}

function viewCell() {
  const open = openZones();
  const collision = cell.collision && {
    ...cell.collision,
    pairs: cell.collision.pairs.filter(([, o, fixed]) => !(fixed && open.has(cell.collision.obstacles[o].zone))),
  };
  return {collision, ...cell.scene, zones: cell.scene.zones.map((z) => ({...z, open: open.has(z.name)}))};
}

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
    view.setCell(viewCell());
    drawnIn = view;
  }
  if (view && view.setFault) view.setFault(r.state === "faulted" ? r.hit : null);
  if (s && s.run) sayRunChange(r);
  renderRun(s);
  renderTeach(s);
  renderCalls(s);
  markCurrent();
  refreshEvents(s);
  pollIo();
  // Keys whose state follows the run, the arm's readiness, the brain and the patterns' counters: redraw the lists when
  // any changes.
  const brain = (s && s.brain) || {};
  const keys = `${r.state}|${r.program}|${ready(s)}|${Boolean(s && s.live)}|${brain.ok}|${Object.keys(brain.skills || {})}|`
    + `${JSON.stringify((s && s.counters) || {})}|${JSON.stringify((s && s.held) || {})}`;
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
