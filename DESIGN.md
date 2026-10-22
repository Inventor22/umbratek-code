---
name: UTRA Jog Panel
description: A six-channel mixing desk for jogging an Umbratek UTRA 6-1000 arm one exact step at a time.
colors:
  amber: "#f2aa3c"
  amber-ink: "#1d1304"
  green: "#5ec683"
  red: "#d63a2f"
  alarm: "#ff8073"
  tape-1: "#e3ba4f"
  tape-2: "#7fb3d9"
  tape-3: "#92cc8a"
  tape-4: "#e19769"
  tape-5: "#baa2dd"
  tape-6: "#dad5c9"
  tape-ink: "#15161a"
  axis-x: "#ff3352"
  axis-y: "#8bdc00"
  axis-z: "#2890ff"
  desk: "#18191b"
  well: "#0e0f10"
  channel: "#1b1d20"
  panel: "#222427"
  seam: "#33363b"
  cap: "#34383d"
  cap-top: "#3f444a"
  cap-edge: "#50565d"
  ink: "#ece8df"
  ink-2: "#aaa69d"
  legend-lit: "#ffffff"
typography:
  display:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "36px"
    fontWeight: 800
    lineHeight: 1
    letterSpacing: "0.08em"
    fontFeature: '"tnum"'
  headline:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "26px"
    fontWeight: 700
    lineHeight: 1.1
    letterSpacing: "0.04em"
    fontFeature: '"tnum"'
  key-glyph:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "24px"
    fontWeight: 500
    lineHeight: 1.4
    fontFeature: '"tnum"'
  readout:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "20px"
    fontWeight: 650
    lineHeight: 1.4
    letterSpacing: "-0.01em"
    fontFeature: '"tnum"'
  title:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "18px"
    fontWeight: 650
    lineHeight: 1.4
    letterSpacing: "-0.01em"
    fontFeature: '"tnum"'
  target:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "15px"
    fontWeight: 650
    lineHeight: 1.4
    fontFeature: '"tnum"'
  bank-title:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "14px"
    fontWeight: 700
    lineHeight: 1.4
    letterSpacing: "0.1em"
    fontFeature: '"tnum"'
  body:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "14px"
    fontWeight: 400
    lineHeight: 1.4
    fontFeature: '"tnum"'
  body-sm:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "13px"
    fontWeight: 400
    lineHeight: 1.4
    fontFeature: '"tnum"'
  key-legend:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "12px"
    fontWeight: 600
    lineHeight: 1.4
    fontFeature: '"tnum"'
  channel-label:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "12px"
    fontWeight: 700
    lineHeight: 1.4
    letterSpacing: "0.06em"
    fontFeature: '"tnum"'
  label:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "11px"
    fontWeight: 600
    lineHeight: 1.4
    letterSpacing: "0.08em"
    fontFeature: '"tnum"'
  fader-value:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "13px"
    fontWeight: 650
    lineHeight: 1.15
    fontFeature: '"tnum"'
  scale:
    fontFamily: "Segoe UI Variable Text, Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif"
    fontSize: "10px"
    fontWeight: 400
    lineHeight: 1.2
    fontFeature: '"tnum"'
rounded:
  light: "1px"
  tape: "2px"
  meter: "3px"
  key: "4px"
  panel: "6px"
spacing:
  xs: "4px"
  sm: "6px"
  md: "8px"
  lg: "12px"
  gutter: "14px"
  margin: "16px"
components:
  key:
    backgroundColor: "{colors.cap}"
    textColor: "{colors.ink}"
    rounded: "{rounded.key}"
  key-hover:
    backgroundColor: "{colors.cap-top}"
  key-held:
    backgroundColor: "{colors.amber}"
    textColor: "{colors.amber-ink}"
  key-refused:
    textColor: "{colors.alarm}"
  key-jog:
    typography: "{typography.key-glyph}"
    height: "46px"
  key-jog-tool:
    width: "50px"
    height: "44px"
  key-small:
    typography: "{typography.key-legend}"
    height: "30px"
    padding: "0 8px"
  detent-slot:
    backgroundColor: "{colors.well}"
    rounded: "{rounded.meter}"
    height: "6px"
  detent-cap:
    backgroundColor: "#f6f3ec"
    rounded: "{rounded.tape}"
    width: "11px"
    height: "18px"
  detent-value:
    textColor: "{colors.ink}"
    typography: "{typography.fader-value}"
  detent-scale:
    textColor: "#8f8b83"
    typography: "{typography.scale}"
  detent-redline:
    backgroundColor: "{colors.alarm}"
    height: "2px"
  key-enable:
    width: "150px"
    padding: "10px 14px"
  key-stop:
    backgroundColor: "{colors.red}"
    textColor: "{colors.legend-lit}"
    typography: "{typography.display}"
    rounded: "{rounded.panel}"
    width: "232px"
    padding: "8px 26px"
  lamp-live:
    backgroundColor: "{colors.green}"
    size: "9px"
  lamp-slow:
    backgroundColor: "#8f8b83"
    size: "9px"
  lamp-dead:
    backgroundColor: "{colors.alarm}"
    size: "9px"
  legend:
    textColor: "{colors.ink-2}"
    typography: "{typography.label}"
  readout-window:
    backgroundColor: "{colors.well}"
    textColor: "{colors.ink}"
    typography: "{typography.headline}"
    rounded: "{rounded.key}"
    padding: "8px 14px"
  readout-window-moving:
    backgroundColor: "{colors.amber}"
    textColor: "{colors.amber-ink}"
  readout-window-alarm:
    backgroundColor: "#c62f25"
    textColor: "{colors.legend-lit}"
  readout-window-ready:
    backgroundColor: "#1a4a2d"
    textColor: "#a6efbf"
  readout-window-stopped:
    backgroundColor: "#2b100d"
    textColor: "{colors.alarm}"
  section:
    backgroundColor: "{colors.panel}"
    rounded: "{rounded.panel}"
    padding: "{spacing.lg}"
  screen:
    backgroundColor: "{colors.well}"
    rounded: "{rounded.key}"
  screen-help:
    textColor: "#9a968d"
  screen-tag:
    backgroundColor: "{colors.well}"
    textColor: "{colors.ink}"
    typography: "{typography.body-sm}"
    rounded: "{rounded.meter}"
    padding: "6px 9px"
  screen-tag-target:
    textColor: "{colors.amber}"
  screen-tag-blocked:
    textColor: "{colors.alarm}"
  plan-bar:
    backgroundColor: "{colors.desk}"
    rounded: "{rounded.key}"
    padding: "8px 10px"
  plan-chip:
    backgroundColor: "{colors.well}"
    textColor: "{colors.ink}"
    typography: "{typography.body-sm}"
    rounded: "{rounded.meter}"
    padding: "2px 2px 2px 3px"
  plan-note:
    textColor: "{colors.ink-2}"
  plan-note-blocked:
    textColor: "{colors.alarm}"
  key-move:
    textColor: "{colors.amber}"
    typography: "{typography.target}"
    height: "40px"
    width: "104px"
    padding: "0 18px"
  channel-strip:
    backgroundColor: "{colors.channel}"
    rounded: "{rounded.key}"
    padding: "{spacing.md}"
  channel-readout:
    textColor: "{colors.ink}"
    typography: "{typography.readout}"
  channel-readout-moving:
    textColor: "{colors.amber}"
  channel-target:
    textColor: "{colors.amber}"
    typography: "{typography.target}"
  channel-target-blocked:
    textColor: "{colors.alarm}"
  meter:
    backgroundColor: "{colors.well}"
    rounded: "{rounded.meter}"
  lead-ladder:
    backgroundColor: "{colors.well}"
    rounded: "{rounded.meter}"
    width: "30px"
    padding: "4px 5px"
  lead-light:
    backgroundColor: "#25282c"
    rounded: "{rounded.light}"
  lead-light-on:
    backgroundColor: "{colors.amber}"
  scribble:
    textColor: "{colors.tape-ink}"
    typography: "{typography.channel-label}"
    rounded: "{rounded.tape}"
    padding: "4px 7px"
  scribble-j1:
    backgroundColor: "{colors.tape-1}"
    textColor: "{colors.tape-ink}"
  scribble-j2:
    backgroundColor: "{colors.tape-2}"
    textColor: "{colors.tape-ink}"
  scribble-j3:
    backgroundColor: "{colors.tape-3}"
    textColor: "{colors.tape-ink}"
  scribble-j4:
    backgroundColor: "{colors.tape-4}"
    textColor: "{colors.tape-ink}"
  scribble-j5:
    backgroundColor: "{colors.tape-5}"
    textColor: "{colors.tape-ink}"
  scribble-j6:
    backgroundColor: "{colors.tape-6}"
    textColor: "{colors.tape-ink}"
  message-line:
    backgroundColor: "{colors.well}"
    textColor: "{colors.ink}"
    typography: "{typography.body-sm}"
    padding: "9px 16px"
---

# Design System: UTRA Jog Panel

## Overview

**Creative North Star: "The Six-Channel Desk"**

The jog panel is a mixing desk for a robot arm. Each of the six joints is a channel strip: a length of coloured scribble tape with the joint's name, a plus key, a vertical meter with a ladder of lead lights beside it, a minus key and a readout. The meter shows where the joint is (a solid fader cap that can be grabbed and dragged) and where it was sent (a hollow amber frame); the lead lights count the steps the joint still has to go and go out as it catches up. A 3D monitor, stacked above the faders in the owner's app side panel and beside them on wide screens, shows the arm itself, standing on its cart above the floor, with a ring at each joint in that joint's tape colour: dragging rings (or the meters' fader caps) stages a pose, shown by an amber shadow arm, and nothing moves until Move sends it. The tool gets a master section of six axis rows. A meter bridge across the top carries the link lamp, the machine state, the hold-to-enable key and an oversized red STOP; a message line is pinned along the bottom.

The desk is matte charcoal and dense, with every control on screen at once. Colour is light, not paint: the keys are identical charcoal caps, and colour appears only to report state. Amber is reserved for held keys and motion, green for live and powered, red for stopped, faulted or unreachable, and for moves that are refused or blocked. The owner watches the arm more than the screen, so the state reads at a glance: one big state word in a window that lights up, one big red key. Light is flat and never glows. Motion is short and mechanical, mostly on one quick ease-out curve, and switches off when reduced motion is requested.

The panel rejects Umbratek Studio's grey round buttons floating over a 3D viewport. Its 3D view is a monitor sunk into the desk, the controls inside it are the joints' own rings, and the only panels over the picture are a readout while a drag is under way and the plan bar docked along its bottom while a pose is staged. Every other control is a square key with a printed legend, mounted on the desk.

**Key Characteristics:**
- Matte charcoal desk; depth comes from recessed wells, flat panels and raised key caps.
- Square backlit keys; colour only reports state: amber held or moving, green live, red stop or fault.
- Step and speed set on small detented faders, the same cap and slot as the joint faders, snapping to the values the arm accepts, with a speedometer-style redline wherever the flange could pass 250 mm/s.
- One scribble-tape colour per joint, carried onto that joint's ring in the 3D monitor.
- A sunk 3D monitor where dragging joints' rings or their fader caps, or a gizmo at the flange whose arrows and rings move the tool while the joints follow, stages a pose shown by an amber shadow arm, which Move sends as one move; the arm stands on its cart, and a pose that would hit anything turns red.
- Amber lead lights that count the steps a joint still has to travel.
- One system sans family, tabular figures, uppercase printed legends.
- A sticky meter bridge with STOP, the biggest key, at its top-right, and a message line pinned to the bottom.
- Keys travel: a hard 2px bottom edge that collapses when pressed.

## Colors

A charcoal desk lit by three signal colours, with six strips of coloured tape for joint identity, and red, green and blue naming the tool gizmo's axes.

### Primary
- **Backlight Amber** (#f2aa3c): held keys and motion. A key while it is held down; a joint or axis travelling to its commanded value (the hollow goal frame, the lit lead lights, the "→ target" line, the readout itself, the ring round the tape's activity dot); a move's destination in the 3D monitor (the shadow arm, the sweep arc on a dragged ring, the dragged gizmo arrow, a turned gizmo ring's wedge and arc, the tool path, the floating readout's border and target); the lamp along the plan bar's Move key and along RUN; the state window and the run window, lit solid, while the arm is MOVING, RUNNING, STEPPING or in FREEDRIVE; the lamp of the step a run is moving through, and a hollow amber ring on the step it waits at (paused, halted or stepped), with that step's number; the freedrive key's charge bar, and its lamp and legend while freedrive is on; messages about a hold or move still in progress; and the matching samples in the legends. Never the focus ring.
- **Amber Key Ink** (#1d1304): text on amber: a held key's legend and the words in the MOVING window.

### Secondary
- **Motors-On Green** (#5ec683): live and powered. The link lamp while fresh status is arriving, Enable's charge bar, and Enable's lamp once the motors are on.
- **STOP Red** (#d63a2f): the face of the STOP key, and nothing else.
- **Alarm Red** (#ff8073): stopped, faulted, unreachable, refused or blocked, as light rather than fill. The dead link lamp; the rim of the alarm state window; the STOPPED and PAUSED state words, and the run window's STOPPED, PAUSED, HALTED and FAULTED; the lamp and number of the step a run faulted at; a failed run's lines in the run log; a step's or target's "no such target" and "out of reach" cue; a delete key's "Sure?" while it waits for its second press; the 2px ring and legend on a refused key; the tick above a meter marker past ±180°; failure messages; and every mark of a blocked move: the shadow arm, sweep arc, tool path and floating readout, the contact mark and the overlay on the part it would hit (also for a program step that faulted on a collision), the plan bar's note, and the fader's goal frame and target line. Also the speed faders' redline, at the owner's request: the rule and ticks past it, the redline's figure and the top figure, the value while the cap sits past it, the message that says why, and the plan bar's note when a staged move would pass it.

### Tertiary
- **Scribble tapes**, one per joint in order: Masking Yellow (#e3ba4f) for J1 Base, Sky Tape (#7fb3d9) for J2 Shoulder, Sage Tape (#92cc8a) for J3 Elbow, Apricot Tape (#e19769) for J4 Wrist 1, Lilac Tape (#baa2dd) for J5 Wrist 2, Paper Tape (#dad5c9) for J6 Flange. Each fills its joint's scribble strip, colours its ring in the 3D monitor and, mixed at 55% with transparency, its meter's lit column.
- **Marker Black** (#15161a): legends written on the tape and the activity dot at the tape's right end.
- **Axis Red** (#ff3352), **Axis Green** (#8bdc00) and **Axis Blue** (#2890ff): the tool gizmo's X, Y and Z, coloured as 3D tools colour them, at the owner's request: each axis's arrow, ring, printed letter, ring name and hover band, and the Tool legend's ring sample. They name an axis and never report state. The red is purer and deeper than Alarm Red's coral, the green yellower than Motors-On Green's mint, and both appear only on the gizmo.

### Neutral
- **Desk Charcoal** (#18191b): the page ground, and (at 95%) the plan bar docked over the 3D screen.
- **Well Black** (#0e0f10): recessed displays: the state window at rest, the 3D screen (its scene background and fog too), the meters, the lead ladders and the message line; at 94% behind the floating readout.
- **Channel Charcoal** (#1b1d20): channel strips, tool axis rows, and the rows in the program, cell and log wells.
- **Panel Grey** (#222427): the meter bridge and the desk's sections.
- **Seam** (#33363b): every 1px border and divider, and the meter's zero line.
- **Key Cap** (#34383d), **Cap Highlight** (#3f444a) and **Cap Rim** (#50565d): a key cap is a vertical gradient from highlight to cap inside a 1px rim.
- **Legend Ink** (#ece8df): text, readouts and key legends; the keyboard focus ring.
- **Printed Grey** (#aaa69d): printed legends and secondary text on panels, channels and wells (6.4 to 7.9:1), including the floating readout's small line and the monitor's notes; and the zero line across each lead ladder.
- **Lit White** (#ffffff): a lit legend: on STOP and on the alarm state window.

### Named Rules
**The Backlight Rule.** Keys are charcoal at rest. Colour on the desk is light that reports state: amber for held keys and motion, green for live and motors on, red for stop, fault, no signal and refusal, and for speeds past the redline. Focus is drawn in Legend Ink, and nothing is coloured for decoration: besides the state colours, only the tapes, which name joints, and the gizmo's axis colours, which name axes, carry hue.

**The Brightness Ladder Rule.** The state window's tones differ in brightness, not just hue. From brightest to darkest: MOVING (and RUNNING or FREEDRIVE) lit solid amber, an alarm in solid red, READY lit green, STOPPED or PAUSED in a dark red tint, MOTORS OFF or CONNECTING in a plain dark well. Stale status drops any lit tone except alarm to that dark well and keeps the word. The run window climbs the same ladder without its solid red rung: RUNNING, STEPPING, PAUSING and STOPPING lit amber; DONE and ARRIVED lit green; PAUSED, HALTED, STOPPED and FAULTED in the dark red tint; NO RUN, DRY RUN and PLANNED in the plain well.

**The One Red Key Rule.** STOP is the only key with a red face, and the alarm state window is the only other solid red (a faulted run takes the dark red tint, never solid red). Elsewhere red is a lamp, a word, a rim, a 2px ring, a tick or a printed rule, and it marks a refused or blocked move, a speed past the redline, a failed step, or a delete waiting for its second press: the red shadow arm, arc, path and readout border, the red contact mark and the red overlay on the part it would hit, the red goal frame on a fader, the red zone on a speed fader, a failed step's lamp, and the "Sure?" legend on an armed delete key.

**The Tape Rule.** Each joint keeps its tape colour, in joint order, on its scribble strip, its meter's lit column and its ring in the 3D monitor, and nowhere else (the legend's ring sample borrows tape 2). Tape names a joint; its hue never reports state.

**The Axis Colour Rule.** The tool gizmo colours X, Y and Z red, green and blue, as 3D tools do, and nothing else wears those three colours (the Tool legend's ring sample borrows them). They name an axis and never report state: a dragged arrow lights amber and a blocked turn goes Alarm Red, as everywhere. The printed letters stay, so colour is never the only cue, as for someone who can't tell red from green.

## Typography

**Display Font:** Segoe UI Variable Text (with Segoe UI, system-ui, -apple-system, Helvetica Neue, sans-serif)
**Body Font:** the same family
**Label/Mono Font:** the same family, uppercase and tracked; there is no mono face

**Character:** One plain system sans does every job, the way a desk's legends are all printed in one face. Weight (400 to 800) and size carry the hierarchy, and every figure is tabular.

### Hierarchy
- **Display** (800, 36px, line-height 1, 0.08em): the STOP legend only; 26px under 560px.
- **Headline** (700, 26px, 1.1, 0.04em): the uppercase state word in the state window; 20px under 560px.
- **Key glyph** (500, 24px): the + and − on joint keys; tool keys use 22px at weight 400.
- **Readout** (650, 20px, 1.4, -0.01em): joint angles with two decimals and °. Tool values use the same voice at 19px with one decimal.
- **Title** (650, 18px, 1.4, -0.01em): the panel name in the bridge.
- **Target** (650, 15px, 1.4): the amber "→ target" line under each joint readout, and Enable's label. Tool rows use a smaller 11px target line.
- **Bank title** (700, 14px, 0.1em, uppercase): section names, ARM, JOINTS and TOOL.
- **Body** (400, 14px, 1.4): the base size.
- **Body small** (400, 13px, 1.4): the link line, the state window's sentence, the message line, and the floating readout (at line-height 1.35).
- **Key legend** (600, 12px): the Reset view and Clear keys.
- **Fader value** (650, 13px, line-height 15px): the step and speed values above their faders, with units (0.5°, 20 mm/s).
- **Channel label** (700, 12px, 0.06em, uppercase): tape legends such as J1 BASE; 11px at 0.02em when the tape wraps to two lines. Tool axis names use 12px bold at 0.08em.
- **Label** (600, 11px, 0.08em, uppercase): printed legends such as STEP, SPEED, POSITION and ORIENTATION.
- **Scale** (400, 10px, line-height 12px): the meters' scale labels and the faders' range ends, in #8f8b83.

### Named Rules
**The Tabular Rule.** Every figure is tabular, set once for the whole page, with fixed decimals: two for joint angles, one for tool values and drag targets. Digits never shuffle while the arm moves.

**The Printed Legend Rule.** Names of groups, settings and channels are printed: uppercase, semibold or bold, tracked 0.06 to 0.1em. Machine states are uppercase words, even inside a message ("STOP NOT CONFIRMED: press the hardware E-stop now."). Instructions, values, units and messages otherwise stay in sentence case.

## Layout

The page is three bands filling the viewport: the meter bridge, the desk and the message line. The bridge is sticky at the top, and the message line is pinned to the bottom (`position: sticky`).

**First viewport:** in the owner's app side panel (about 1018px wide) the desk is stacked, with the monitor above the faders. Side by side applies only at 1241px and wider; the breakpoint is 1240px. Stacked, the screen is clamp(300px, 36vh, 500px) tall and the fader meters are clamp(170px, 22vh, 280px), so the fader readouts land in the first screenful.

- **Meter bridge:** one row of four zones: name and link lamp (at least 170px, 1fr), the state window (at least 280px, 2fr), Enable, then STOP at the right end. 14px gutters, 12px by 16px padding. It casts a shadow onto the desk.
- **Desk at 1241px and wider:** the monitor at the left, beside a column of the joint, tool, program, teach and cell sections. The monitor column is at least 400px and takes the spare width; the right column is up to 780px. 14px gutters inside a 14px by 16px margin. The monitor is pinned (sticky, 110px from the top) and sized to fill the space between the bridge and the message line (at least 460px tall) while the right column scrolls.
- **Desk at 1240px and narrower:** everything stacks: monitor, joints, tool, program, teach, cell, with the screen clamp(300px, 36vh, 500px) tall.
- **Sections:** each opens with a bank row: its title, then (for the monitor) the legend and the Reset view key, (for joints) the step and speed faders and the meter legend (actual, commanded, planned, steps ahead), or (for the tool) its step, turn and speed faders. Faders share their bank row in equal widths (at least 150px, at most 320px, about 200px before they wrap), 24px apart with a seam between them, with the title baseline-aligned to their legends; the meter legend shares the row when it fits and takes its own row, flush right, when it doesn't.
- **Joint strips:** six equal strips (at least 96px) in joint order, 8px apart, packed to the top. Top to bottom: tape, + key (46px), the meters (clamp(170px, 22vh, 280px)), − key (46px), readout, target line. The meters are the ±180° meter, taking the remaining width, and its 30px lead ladder, 4px apart.
- **Tool rows:** two columns, POSITION (X, Y, Z) beside ORIENTATION (Roll, Pitch, Yaw), each a printed legend over three rows 8px apart. Each row: name (58px), − key (50px), reading, + key (50px).
- **Message line:** the latest message on the left and a standing hint on the right, wrapping when narrow.

The spacing rhythm is 8px inside channels and rows, 12px inside sections, 14px between zones and sections, 16px at the page edge, and 4 to 6px between a lamp or legend and its neighbour.

Responsive behaviour:
- **1240px and below:** the desk stacks as above.
- **860px and below:** the bridge splits into two rows (name, Enable and STOP, then the state window at full width), and strips go three to a row.
- **560px and below:** the state window and STOP share the bridge's first row, name and Enable the second; the bridge's gaps and padding drop to 8px; STOP narrows to at least 112px with a 26px legend; the state word drops to 20px; the screen is half the window's height (360px to 500px), so the tool gizmo fits above a staged plan and Move stays above the message line; the lead ladder narrows to 22px with a 3px gap; tool rows go to one column; bank titles take their own row and each step or speed fader the full width.
- **Fader bank narrower than 760px** (a container query on the strips): the tape wraps to two lines, J-number and activity dot, then the full name.
- **3D screen narrower than 520px** (a container query on the screen): the plan bar drops its PLAN legend and each chip's from-angle, its joints take the full width, and its note sits beside Clear and Move.
- **3D screen at least 640px wide and more than 1.6 times as wide as tall** (set by the 3D view on the screen): the plan bar and help line dock in a 248px column at the right, and the arm and cart take the full height.

### Named Rules
**The Bridge Stays Rule.** The meter bridge and the message line never scroll away, and STOP is always at the bridge's top-right.

**The First Screenful Rule.** Stacked, the monitor stays short enough (clamp(300px, 36vh, 500px)) and the fader meters compact enough (clamp(170px, 22vh, 280px)) that the fader readouts still land in the first screenful; side by side, at 1241px and wider, the monitor stays pinned in view while the faders scroll.

**The Up Is Plus Rule.** On a strip, + sits above the meter and − below, matching the meter's scale (+180 at the top) and the lead lights (a + lead lights upward). Tool rows read − on the left and + on the right.

## Elevation & Depth

Depth is physical and mostly tonal. Displays sink into wells darker than the desk; sections and strips lie flat, edged with 1px seams; keys stand up as caps lit from above with a hard 2px bottom edge. Depth shadows are black and appear only where a real object would cast or receive one: under keys and fader caps, inside wells, under the sticky bridge, under the readout and the plan bar over the 3D screen, and as the arm's and cart's contact shadows on the floor. Coloured light never glows.

### Shadow Vocabulary
- **Key at rest** (`box-shadow: 0 2px 0 #0b0c0d, 0 3px 6px rgb(0 0 0 / 0.35)`): the hard edge and a soft drop under every key.
- **Key pressed** (`box-shadow: 0 0 0 #0b0c0d, inset 0 2px 4px rgb(0 0 0 / 0.3)` with `transform: translateY(2px)`): the edge collapses as the cap goes down.
- **STOP at rest** (`box-shadow: 0 3px 0 #5f130d, 0 6px 14px rgb(0 0 0 / 0.45)`): a deeper edge for the biggest key.
- **STOP pressed** (`box-shadow: 0 0 0 #5f130d, inset 0 3px 6px rgb(0 0 0 / 0.35)` with `transform: translateY(3px)`).
- **Screen well** (`box-shadow: inset 0 2px 6px rgb(0 0 0 / 0.5)`): the state window (in every tone) and the 3D screen.
- **Meter well** (`box-shadow: inset 0 2px 5px rgb(0 0 0 / 0.45)`): meters and lead ladders.
- **Slot well** (`box-shadow: inset 0 1px 3px rgb(0 0 0 / 0.6)`): the step and speed faders' slots.
- **Fader cap** (`box-shadow: 0 1px 3px rgb(0 0 0 / 0.7)`): every fader cap, on the joint meters and the step and speed faders.
- **Bridge** (`box-shadow: 0 4px 12px rgb(0 0 0 / 0.35)`): the sticky bridge over the scrolling desk.
- **Floating readout** (`box-shadow: 0 4px 10px rgb(0 0 0 / 0.45)`): the drag readout over the 3D screen.
- **Plan bar** (`box-shadow: 0 4px 12px rgb(0 0 0 / 0.45)`): the plan bar docked over the 3D screen.
- **Refusal ring** (`box-shadow: inset 0 0 0 2px var(--alarm)`): a refused key, for 450ms.
- **Activity ring** (`box-shadow: 0 0 0 2px var(--amber)`): round the activity dot of a moving channel.

### Named Rules
**The Key Travel Rule.** A pressed key moves: its 2px edge collapses, the cap drops 2px (STOP 3px) and the shadow turns inward, over 120ms (STOP 100ms). Hover only brightens the cap a step; nothing lifts.

**The Sunk Display Rule.** The state window, the 3D screen, the meters, the lead ladders and the message line are wells; keys are raised caps; sections and strips stay flat between them.

**The Flat Light Rule.** Lit things are flat: lamps, lead lights and lit state windows never glow, the joint rings are flat unlit colour, and the shadow arm is self-lit and translucent without a halo. The only coloured shadows are hard 2px rings: the refusal ring and the activity ring.

## Shapes

Small, machined corners: 4px on keys, the state window, the 3D screen, the plan bar, channel strips and tool rows; 6px on sections and STOP; 3px on meters, lead ladders, fader slots, the floating readout and plan chips; 2px on tape, a plan chip's joint tag, meter markers and fader caps; 1px on lead lights and legend swatches. Only lamps and rings are round: the 9px link and Enable lamps, the 8px activity dot on each tape, the joint and gizmo rings in the 3D monitor and their legend samples, and the 14px contact mark on a blocked path; lead lights are flat bars. Sections, strips, wells and the bridge are edged with 1px seams, keys with a 1px rim, a lit state window with a 1px rim in its tone, the floating readout with a 1px amber (or red) border, and STOP with a 2px light red rim (#ef5d50). On a meter the actual angle is a solid fader cap with a centre groove, and the commanded angle is a hollow frame with a 2px amber border.

### Named Rules
**The Square Key Rule.** Every key is a squared cap with 4px corners (STOP 6px). No key is round or pill-shaped; round belongs to lamps and joint rings.

## Components

### Keys
Charcoal caps lit from above, with real travel.
- **Shape:** 4px corners, a 1px Cap Rim and a 2px hard bottom edge.
- **Rest:** a gradient from Cap Highlight to Key Cap with a Legend Ink legend.
- **Hover:** the cap brightens one step.
- **Held:** a Backlight Amber face with an Amber Key Ink legend, the cap down 2px. Every key but STOP lights amber while held; jog keys repeat while held.
- **Focus:** a 2px Legend Ink outline at a 2px offset, never amber.
- **Refused:** a darkened red cap with a 2px Alarm Red ring and legend, for 450ms: a jog key the arm can't take, or Move when the plan can't go.
- **Disabled:** 38% opacity, no edge, a not-allowed cursor. Jog keys are disabled whenever the arm can't be jogged.
- **Jog keys:** the full strip width and 46px tall with a 24px glyph on strips; 50px by 44px with a 22px regular-weight glyph on tool rows. Minus keys use the true minus sign (−).

### Step and speed faders
Short faders in the joint and tool banks, one per setting: joint step (0.1–15°) and speed (1–120 °/s); tool step (0.1–50 mm), turn (0.1–15°) and speed (1–1000 mm/s, with a detent at 250). The detents are the values the server accepts, so a cap can only rest where the arm will take it; joint moves are capped at 80% of each joint servo's own speed limit (see README).
- **Channel:** a printed legend (STEP, TURN, SPEED) at the left and the value in Fader value type at the right, over a slot as wide as the channel, with the range's two ends printed under it in Scale type (#8f8b83): "0.1 … 15", and on the speed faders the redline's figure between them (see Redline). 46px tall. Channels sharing a row stand 24px apart with a 1px seam between them, like a desk's channel dividers; a channel that wraps to the start of a row has none.
- **Slot:** a 6px Well Black slot with a 1px seam, 3px corners and the slot-well shadow, with a 1px by 4px tick (#7a7f87) under every detent. On the speed faders the slow default they start from (10 °/s, 20 mm/s) is a longer 6px tick in the Scale ink (#8f8b83), like a desk's unity mark.
- **Top-speed limits:** when the joint speed is set above a joint's top speed, the joints it can't reach and their top speeds are printed in Scale type between the range ends: "J1–2 max 84, J3 114.5, J4–6 119.1". Below 84 °/s the line is empty.
- **Redline:** like a speedometer's red zone, each speed fader marks the speeds that could carry the flange faster than 250 mm/s, ISO 10218's limit for jogging by hand.
  - **Marks:** a 2px Alarm Red rule runs along the ticks' feet, clear of the focus ring, from the redline to the top of the scale. The ticks past it turn Alarm Red, and so do the top figure and the redline's own figure, printed in Scale type at its start ("250", or "17" on joint speed). The redline's figure hides where it would touch an end figure or the top-speed note. While the cap sits past the line, the value turns Alarm Red too.
  - **Messages:** whenever the cap moves into or within the red, the message line says why in its failure tone ("20°/s is in the red here: J2 would swing the flange at about 310 mm/s, past the 250 mm/s limit for jogging by hand."). When it leaves, the line says so in its plain tone. It also speaks when the line moves past a resting cap, or when what the line measures changes, except during a drag or while a move is under way. Figures round up to 10 mm/s.
  - **A warning, not a stop:** the red speeds still work.
  - **Tool speed:** the redline is fixed at 250 mm/s, a detent of its own.
  - **Joint speed:** the redline follows the arm. Turning one joint alone, within its top speed, it sits at the slowest speed that would swing the flange past 250 mm/s from where the arm is now. That is usually the shoulder, near 25 °/s with the arm folded and 14 °/s stretched out. With a plan staged, the move turns every joint in step, so the plan has a line of its own; the fader shows the lower of the two, since the ± keys still jog one joint at this speed. When the plan's own line is passed, the plan bar's note says so. Until the pose is known it assumes full reach (1000 mm). A crossing made while the arm moves is reported once it settles.
  - **Screen readers** hear "in the red" after the value.
- **Cap:** the joint faders' cap turned upright: 11px by 18px, 2px corners, a gradient from #f6f3ec to #cfcac0 with a dark centre groove and the fader-cap shadow. It lightens to #ffffff and #e4dfd5 on hover and while dragged.
- **Behaviour:** dragging slides the cap from detent to detent and clicking the slot jumps to the nearest; arrow keys step one detent, Home and End go to the ends. The value updates as it moves. Steps are remembered on this PC; the speeds go back to their slow defaults every time the panel opens. Screen readers hear the value with its unit in words ("1 millimetre", "120 degrees per second"), followed on joint speed by the joints it limits ("limited to 84 for J1 and J2, 114.5 for J3, 119.1 for J4, J5 and J6").
- **Focus:** a 2px Legend Ink ring drawn tight around the slot, with no offset, so it clears the value above and the figures below; the cap rides over it. Only the keyboard shows it.
- **High Contrast:** Windows High Contrast drops the gradients and shadows, so there the cap is solid ButtonText, the ticks and seams GrayText, the default mark CanvasText, the redline, its figures and a value past it Highlight, and focus shows as a system-colour outline around the fader.
- The monitor's Reset view key and the plan bar's Clear key are small keys at least 46px wide and 30px tall (Clear 36px) with a 12px semibold Printed Grey legend.

### Hold-to-enable key
- **Style:** at least 150px wide: a lamp and a 15px label over an 11px sub-line.
- **Hold:** holding fills a 5px Motors-On Green bar along the bottom edge over 700ms (linear). Letting go early drains it in 160ms, and the message line says to keep holding.
- **On:** a dark green cap, a green lamp and pale green legends ("Motors on", "Hold again to reset").
- **Disabled:** without a link or while a stop is unconfirmed; in the second case the sub-line reads "Locked: use the E-stop".

### STOP key
- **Style:** a STOP Red face, a 2px rim, 6px corners and at least 232px wide: the 36px STOP legend over an 11px sub-line naming its shortcuts (Space or Esc). The build sets that sub-line at 92% opacity (about 4.2:1); at full Lit White it measures 4.7:1.
- **Behaviour:** fires on press, not release, and from Space anywhere on the page, except in a text field, where Space types a space (STOP and Esc still stop there). Esc stops too, except that while the arm is still it first backs out one level: a drag in progress, then a staged plan. It is never disabled. It also ends a program run and freedrive.
- **Hover / Pressed:** a lighter red on hover; a deep red with the cap down 3px when pressed.

### State window
A recessed window: the uppercase state word over a 13px sentence saying what to do next. Its tone lights the whole window over 160ms, each rung at a different brightness:
- **Moving** (solid Backlight Amber, Amber Key Ink words, #ffd08a rim): MOVING.
- **Alarm** (solid #c62f25, Lit White words, Alarm Red rim): STOP NOT CONFIRMED, E-STOP, FAULT, and NO SIGNAL (only after 5s without controller status, or once the jog panel server has stopped answering).
- **Ready** (#1a4a2d, pale green word #a6efbf and sentence #d3eedc, #4f9c6b rim): READY. With a pose staged, its sentence follows the plan: "J2, J3 planned. Press Move to send them, or Esc to clear.", or when it's blocked, "The plan is blocked by the cart's column. Change it, or press Esc to clear." (the plan bar's note gives the full reason).
- **Stopped** (#2b100d, Alarm Red word, pale pink sentence #f3c3bd, #8c2f26 rim): STOPPED, PAUSED.
- **Idle** (the plain well, Legend Ink word, Printed Grey sentence): MOTORS OFF, and CONNECTING before the first status arrives.
- **Stale status:** when the controller's status is more than 1s old, the window dims to the unlit idle tone but keeps its word (READY stays READY); the sentence says how old the status is and that moves wait for fresh status, and new drags are locked. Staleness never takes the alarm tone; a window already in alarm stays in alarm.
- **NO SIGNAL:** shows only after 5s without controller status. Three failed polls in a row is the separate case of the jog panel server not answering, with its own sentence and the link text "Jog server not answering".

### Channel strip (signature)
- **Strip:** Channel Charcoal, a 1px seam, 4px corners, 8px padding and gaps.
- **Tape:** the joint's tape colour with a Marker Black legend (J1 BASE) and 2px corners; an activity dot at the right end, dim at rest, solid with an amber ring while the joint moves. The name wraps onto its own line when the fader bank is narrower than 760px.
- **Meter:** a well running from +180° at the top to −180° at the bottom, with a seam zero line, tick rails every 30° on both sides and 10px scale labels (180, 90, 0, −90, −180). A column of the tape colour runs from 0 to the actual angle; the solid fader cap marks the actual angle; the hollow amber frame marks the commanded angle and shows only while the joint is travelling; a dashed frame marks an angle staged in the plan and not yet sent (solid amber while it's being dragged, red when the plan is blocked); a red tick sits above a marker past ±180°. Markers and the column glide over 140ms.
- **Fader-cap drag:** the cap can be grabbed within 14px. It lightens on hover and the cursor becomes a grab hand, then a grabbing hand. While dragging, the hollow goal frame follows the pointer (0.5° steps, 5° with Shift), the target line reads "→ x°", the lead lights go dark, and the monitor shows the same floating readout anchored just outside that joint's ring, on the side away from the links that follow it (for J6, away from J5). A cap on a staged joint can be grabbed at its planned angle too. Releasing stages the joint at that angle (dropped back where the arm is, it leaves the plan), and the goal frame and target line keep showing the planned angle, dashed and grey, until the plan is sent or cleared; a blocked plan turns the goal frame red and the target line reads "Blocked".
- **Readout:** Readout type, amber while moving, with the amber Target line ("→ 47.50°") beneath it while moving or dragging. A staged angle shows there in Printed Grey ("→ -60.0°") until it's sent.

### Lead lights (signature)
An LED ladder beside each meter that counts how far the commanded goal is ahead of the joint.
- **Ladder:** a 30px well (22px under 560px), 3px corners, with a 2px Printed Grey zero line across its middle.
- **Lights:** three steps each way (the panel's maximum lead), four lights per step. Lights are 2px apart, step groups 5px apart, 1px corners; unlit #25282c, lit flat Backlight Amber.
- **Reading:** lights come on outward from the zero line, upward for a + lead and downward for a − lead. The count is the gap times four over the selected step, rounded up: at least one while moving, at most twelve. They go out as the joint catches up, switching in 80ms.
- **Legend:** "steps ahead", a flat amber 7px by 12px swatch beside "actual", "commanded" and "planned" (a dashed frame) in the joint bank.

### 3D monitor (signature)
A screen sunk into the desk, showing the arm built from Umbratek's URDF and STL meshes.
- **Section:** a desk section whose bank holds the ARM title, the mode keys (Joints, Tool), a legend (a ring sample, "drag a ring" in Joints mode as a Sky Tape ring and "drag an arrow or ring" in Tool mode as a ring in the three axis colours, a translucent amber swatch "plan", a translucent red swatch "blocked") and the Reset view key.
- **Screen:** a Well Black well with a 1px seam, 4px corners and the screen-well shadow. A grey help line (12px, #9a968d) runs along its bottom and says what a drag will do, in the mode in use, or why the rings or gizmo are locked; the plan bar covers it while a pose is staged, and a centred Printed Grey note covers it while the model loads or if the 3D view fails.
- **Scene:** warm light grey links (#d9d4ca) on a darker base (#8f8b84), standing on the cart from its CAD, whose URDF colours are drawn 40% of the way to #5c6068 so the anodised frame recedes behind the arm and the dark controller and casters still read against the well. Hemisphere, key and rim lights; soft contact shadows on a grid (#24272b, centre lines #3a3e44, 3.2m across) that marks the floor under the casters; fog fades the far floor into the well. The arm appears with the first status and eases toward each new status frame (about 80ms, exponential).
- **Camera framing:** from a three-quarter view (azimuth -55°, elevation 20°), the camera fits the arm, as it stands, and its cart into the screen inside 14px margins, keeping room for the plan bar: an 80px band along the bottom of a tall screen (at most 30% of a short one), or a 272px column at the right of a wide, short one, where the picture then takes the full height. It centres the picture in that space and fits a sample of the meshes' own points, not their boxes, so the cart's empty box corners don't shrink the arm. It re-frames when the screen is resized unless the user has turned or zoomed the view (and never during a drag), and Reset view re-frames it. In Tool mode the framing also keeps the whole gizmo on screen around the flange of the shadow arm, and the band along the bottom grows to the plan bar's own height while it shows (up to half the screen); when a newly staged pose, or the bar growing, leaves the gizmo's arrows near an edge or under the bar, the view re-frames, unless the user has turned or zoomed it. Dragging away from the rings or gizmo turns the view.

### Mode keys
Two small keys joined into one block in the ARM bank, like a desk's source selector: Joints and Tool, 58px wide and 30px tall, sharing a 1px rim with 4px outer corners. They choose what dragging in the 3D view moves. The one in use stays pressed in: a sunk face (a #25282c to #2d3136 gradient with an inner shadow) inside a rim that stays put, so the joined block keeps a straight outline, with its legend in Legend Ink, 2px lower; the other stands up as a small key with a Printed Grey legend. They're disabled until the 3D view is up. The choice is remembered on this PC, and switching cancels a drag but keeps the plan, so a pose can be built from both.

### Joint rings
Shown in Joints mode.
- **Rings:** one torus per joint in that joint's tape colour, drawn twice: solid where it's in front, and faint (24%) where the arm hides it.
- **Locked:** all rings dim to 30% when drags are locked; during a drag, the other rings dim the same way.
- **Hover:** within 14px of a ring, it lightens, gains a translucent band and the cursor becomes a grab hand.
- **Drag:** the cursor becomes a grabbing hand and an amber arc on the ring shows the sweep (0.5° steps, 5° with Shift), with the shadow arm, the tool path and the floating readout; releasing stages the joint at that angle. The rings ride on the shadow arm, so each drag carries on from the staged pose.

### Tool gizmo
Shown in Tool mode, at the flange of the shadow arm (so it rides on the staged pose), with its axes along the arm's base frame: the X, Y and Z the tool rows and the controller use.
- **Form:** three arrows, each a 3px shaft from 16px to 74px out and a 20px cone tipped at 94px, and three rings 58px in radius, each about its arrow's axis. A printed letter (X, Y, Z: 18px bold, in its axis colour, with a 2px Well Black outline) sits 108px out along each arrow, and each ring's name (Rx, Ry, Rz, 13px, the same print) sits just outside it, 76px out, at whichever of its four diagonals reads best from the camera: on the ring's near side, clear of the arrows, their letters and the other names. A name holds its place until another spot is clearly better, so the names don't flicker as the view turns. Each axis wears its colour on its arrow, ring, letter and ring name (X Axis Red, Y Axis Green, Z Axis Blue) over a 1.7px Well Black rim, so it reads over the light arm and the dark well alike, and the gizmo is drawn over everything, like the tool path. It keeps that size on screen at any zoom.
- **Depth:** each ring's far half, the one facing away from the camera, is drawn at 35%, so the cage reads in depth over the arm; a ring seen face-on has no far half and is drawn whole.
- **Axis colours:** red, green and blue for X, Y and Z, as in other 3D tools, at the owner's request (see the Axis Colour Rule). The printed letters stay, so colour is never the only cue.
- **Locked:** the whole gizmo dims to 30% when drags are locked; during a drag, the other handles dim the same way.
- **Hover:** within 12px of a handle on screen, it brightens a third of the way to white, keeping its hue, and gains a translucent band of its colour, and the cursor becomes a grab hand.
- **Arrow drag:** the dragged arrow lights amber, like a held key, and the flange slides along that axis in 1mm steps (10mm with Shift), following the pointer along the arrow's line; seen end-on, dragging up and down slides it instead.
- **Ring drag:** the dragged ring lights white, and the flange turns in place about that axis, in 0.5° steps (5° with Shift). The turn itself is the amber: a translucent amber wedge (45%) edged by a heavier amber arc on the ring, showing how far, and which way, it turns (both Alarm Red when blocked). On a ring seen nearly side-on, like Rz from the default view, the wedge flattens into the arc, and a small turn near the ring's foreshortened end shows only a short stretch of it; the readout always carries the angle. Seen edge-on, a ring is followed along its on-screen direction instead.
- **The joints follow:** the panel solves the joint angles for each new pose by inverse kinematics, starting from the last, so the arm keeps its elbow and wrist configuration, and the shadow arm, the tool path, the strips' targets and the tool rows all follow. Where the pose is out of reach or would need a joint past its limit, the drag stops at the last pose it can reach, and the readout says why. Releasing stages every joint the solution turns.

### Shadow arm and tool path
- **Shadow arm:** the arm again in translucent, self-lit amber, drawn after a depth-only pass so only its front skin shows. It draws only the links from the first joint that differs from the real arm. It shows a drag in progress, else the staged plan, else a joint goal while the arm is heading for one, so after Move it stays until the arm settles.
- **Tool path:** a 1px amber line along the flange's swept path from the arm to the shadow, shown during a drag and while a plan is staged.

### Floating readout
A black well panel (Well Black at 94%) with a 1px amber border, 3px corners and the floating-readout shadow. It reads "J2 -30.0° → -59.0°" at 13px with the target in amber at weight 650, over a small Printed Grey line ("Release to plan · Esc cancels", or where the joint's drag limit stops it).
- **Ring drag:** it sits outward from the ring's screen centre, past the pointer.
- **Fader drag:** the same readout shows in the monitor, anchored just outside that joint's ring on the side away from the links that follow it; for J6, whose ring sits on the flange, it sits away from J5.
- **Gizmo drag:** it reads the axis and the tool's position in the controller's terms ("Y 56.0 mm → 18.0 mm"), or the turn about it ("Rx −27.5°"), and sits just outside the gizmo toward the pointer; where there's no room on that side, it centres above or below the gizmo instead, 120px from its centre, clear of the letters. Its small line says "Out of reach, so it stops here" or "J3 is at its limit, so it stops here" when the drag has run out of room.
- **Either way:** it is pushed 34px further out and flips sides to stay on screen, so it never covers the ring or its arc.

### Blocked moves
A plan is checked along its whole path, in the browser just as the server checks it, against the arm itself, its cart and the floor, with a 10mm margin. When the path would come closer than that, the shadow arm, the arc, the path line and the floating readout turn Alarm Red (the readout's small line names the parts: "Blocked: the forearm would come within 10 mm of a cart brace"); a red contact mark shows where the path first comes too close: a flat 14px disc, an Alarm Red core in a hard 2px Well Black rim, that keeps its size on screen and is drawn over everything; and the cart part it would hit gets a faint (15%) red overlay with red edges (the floor and the arm's own links get the mark alone). There is no red wash over the scene. On the fader, the goal frame turns red and the target line reads "Blocked"; the plan bar's note and the state window's sentence say what would come too close to what. Move stays pressable but unlit, and a press is refused (see Plan bar).

### Plan bar
Docked along the bottom of the 3D screen while a pose is staged: Desk Charcoal at 95%, a 1px seam, 4px corners, 8px by 10px padding and the plan-bar shadow, 10px in from the screen's edges. Three columns:
- **Legend:** PLAN, a printed legend.
- **Joints and note:** one chip per staged joint, in a Well Black chip with a 1px seam and 3px corners: its tape tag (J2), "−30.0° → −20.0°" at 13px, and a small key cap (24px by 22px, 3px corners) printed × that takes that joint out of the plan. On a 3D screen 520px wide or narrower the chips drop the from-angle ("J2 → −20.0°"; the strips still show it), so a tool move's four or five chips stay low over the arm. Beside or under the chips, a 12px Printed Grey note says what Move will do ("Moves J2, J3 together, at up to 10°/s."), or in semibold Alarm Red what blocks it. When the move at the chosen speed would carry the flange past the 250 mm/s redline, the note turns Alarm Red at regular weight and adds how fast ("…; the flange would reach about 480 mm/s, past the 250 mm/s redline.").
- **Keys:** Clear (a small key, 36px tall) and Move together at the right. Move is at least 104px by 40px, with a 15px uppercase Backlight Amber legend and an amber lamp along its lower edge while the plan can go. When it can't (blocked, motors off, stale status, or the arm still moving) Move stays pressable but unlit, a Printed Grey legend on a plain cap, and a press is refused: the refusal ring for 450ms and the reason on the message line. It's disabled only while the plan is being sent. After Move, the plan hands over to the arm's goal and the bar goes; the shadow arm stays until the arm settles.
- **Docking:** along the bottom, 10px in from the screen's edges, on a tall screen. On a wide, short one (at least 640px wide and more than 1.6 times as wide as tall, like the stacked side panel) it docks at the bottom-right, 248px wide, with its parts stacked: the help line moves into the same column, and the camera keeps that column free and frames the arm in the full height.

### Tool axis row
The same channel ground laid flat: the axis name, a − key, the reading (19px, one decimal and its unit, with an 11px "→ target" line below) and a + key. The target line is amber while the tool is moving to a commanded pose or a drag is under way, and Printed Grey for the pose a staged plan would put the tool at, however it was staged.

### Lamps
9px circles: unlit grey while connecting, Motors-On Green when live, Alarm Red when dead. When the status is more than 1s old the lamp turns slow: plain grey (#8f8b83), with no ring, and the link text reads "Live · <address> · status 1.4 s old".

### Message line
A well pinned to the bottom of the viewport (`position: sticky`): the latest message at 13px, coloured by tone (Legend Ink when done, Backlight Amber while a hold or move is pending, semibold Alarm Red on failure), with a Printed Grey hint at the right. It also reports a program run's changes as they happen: halted (with the halt's message), paused, faulted, stopped or finished.

### Program, Teach and Cell sections
Three more desk sections below Tool (stacked under the monitor at 1240px and narrower), built only from the desk's existing parts. Each opens with a bank row (its title, then its main controls) and is otherwise flat Panel Grey with groups split by 1px seams.
- **PROGRAM:** the bank holds a program select, Delete, a New name field with Create, and the Override fader. Below it, the run window beside the run keys, the step list, the add-step row and the run log.
- **TEACH:** the bank holds the freedrive key and the Sensitivity fader. Below it, the record row (name, Record as [Tool pose | Joints], In frame, Record here) and the add row (Then add [No move | Straight move | Joint move], with a cue naming the program and where the move goes).
- **CELL:** a Printed Grey summary line (counts, the tool in use, the file and its revision), then four groups: Tools (list, form, touch-fit row), Frames (list, three-point touch row), Targets (list) and Keep-out zones (list, form).

### Run window
The state window's smaller twin, at the left of the program's run panel: an 18px uppercase word over the 13px sentence, sharing the state window's tones (see the Brightness Ladder Rule). The word says what the run is doing (NO RUN, RUNNING, STEPPING, DRY RUN, PAUSING, PAUSED, HALTED, STOPPING, STOPPED, FAULTED, DONE, PLANNED, MOVING, ARRIVED) and the sentence where (program, step number and the step, with "· asking the brain" while the run waits for the brain's answer), why it stopped, and what to do next ("Resume re-plans from where the arm is.", "Move the target or the zone, or change the step, then press Reset."). While no run can start, it adds why ("The motors are off: hold Enable.", or for a program with planned or learned steps, "tend needs the brain, which isn't answering at http://127.0.0.1:8770: start it (python brain/stub/stub_brain.py)."). The bridge's state window says RUNNING (lit amber) while a program moves the arm, FREEDRIVE (lit amber) in freedrive, and names a paused or halted run in its READY sentence.

### Run keys
Six keys in a 3 by 2 grid beside the run window, 36px tall with 13px semibold legends: RUN, Step, Dry run over Pause (Resume while paused), Stop run, Reset. RUN is printed uppercase in Backlight Amber with Move's amber lamp along its lower edge while a run can start. RUN, Step and Dry run behave like Move: when they can't start they stay pressable but unlit (Printed Grey legend, no lamp), and a press is refused, with the refusal ring and the reason on the message line. Pause, Stop run and Reset are disabled when they don't apply.

### Step list
A well (Well Black, 1px seam, 3px corners, the meter-well shadow, at most 300px tall, scrolling) of rows, one per step: a 9px lamp, a flat full-width button with the step's number (Printed Grey, 12px) and its sentence ("Straight move to vise_part at 20 mm/s"), then its mini keys. Rows are Channel Charcoal with a 1px seam and 3px corners, at least 34px tall, and grow with their text.
- **Lamps:** unlit grey; Backlight Amber while the run moves through the step; a hollow 2px amber ring while the run waits at it (paused, halted or stepped), with its number in amber; Alarm Red, with its number, where a run faulted; plain grey during a dry run. The list scrolls itself (never the page) to keep that step in sight.
- **Selecting:** clicking a row's sentence selects it (aria-pressed), lifting the row to a Key Cap face with a Cap Rim edge. New steps and recorded moves go after the selected step; the add key and the record cue say so ("Add after 3", "after step 3").
- **Nesting:** a repeat's steps, and an if's branches, are indented 14px per level (an "Else" label row separates the branches). The page edits the top level and repeats; ifs, calls and branches are edited in the program's JSON.
- **Comments** are printed in Printed Grey. A move to a deleted or now-unreachable target carries an Alarm Red cue ("no such target").
- **Steps the brain takes part in** read as sentences like any other ("Planned move to C, 50 mm back along the tool"; "Learned skill seek near A, within 60 mm, up to 30 s, then tool.1 = 1"), followed by a Printed Grey cue naming the brain's part ("· the brain plans the path", "· the brain acts"). When the brain isn't answering, or hasn't the skill, the cue is Alarm Red instead ("brain not answering", "the brain has no skill seek"), as a missing target's is.
- **Adding steps:** the add row under the list picks a step type in a select and shows only that type's fields. A planned move takes a target, Back and Speed, as a joint move does; a learned skill takes its skill (a select of the brain's skills, reading "No brain" while it isn't answering), NEAR a target, WITHIN (mm), UP TO (s) and Speed, so the row reads as the step's sentence. A step that can't be added yet is refused with the reason on the message line ("Start the brain … to list its skills").
- **Empty:** the well prints how to add steps.

### Mini keys and two-press deletes
Row keys are mini keys: 22px tall, at least 26px wide, 3px corners, 12px semibold Printed Grey legends, printed with their action ("Run from", "Plan", "Re-touch", "Use", "Edit") or the panel's glyphs (↑, ↓, ×), each with an aria-label naming its row. A delete (×, or a program's Delete) takes two presses: the first arms it for 3 seconds, its legend turning to "Sure?" in Alarm Red, and the message line says to press again.

### Fields
Text and number fields and selects are sunk like a fader slot: Well Black, a 1px seam, 3px corners, the slot-well shadow, 30px tall, 13px Legend Ink in the page's one family with tabular figures; numbers 64px wide, names 132px. Focus is the 2px Legend Ink ring, tight (no offset). Selects are used only for open-ended lists (programs, step types, targets, frames). In a text field Space types a space; everywhere else on the page it is still STOP.

### Mode keys in forms
Two- and three-way choices use the 3D monitor's joined mode keys: interlocked 30px keys, the chosen one pressed in (aria-pressed) with its legend in Legend Ink, the others standing up in Printed Grey: Record as [Tool pose | Joints], Then add [No move | Straight move | Joint move], an input or output on the [Tool | Controller], and a value [On | Off]. A choice that can't apply is disabled (a joint target can't take a straight move).

### Run log
A well like the step list, at most 190px tall, of 12px Printed Grey lines: the time (HH:MM:SS, in Scale ink #8f8b83) then a sentence ("5: Straight line to vise_part, 50.0 mm, at 20 mm/s, about 2.6 s"; "3: Path to C (offset) from the brain, 3 segments at up to 15°/s, about 6.2 s (the brain: up and over, 120 mm above the start and goal)"; "2: burst 4: Burst to seek's next pose from the brain, 1 segment at up to 15°/s, about 0.3 s"; "Paused: the operator pressed Pause"). Faults, stops, interruptions and refusals of the brain's proposals are Alarm Red. It follows the newest line unless scrolled back. Every run is also written to cell/logs.

### Freedrive key
A hold-to-commit key like Enable, at least 170px wide: a lamp and a 15px label over an 11px sub-line.
- **Hold:** holding fills a 5px Backlight Amber bar along the bottom edge over 700ms; letting go early drains it and the message line says to keep holding.
- **On:** the cap stays charcoal (keys light amber only while held), its lamp and its legend ("Freedrive on") lit amber, its sub-line "Press to leave"; one press leaves freedrive. The bridge's FREEDRIVE window carries the solid amber.
- **Disabled:** its sub-line says why ("Locked: motors off", "Locked: a program is under way").

### Override and sensitivity faders
Detented faders like the step and speed faders, built the same way: Override (10 to 100% in 10s, its unity mark at 100) in the program's bank, following the server's value; Sensitivity (the controller's teach sensitivity, 1 to 5, its mark at 1) in the teach bank, following the level the server set while freedrive is on.

### Cell rows
The tools, frames, targets and zones are rows in wells like the step list: the name in 650 weight, then its details in Legend Ink ("in vise · (0.0, 0.0, 80.0) mm · (83.2, -62.5, 117.5)°"), then a Printed Grey cue ("in use", "touched 2026-10-22 04:12") or an Alarm Red one ("out of reach since its frame moved"), then mini keys: tools Use, Edit, ×; frames Re-touch, ×; targets Plan, ×; zones ×. A target's Plan stages a joint move to it as the panel's plan (the shadow arm, path check and plan bar), and Move sends it; nothing moves from a row. Touch keys (Touch, Origin, +X, XY) and Fit are small keys; the cue under each touch row counts the touches taken and what's left.

### Cell and faults in the 3D monitor
- **The cell** is drawn in neutral tones, never the axis colours: each keep-out zone as a box with a 7% Printed Grey fill and Scale-ink (#8f8b83) edges; each frame as a 70mm Legend Ink line along its x axis and a 45mm Scale-ink line along y, with its name printed small and uppercase in Printed Grey over a Well Black outline; each target as a small light dot with a Well Black rim. The active tool's shape and the zones join the 3D view's collision check, as on the server.
- **A faulted step** that would have collided is drawn as a blocked plan: the Alarm Red shadow arm at the pose where it would have come too close, the contact mark, and the red overlay on the zone or part it would hit. Reset, or a new run, clears it.

### Named Rules
**The Biggest Key Rule.** No key is bigger or louder than STOP: the only red face, the largest legend on the page, the full height of the bridge, with its shortcuts printed on it.

**The Hold-to-Commit Rule.** Switching the motors on is a hold, not a click: the green bar must fill the key's lower edge before anything happens. Freedrive is held on the same way, with an amber bar.

**The Stage-then-Move Rule.** Dragging a ring, a gizmo handle or a joint's fader cap, or pressing Plan on a target, only stages: the shadow arm, arc, path and readout show the plan, and nothing moves until Move sends every staged joint as one move. Esc backs out a drag, then the plan; a blocked plan can't be sent, and the server checks every plan again. Program runs are the deliberate exception: RUN, Step and Run from start the server's runner, which plans and checks each move itself before sending it, and Dry run shows the whole program's plan without moving.

**The Dash Rule.** An unknown value is an em dash (—) with an empty meter and dark lead lights, never a zero and never the last known number.

## Do's and Don'ts

### Do:
- **Do** keep every key except STOP a charcoal cap with 4px corners, a 1px rim and a 2px hard bottom edge, lit amber only while it is held.
- **Do** print group and setting names as uppercase legends (11px, 600, 0.08em, Printed Grey), and channel names on their tape (12px, 700, uppercase, Marker Black).
- **Do** give each joint its tape colour, in joint order, on its scribble strip, meter column and 3D ring only.
- **Do** show a commanded value only while the joint or axis is still travelling to it: a hollow amber frame on the meter, amber lead lights counting the steps to go, an amber "→ value" line and, for joints, the amber shadow arm. A staged angle is planned, not commanded: a dashed frame and a grey "→ value" until Move sends it.
- **Do** keep the 3D monitor a sunk screen in the desk, with the joints' own rings, or the gizmo at the flange, as its controls.
- **Do** colour the gizmo's axes red, green and blue (X, Y, Z) over a Well Black rim, and still name its axes and rings with printed letters; light a dragged arrow amber, and draw a ring's turn in amber.
- **Do** set the floating readout just outside the ring, outward from its centre (and, for a fader drag, away from the links that follow), so it never covers the ring or its arc.
- **Do** keep STOP the biggest key, red, at the top-right of the sticky bridge, with its keyboard shortcuts printed on it.
- **Do** set step and speed on detented faders that can only rest on values the arm accepts, with the value above and the range printed under the ends; start the speeds slow every time.
- **Do** print a redline on the speed faders, like a speedometer's red zone with its figures, wherever the flange could pass 250 mm/s, and let the joint speed redline follow the arm's pose and any staged plan, whichever is lower.
- **Do** make each state-window tone differ from the others in brightness as well as hue.
- **Do** draw the focus ring in Legend Ink (2px, 2px offset; tight around a fader's slot).
- **Do** put displays in recessed wells, and keep sections and strips flat with 1px seams.
- **Do** keep text at 4.5:1 or better against its ground, as every built pair does except Printed Grey on key caps and STOP's dimmed sub-line.

### Don't:
- **Don't** float grey round buttons over a 3D view of the arm, as Umbratek Studio's Live Control does.
- **Don't** make any key round or pill-shaped; only lamps and the 3D monitor's rings are round.
- **Don't** use the axis colours anywhere but the gizmo and its legend sample, or let them report state: a dragged arrow lights amber and a blocked move goes Alarm Red, as everywhere.
- **Don't** give any key but STOP a red face; a refused key gets a red ring and legend on a darkened cap.
- **Don't** wash the 3D scene red for a blocked move; a red contact mark and a red overlay on the part it would hit say where.
- **Don't** use amber for focus or decoration; it means a held key or motion.
- **Don't** use green or red as decoration, or tape colours for state.
- **Don't** make lights glow; lamps, lead lights, rings and lit windows are flat, and the only coloured shadows are hard 2px rings.
- **Don't** show a zero or the last known number when a value is unknown; show an em dash and an empty meter.
- **Don't** add a second type family or proportional figures.
- **Don't** put shadows on sections or strips; shadows belong to keys, fader caps, wells, the sticky bridge, the floating readout and the plan bar.
- **Don't** mark a list row's state with a coloured stripe; a step's state is its lamp, as a channel's is.
- **Don't** use native checkboxes or two-option selects for a choice; use joined mode keys.
