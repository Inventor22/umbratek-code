# The spine and the brain: the interface, version 1

The arm is run by three parts. The **spine** is this repo's server (`scripts/`, standard-library Python): it owns the
controller link, teaching, programs, the runner, and the safety shell that every motion passes through. The **brain**
is a separate process with heavy dependencies (`brain/`, its own environment): perception, a world model, a motion
planner and learned skills. The **twin** is a simulated arm and cell that takes the same commands as the real arm. This
document specifies how the spine and the brain talk. The rule underneath all of it: **the brain only proposes; the
spine decides.**

A program's steps are of three kinds, each followed by a check:

- **taught:** the moves, waits and I/O that exist already, to targets taught by hand; each move ends with the
  executor's arrival check;
- **planned:** a move from wherever the arm is to a taught target, its path solved afresh by the brain each run, around
  what's in the cell; it too ends with the arrival check;
- **learned:** a skill (a policy) that handles the variable part, acting in short bursts at 10 Hz or slower, inside a
  region around a taught target; it ends when the skill says it's done, and then the step's own check (an input)
  decides.

The interface in code:

| File | What it is |
| ---- | ---------- |
| `scripts/brain_protocol.py` | The messages, their limits and the checks both ends make. A brain written in Python imports it. |
| `scripts/brain_link.py` | The spine's end: asks the brain one question at a time, with deadlines, on loopback only. |
| `scripts/safety.py` | The safety shell: checks a proposal whole, refuses it with every reason, or makes it a plan for the executor. |
| `scripts/runner.py` | Planned and learned steps in programs: validation, runs, dry runs, STOP and pause. |
| `brain/stub/stub_brain.py` | A stub brain speaking this interface: a simple planner and one scripted skill, standard library only. |
| `tests/spine_tests.py` | The tests: the protocol, the shell, and the whole spine against the simulator with stub and buggy brains. |

## The link

- **The brain is the server; the spine is its only client.** The brain serves HTTP/1.1 on a loopback address (by
  default `http://127.0.0.1:8770`; `jog_server.py --brain URL` picks another, `--brain off` none). The spine asks one
  question at a time: a `POST` of one JSON object to `/v1/hello`, `/v1/plan`, `/v1/act` or `/v1/end`, answered with
  one JSON object, `Content-Type: application/json`, UTF-8, at most 1 MiB either way. The brain never speaks unless
  asked: it can't move, start, pause, resume or stop anything.
- **The brain may run in WSL2.** WSL2's default networking forwards Windows' `localhost` to servers in WSL2, but not
  the other way, so a brain serving on WSL2's own `127.0.0.1` is reached from the spine on Windows at
  `http://127.0.0.1:<port>` (checked on this PC: hello in 12 ms, a plan in 0.4 s). With WSL2's mirrored networking it
  works the same. The spine opens no port of its own for the brain (decision D41).
- **This PC only.** The link has no authentication, so the spine refuses any address but a loopback one (`127.x.x.x`,
  `::1`, `localhost`) and won't start with another. Any process on this PC could pose as the brain; that's safe
  because the spine treats every reply as untrusted, and the shell checks all of it.
- **Deadlines.** Connecting must succeed within 1 s (on loopback a listening brain takes the connection at once).
  `hello` must be answered within 2 s, `plan` within 10 s (`--brain-timeout`; a planner's first call can be slow, as
  a GPU planner's is while it compiles its kernels), `act` within 2 s, `end` within 1 s. A reply that's late, an HTTP
  status other than 200, or no connection, is no answer: the step faults, saying so.
- **Abandoning.** While the spine waits, a STOP or a pause abandons the question at once. The reply, if it comes, is
  dropped: the next question has a new id.

## Messages

Every message is a JSON object with:

| Field | Meaning |
| ----- | ------- |
| `v` | The protocol version: `1`. A reply in any other version is refused. |
| `type` | `"hello"`, `"plan"`, `"act"` or `"end"`; a reply has its request's. |
| `id` | Chosen by the spine, unique (`plan-123456-7`); the reply echoes it. Up to 80 of letters, digits, `_ . : -`. |
| `stateSeq` | In replies to `plan` and `act`: the `state.seq` of the request, so the reply is tied to the state it was worked out from. |

Units are the spine's everywhere: joint angles in degrees (J1 to J6), lengths in millimetres, poses as
`{"xyz": [mm], "rpy": [deg]}` with R = Rz(yaw) Ry(pitch) Rx(roll), in the arm's base frame. Every number must be
finite; JSON has no NaN or infinity, and the spine refuses Python's extensions that write them (`NaN`, `Infinity`,
`1e999`). A peer ignores fields it doesn't know, so fields can be added within version 1; changing what a known field
means needs version 2.

### What the spine tells the brain

`state`, in every `plan` and `act` request, is the arm as the spine measured it:

```json
{"seq": 4821, "ageS": 0.031, "joints": [6.2088, 17.2419, 129.6985, 22.4511, 90.0, 81.2104],
 "tool": {"xyz": [382.1, -100.4, 220.0], "rpy": [180.0, 0.0, 15.0]},
 "inputs": {"tool": 2, "controller": 0}}
```

`seq` numbers the controller's 10 Hz status frames, `ageS` is how old the frame was when the request went, `tool` is
the active tool point's pose, and `inputs` (in `act` only) are the digital inputs as 16-bit words (`tool.1` is bit 1 of
`tool`). A dry run's state has `"seq": 0` and `"virtual": true`: it's where the run would be, not a measurement.

`world` is what the spine knows of the cell: the active tool's point and shape, and the keep-out zones as boxes in the
base frame. The arm's own links and its cart are fixed: a brain takes them from the repo (`scripts/model/`, as
`scripts/utra_model.py` reads them).

```json
{"cellRevision": 12, "marginMm": 10.0,
 "tool": {"name": "pointer", "tcp": {"xyz": [0, 0, 100], "rpy": [0, 0, 0]},
          "capsules": [{"a": [0, 0, 0], "b": [0, 0, 90], "r": 12}]},
 "zones": [{"name": "post", "centre": [346.4, 25.6, 200.0],
            "axes": [[0.966, 0.259, 0.0], [-0.259, 0.966, 0.0], [0.0, 0.0, 1.0]], "size": [60, 60, 200]}]}
```

`limits` are the limits the shell holds the brain to, so it can plan within them: `joints` (the planning limits,
±180°, J3 ±165°), `ratedDegS` (each joint's top speed), and for `plan` `maxWaypoints`, for `act` `burstMaxWaypoints`,
`burstMaxJointDeg`, `burstMaxToolMm`, `periodS` and `holdMaxS`.

### hello

Who the brain is and what it can do. The server asks every 5 s, for the panel, and before a run that needs the brain.

```json
{"v": 1, "type": "hello", "id": "hello-123456-1", "spine": "umbratek-code jog_server", "versions": [1]}
{"v": 1, "type": "hello", "id": "hello-123456-1", "brain": "stub-brain 1", "planner": true,
 "skills": {"seek": "Spirals the tool point out from where the step began ..."}}
```

### plan: a path for a planned move

The spine sends where the arm is and the joints the move must end at. It works the goal out itself, from the target's
joints as taught, so the arm arrives in the taught configuration and the brain only finds the way.

```json
{"v": 1, "type": "plan", "id": "plan-123456-7", "state": {...}, "world": {...}, "limits": {...}, "timeoutS": 10,
 "goal": {"joints": [32.15, 25.61, 116.42, 31.3, 90.0, 92.37], "target": "C", "tool": {"xyz": [...], "rpy": [...]}}}
{"v": 1, "type": "plan", "id": "plan-123456-7", "stateSeq": 4821, "ok": true,
 "path": [[6.2088, 17.2419, ...], [...], [...], [32.15, 25.61, ...]], "note": "up and over, 120 mm above"}
{"v": 1, "type": "plan", "id": "plan-123456-7", "stateSeq": 4821, "ok": false, "reason": "no clear path"}
```

`path` is 2 to 64 waypoints of six joint angles, the first the `state.joints` it was sent and the last the
`goal.joints`. Each pair of waypoints is a segment, which the controller runs as one joint move (a straight line in
joint space, from rest to rest). Fewer waypoints run faster. `note` and `reason` are for the log (300 characters at
most).

### act: the next burst of a learned skill

```json
{"v": 1, "type": "act", "id": "act-123456-12", "skill": "seek", "params": {"stepMm": 8}, "episode": "174206-s2-1",
 "tick": 3, "state": {...}, "anchor": {"xyz": [382.1, -100.4, 220.0]}, "withinMm": 60, "world": {...},
 "limits": {...}, "timeoutS": 2, "remainingS": 26.4}
```

`episode` names this run of the step (it's the same for every tick until the step ends), `tick` counts the requests in
it from 0, `params` are the step's own parameters for the skill (a JSON object of at most 2000 bytes), `anchor` and
`withinMm` are the step's region (the tool point must stay within `withinMm` of `anchor`), and `remainingS` is the
time the step has left. The reply has one `action`:

| Action | Meaning |
| ------ | ------- |
| `{"kind": "move", "path": [[...], ...], "speed": 10}` | A burst: the waypoint the arm is at, then up to 10 more. `speed` (°/s, optional) can only slow it. |
| `{"kind": "hold", "seconds": 0.5}` | Nothing moves; ask again after up to 1 s. |
| `{"kind": "done", "note": "..."}` | The skill has finished. The spine then reads the step's check, if it has one. |
| `{"kind": "fail", "reason": "..."}` | The skill gives up: the step faults with the reason. |

The spine asks for the next burst once the last has run and a newer status frame shows where the arm ended, and never
sooner than 0.1 s after the last request: the skill acts at 10 Hz at most, and slower whenever a burst takes longer.

### end

Sent, without waiting, when a learned step's episode ends, so the brain can reset or learn from it:
`{"v": 1, "type": "end", "id": ..., "episode": ..., "skill": ..., "outcome": ..., "reason": ...}` with `outcome` one of
`done`, `failed` (the skill gave up, or its check read wrong), `refused` (the shell refused a burst), `stopped`,
`paused`, `faulted` (anything else: the controller, the link) or `timeout`. The reply is just the envelope.

## What the spine checks before anything moves

The brain's reply is checked whole, and if any part fails nothing of it is sent: the step faults, the log has every
reason, and where a segment would have collided, the 3D view shows where (`scripts/safety.py`).

| Check | Refused when | For example |
| ----- | ------------ | ----------- |
| The protocol | not JSON, another version, a number that isn't finite, a field malformed, too many waypoints, over 1 MiB | `the brain's reply is malformed: path[1] is [...], not 6 joint angles` |
| Staleness | the reply answers another request (`id`), or was worked out from another state (`stateSeq`) | `the brain's reply answers request "plan-000000-1", not 'plan-174206-9': it's stale` |
| Where it starts and ends | the first waypoint isn't where the arm was (or a planned path's last isn't the goal), within 0.01° | `refused the brain's plan: it starts 2.00° (J1) from where the arm is` |
| Joint limits | any waypoint outside the planning limits (then every point between is inside too) | `waypoint 1 has J3 at 170.0°, outside -165° to 165°` |
| Collisions | any segment brings the arm or tool within 10 mm of the arm itself, its cart, the floor or a keep-out zone, checked continuously along the joint move (`utra_model.check_path`, as every move is) | `segment 1: the tool would come within 10 mm of the post zone` |
| Speed caps | never refused: each segment is sent at the step's speed (or the brain's, if slower), each joint within its rated speed, and slowed until the tool point can't pass 250 mm/s (or 1000 mm/s if the program is marked `"fullSpeed": true`) | `Path to C from the brain, 3 segments at up to 18.8°/s, keeping the tool point under 250 mm/s` |
| A burst's length | any joint turning more than 15° in a burst, or the tool point travelling more than 50 mm | `it moves the tool point 60 mm, more than the 50 mm a burst may` |
| A burst's region | the tool point, sampled every 0.1° of turn, leaving `within` mm of the `near` target | `segment 1 takes the tool point 45 mm from A, beyond the 30 mm the step allows` |
| Freshness, just before it runs | the controller's status is over 1 s old, the arm has moved more than 0.1° since the brain was asked, or the cell has changed | `the arm has moved 0.50° (J1) since the brain was asked` |

What passes becomes an ordinary plan for the supervised executor (`scripts/executor.py`): each segment is sent alone
and followed on the status until it arrives, within its path tolerance and time budget, before the next is sent, as
every move is. The executor checks for a STOP or pause before each segment.

## STOP, pause and faults

- **STOP is absolute.** STOP (the key, Space or Esc) stops the arm over the controller connection at once, whatever a
  step is doing: waiting for the brain, being checked, or running. The run then ends as *stopped*, whatever else went
  wrong meanwhile. A reply that arrives after STOP is dropped, and a move that reaches the controller in the same
  instant is discarded by it, as it's stopped.
- **Pause** (a STOP too, decision D20) abandons a question to the brain or stops a path mid-segment. Resume asks the
  brain again, from wherever the arm is: a planned move gets a fresh path, a learned step starts a new episode.
- **A refusal, no answer, or the skill giving up faults the run** (stopping the arm, though nothing of the refused
  proposal was sent). Reset clears it, as for any fault. Refusals aren't retried: a brain that proposes something
  unsafe is a bug to fix.

## Steps in programs

```json
{"type": "move", "kind": "planned", "target": "C", "offset": {"frame": "tool", "xyz": [0, 0, -50]}, "speed": 15}
{"type": "learned", "skill": "seek", "near": "A", "within": 60, "seconds": 30, "speed": 10,
 "params": {"stepMm": 8}, "check": {"input": "tool.1", "value": 1}}
```

A planned move takes a move's fields: a `target`, an optional `offset` (as for joint moves) and an optional `speed`
(°/s, or the program's joint speed), times the override. A learned step names its `skill`, the target it works `near`,
how far the tool point may go from it (`within`, 5 to 500 mm, default 100), how long it may take (`seconds`, 1 to 3600,
default 60), an optional `speed` (°/s), `params` for the skill, and an optional `check` that must read true once the
skill says it's done. A target that a planned move or a learned step uses can't be deleted.

A run that needs the brain asks it first, and won't start if it isn't answering, doesn't plan, or lacks a skill.
A dry run asks the brain for every planned move's path and checks it as a run would, so its estimate includes the
path; it assumes a learned step ends where it began.

## Writing a brain

A brain in any language follows this document. A brain in Python can use `scripts/brain_protocol.py`; this one
plans straight paths and has no skills (the spine refuses its paths whenever a straight one isn't clear):

```python
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, "scripts")  # run from the repo's root
import brain_protocol as bp


class Brain(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_POST(self):
        kind = self.path.rsplit("/", 1)[-1]
        try:
            request = bp.check_request(bp.decode(self.rfile.read(int(self.headers["Content-Length"]))), kind)
        except (bp.ProtocolError, TypeError, ValueError):
            return self.send_error(400)
        if kind == "hello":
            reply = bp.reply(request, brain="straight 1", planner=True, skills={})
        elif kind == "plan":
            reply = bp.reply(request, ok=True, path=[request["state"]["joints"], request["goal"]["joints"]])
        elif kind == "act":
            reply = bp.reply(request, action={"kind": "fail", "reason": "this brain has no skills"})
        else:
            reply = bp.reply(request)
        data = bp.encode(reply)
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


HTTPServer(("127.0.0.1", 8770), Brain).serve_forever()
```

The stub brain (`brain/stub/stub_brain.py`, see `brain/README.md`) is a fuller example: it plans around the zones it's
told about, keeps state per episode for its skill, and has `--ignore-zones` to play a buggy brain.
