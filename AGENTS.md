# For coding agents

## What this is

Software that drives an Umbratek UTRA 6-1000 arm from a Windows PC: the jog panel and its server, programs and the
cell, the safety checks every move passes, the brain interface with a stub brain, a stand-in for the arm's controller
(`tools/utra_sim.py`) and a MuJoCo twin. Umbratek's SDK is the `ut_sdk_python` submodule. [README.md](README.md) shows
it; [docs/decisions.md](docs/decisions.md) says why it's built as it is. "The owner" in the docs is the arm's owner,
the person you're working for. Related public repos: [umbratek-cad](https://github.com/Inventor22/umbratek-cad) (the
arm, its cart and cell; `scripts/model/cart/` is built from it) and
[umbratek-end-effectors](https://github.com/Inventor22/umbratek-end-effectors) (pen holders, and the chess gripper
whose servo sketch is `arduino/Chess Gripper/`).

## Where things live

[docs/repo-layout.md](docs/repo-layout.md) lists every file. The spine is `scripts/`, standard-library Python that
every motion goes through: the panel (`jog_server.py`, `jog_panel.html`, `web/`), the cell, planning and programs
(`cell.py`, `planner.py`, `executor.py`, `runner.py`), the brain's interface and the checks on it (`safety.py`,
`brain_protocol.py`, `brain_link.py`), and the kinematics and collision model (`utra_model.py`, `model/`). `brain/`
holds the stub brain and the twin; `tools/` the simulator and the capture, picture and model tools; `tests/` the
suites. `data/captures/` is what the real arm measured, kept byte for byte: suites check against it, so don't edit it.

## The real arm is off limits unless the owner asks

The arm is irreplaceable: if its controller is bricked or misconfigured, there's no vendor left to restore it.

- Work against the simulator and the twin. Give the panel, `scripts/arm.py` and SDK code only 127.x.x.x addresses;
  the panel then reports `"simulated": true`.
- Never connect to, move or reconfigure the real arm or its controller (its ports, Umbratek Studio, settings,
  firmware, calibration) unless the owner asks for that in the current session. That includes `scripts/arm.py find`
  (it scans this PC's networks for the controller), any address that isn't loopback, the notebooks (they move the arm)
  and `tools/capture/`, whose recorders and probes are for use with the owner present (`capture.py analyze` and
  `fit_sim.py` only read files).
- Assume the arm is powered off. If a task needs it, stop and ask the owner to turn it on; don't work around it.
- When he does ask, he stays at the arm with the E-stop in reach while the motors are on. It never runs unattended:
  its controller's own collision detection is off.

## This repo is public

It's public on purpose. Add only what the owner has chosen to make public: no secrets or tokens
(`UMBRATEK_PANEL_TOKEN` included), no paths, hostnames or accounts from his PC, and nothing from or about his private
projects; if unsure, ask before committing. What was moved out (listed at the top of `docs/decisions.md`) stays out.
Start from `origin/main`: local branches from before the 2026-10-23 history rewrite must not be merged, pushed or
built on (this clone's pre-push hook refuses to push them). Push, or open or merge pull requests, only when asked.

## Setup and tests

PowerShell on the owner's Windows PC, where `python` is 3.9 and `py -3.12` is installed; the spine needs 3.8+ and only
the standard library. Node runs the `js` suite.

```powershell
git submodule update --init      # Umbratek's SDK; a new worktree doesn't have it
python tests/run_all.py --list   # [oracle] and [brain] mark the suites that need an environment
$B = 129                         # your base: see "Beside other sessions"
python tests/run_all.py --base $B inproc cli    # the suites you touched; full output in tests/logs/<suite>.out
python tests/run_all.py --base $B --python (py -3.12 -c "import sys; print(sys.executable)") inproc   # code on 3.12
$env:UTRA_TEST_HOST = "127.0.0.$B"; $env:UTRA_TEST_HTTP_PORT = 8800 + $B - 11   # before running a script directly,
python tests/tending_tests.py scenario_grippers                                 # else it uses 127.0.0.1 and port 8799
```

Spine, tending and twin scripts take scenario or section names. The environments are per checkout and git-ignored, so
a new worktree has neither and those suites report SKIP, failing the run. Building them wasn't re-run for this file:

- `.venv-oracle` (collide, oracle, geometry, feature): see [tests/README.md](tests/README.md), or clone another
  checkout's, `conda create -p .venv-oracle --clone <that checkout>\.venv-oracle --offline -y`. Run oracle scripts by
  hand with `conda run -p .venv-oracle python tests/<script>.py`; its `python.exe` alone crashes in NumPy.
- `brain\.venv` (twin): see [brain/README.md](brain/README.md), or set `$env:UMBRATEK_BRAIN_PYTHON` to another
  checkout's `brain\.venv\Scripts\python.exe` (run it; never install into another checkout's environment).

The simulator and the panel by hand, on your own address and port, with a scratch cell and no brain:

```powershell
$log = "$env:TEMP\umbratek-$B"; New-Item -ItemType Directory -Force $log | Out-Null; $port = 8800 + $B - 11
$sim = Start-Process python "tools/utra_sim.py --host 127.0.0.$B" -PassThru -NoNewWindow -RedirectStandardOutput "$log\sim.out" -RedirectStandardError "$log\sim.err"
$panel = Start-Process python "scripts/jog_server.py 127.0.0.$B --port $port --brain off --cell $log\cell" -PassThru -NoNewWindow -RedirectStandardOutput "$log\panel.out" -RedirectStandardError "$log\panel.err"
"simulator $($sim.Id), panel $($panel.Id): http://127.0.0.1:$port"   # stop both when done, and before suites on $B
```

For a brain, start `python brain/stub/stub_brain.py --port <port + 1000>` and give the panel
`--brain http://127.0.0.1:<that port>`. The twin takes `--host` and `--port` too (brain/README.md).

## Beside other sessions

The owner often runs several agent sessions on this PC at once, each in its own worktree. Runs here have collided: a
suite reached another run's jog server (another cell, "A program is running", a 403 for another panel's token), and
leftover simulators and twins kept ports busy and the CPU loaded.

- **Ports are shared silently.** On Windows the HTTP servers here (the panel, the stub brain, the twin's HTTP port)
  start even when their port is taken, and share it; only the simulator refuses. Don't use the defaults (panel 8765,
  stub brain 8770, which is also the panel's default `--brain`, twin 30100, simulator 502, 30001, 30003 and 30099 on
  127.0.0.1); other programs on this PC hold some of them too.
- **Pick a base.** `--base B` takes 127.0.0.B to B+13 for simulators (the twin suite's HTTP port 30100 included),
  panel ports 8800+B-11 to 8813+B-11 on 127.0.0.1, and those + 1000 for stub brains, so bases under 14 apart overlap.
  Use the base your prompt names; otherwise pick one of 101, 115, 129 and 143 at random. Other projects on this PC use
  other loopback ranges, 11 (the default) is the owner's, and tests/README.md's example, 111, overlaps 101 and 115, so
  don't use it. Then check that nothing holds the base's addresses or ports and that no other run uses it (a
  `run_all.py` without `--base` uses 11):

  ```powershell
  Get-NetTCPConnection -State Listen | Where-Object LocalAddress -like '127.*' | Sort-Object LocalAddress, LocalPort | Format-Table LocalAddress, LocalPort, OwningProcess
  Get-CimInstance Win32_Process -Filter "Name like 'python%'" | Where-Object CommandLine -match 'run_all' | Format-List ProcessId, CommandLine
  ```

- **Stop what you started, and only that.** Note PIDs as you start things: background processes outlive their shell,
  and a twin's viewer outlives its twin. Yours listen on your base's addresses and ports (the first command above),
  or have your checkout's path in their command line (suites start theirs that way):

  ```powershell
  $here = (Get-Location).Path
  Get-CimInstance Win32_Process -Filter "Name like 'python%'" | Where-Object { $_.CommandLine -and $_.CommandLine.Contains($here) } | Format-List ProcessId, CommandLine
  wsl -e pgrep -af "utra_sim|brain\.twin|jog_server|stub_brain"   # the same in WSL2
  ```

  Stop them with `Stop-Process -Id <pid>` (`wsl -e kill <pid>`). Never stop processes by name, or one you didn't
  start, even if it's in your way: pick another base or port.
- **CPU.** While other sessions are busy, run only the suites you need. Timing checks can fail under load (`feature`'s
  "estimate within 10% of the real run" does on Python 3.9): rerun alone, with `--jobs 1` or the code on 3.12, before
  calling it a regression. WSL2 forwards only 127.0.0.1, so a twin there clashes with Windows' listeners on its ports.
- **Git and files.** Work on your own branch in your own worktree; leave other branches, worktrees and stashes alone,
  and never edit another checkout's files. `cell/` (git-ignored) is the panel's default `--cell`: in the owner's own
  checkout it's his real cell, programs and run logs, so give servers you start a scratch `--cell`.

## Conventions

- **Commits.** The subject says what changed, as a plain sentence with no prefix or full stop ("Stop the 1 mm
  allowance at a closed guarded zone's face"). The body says why, cites decisions (D121), names the tests and, for a
  fix, that the new tests fail on the commit before ("Tests for each fail on 576c84f"). Pull requests are merged with
  a merge commit, "Merge PR #N: ...".
- **Docs change with the code**, in the same commit. A design decision gets an entry in `docs/decisions.md`, headed
  `## DNNN. The decision, as a sentence` and then **Decision:**, **Alternatives:**, **Why:**, numbered on from the
  last under a dated heading for the piece of work; a later revision is a **Revised ...:** bullet in the old entry. If
  another branch takes the same numbers, the later to merge renumbers. Also update the guide the change touches
  (`docs/` and `brain/README.md`), `tests/README.md` for suites, `docs/repo-layout.md` for new files, `DESIGN.md` for
  new parts of the page, and `README.md` only for what a newcomer sees (D80; film its pictures again with
  `tools/media`, D81). Write as the docs do: plain, concrete sentences, with colons and semicolons rather than dashes.
- **Tests** go in `tests/`: one script per suite, listed in `run_all.py`'s `SUITES`, taking its address and ports from
  `simrig.py`, printing PASS or FAIL per check and exiting 0 only if all pass. They run against the simulator, the
  twin and independent oracles, never the arm. New checks should fail before the change they test.
- **Code.** `scripts/`, `tools/utra_sim.py` and `brain/stub/` stay standard-library Python 3.8+; heavy dependencies go
  in `brain/requirements.txt` (pinned) or a tool's own environment. Keep `ut_sdk_python` at its pinned commit. Every
  move is planned, collision-checked and supervised, and the brain only proposes (D40, D43); STOP wins (D48); old
  cells, programs and API fields keep working (D119). A collision-model change goes in both `scripts/utra_model.py`
  and `scripts/web/collide.js` (the `js` suite compares them). A move checked in parts passes one
  `limits=model.start_limits(first_pose)` to every `check_path` call (D124).

## Reading big files

Search, tail or read line ranges; don't read these whole:

- `docs/decisions.md` (140 KB): search for `^## D` to find an entry, then read its lines.
- 60 to 140 KB each: `scripts/jog_server.py`, `jog_panel.html`, `runner.py`, `web/featureset.js`, `web/arm_view.js`;
  `tools/utra_sim.py`; `DESIGN.md`; `tests/tending_tests.py`, `spine_tests.py`, `twin_tests.py`, `panel_tests.py`.
- Data and binaries: `data/captures/` (raw `.bin` and `.bin.gz` streams and a 1 MB `.jsonl`; each capture's README
  says what's there), `scripts/model/**/meshes/*.STL`, `docs/media/`, `.impeccable/design.json` (a generated
  design-system snapshot) and the notebook.
- Generated and git-ignored: `tests/logs/` (`<suite>.out` and the simulators' and servers' logs: read the end, or
  search for `FAIL`; suites print a line per check), `cell/logs/`, `scripts/vendor/` (minified three.js),
  `.venv-oracle/`, `brain/.venv/`, `tools/media/.venv/`, `twin-frames/`; and `ut_sdk_python/`, which is Umbratek's.
