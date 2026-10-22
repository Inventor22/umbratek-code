# brain/

The brain: a separate process that proposes motion to the spine (`scripts/`), which checks everything it proposes
before anything moves. The interface is `docs/brain-interface.md`.

## Stub brain (`brain/stub/`)

`stub_brain.py` is a stand-in brain that speaks the interface, so planned and learned steps run end to end before the
real brain exists. It's standard-library Python (3.8 or later) and needs no environment of its own: it imports the
spine's protocol, arm model and planner from `scripts/`.

```
python brain/stub/stub_brain.py                 # serves http://127.0.0.1:8770, where the jog server looks by default
python tools/utra_sim.py                        # the simulated arm, in another window
python scripts/jog_server.py 127.0.0.1          # then open http://127.0.0.1:8765
```

It runs the same in WSL2 (`python3 brain/stub/stub_brain.py` from the repo under `/mnt/c/...`): WSL2 forwards Windows'
`localhost` to it, so the jog server on Windows reaches it at `http://127.0.0.1:8770` unchanged.

- **Planned moves:** it plans a path from where the arm is to the goal the spine sets, against the spine's model of the
  arm and its cart plus the tool and keep-out zones the spine sends: straight there if that's clear, else up and over
  (through poses with the tool point 60 to 300 mm above the start and the goal), else through random via points. It
  gives up, saying so, when 60% of the spine's deadline has gone.
- **One learned skill, `seek`:** a scripted stand-in for a policy. It spirals the tool point out from where the step
  began, a few millimetres a burst, keeping its orientation, until an input goes high. Params: `input` (default
  `tool.1`), `stepMm` (8), `pitchMm` (10 per turn), `radiusMm` (40) and `doneAfter` (a number of bursts, to say it's
  done regardless).
- **`--ignore-zones`** plays a buggy brain whose world model has lost the cell's keep-out zones: its paths go straight
  through them, and the spine refuses them. `--port`, `--host` (loopback by default; it warns otherwise, as the
  interface has no authentication), `--seed` (for the random via points) and `--quiet` (no line per request).

`tests/spine_tests.py` (the `spine` suite of `tests/run_all.py`) runs it as a separate process against the simulator,
and plays misbehaving brains with its answers in-process.
