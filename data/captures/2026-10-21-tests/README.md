# 2026-10-21, afternoon: the featureset's tests on the real arm

The owner at the arm, 12:09 to 15:27 (Pacific time), running `docs/morning-test-plan.md` with the server from this
branch. Firmware 2021-12-18, nothing on the flange, teach and collision sensitivity 0 at power-on. Until 13:15 over
the controller's Wi-Fi (192.168.2.1); from 13:15 the controller was cabled to the home router (192.168.1.43).

## Files

| File | What | Made by |
| ---- | ---- | ------- |
| `settings.json` | The controller's and the six joint servos' settings and registers, read only, 12:09. | `tools/capture/settings_probe.py` |
| `rest-off.json`, `rest-on.json` | 10 s at rest with the motors off (12:10) and on (12:16): joints, pose, "tau". | `tools/capture/tau_check.py record` |
| `stop-test.json` | MT-2: STOP while J1 turned at -10 °/s, 12:28. | `stop_test.py` (session tool; numbers in the test plan's sheet) |
| `status10hz.bin.gz`, `config.bin.gz` | Ports 30001 and 30003, recorded from 12:09:12 to 15:26:53. | `tools/capture/record.py` |
| `teach_probe.jsonl` | One line per controller write while working out freedrive, 14:12 to 14:45: the write, its reply, the registers before and after, and the mode and state as the registers and the 10 Hz report showed them. | `tools/capture/teach_probe.py` |
| `runs/*.jsonl` | The server's run logs (from `cell/logs/2026-10-21/`): `check` for MT-3 (12:32 to 13:28) and MT-9 (133132), `Taught` for MT-8 (dry 151136, 20% 151829, 50% 152341). | `scripts/runner.py` |

The two `.gz` files are gzip of the recorder's files, byte for byte (`gzip.open(path, "rb").read()` gives them back):
repeated (int64 wall-clock ns, int64 monotonic ns, uint16 frame length, frame), the format of
`tools/capture/capture.py`. SHA-256 of the uncompressed files: status10hz.bin
`da0501706b9ebf6ee05e08a839918cb5c9774734f103eba086d12a97e47fb761`, config.bin
`9a4c1760709c9a038c609f111601a62e1bf223486d54a51366ed73f9ba2c6a58`.

Gaps in the recording: about 3 s every 33 s on Wi-Fi, from 12:09 to 13:15 (none on the cable afterwards); 13:14:57
to 13:15:50, moving to the cable; 14:41:23 to 14:44:00, a power cut (the recorder didn't notice the dead link until
restarted, since fixed).

## Timeline

| Time | What |
| ---- | ---- |
| 12:09 to 12:16 | MT-1: settings, rest with the motors off, then on (Enable shift 0.468 mm). |
| 12:28 | MT-2: STOP from -10 °/s, 0.167° past the stop point. |
| 12:32 to 13:28 | MT-3: the program `check`, dry, stepped at 20%, run at 50%, paused, stopped, and faulted by a keep-out zone. E-stop at 13:10:55 (clearing a stop whose reply Wi-Fi lost) and 13:22:05. |
| 13:31 | MT-9: `check` at 50% with the page closed, DONE. |
| 13:34 to 14:03 | Freedrive from the panel fails twice, with two server versions: the arm stops in mode 0. |
| 14:12 to 14:45 | Freedrive worked out with `teach_probe.py` (the panel closed): sensitivity 0, then 100, STOP in freedrive, 90, 110. |
| 14:26:30 | Wrist 1 turned by hand in freedrive to 358°: error 23 (Mechanical angle limit), motors off. |
| 14:29 to 14:40 | Recovery (decision D39): every "ready" raised error 23 again, freedrive held, a move back was refused; J4's motor alone switched off (error cleared at 14:37:02), its brake released, turned by hand, first the wrong way to 762°, then back to 348° at 14:40:17. |
| 14:41:23 | Power cord kicked: the controller restarts; J3 settles 0.46° onto its brake. |
| 14:45 | Freedrive again: wrist 1 turned by hand back to 80.6°. |
| 15:04 to 15:05 | MT-7 from the panel with the fixed server: enter in 0.3 s, guide, leave in 0.3 s. |
| 15:10 to 15:11 | MT-8: p1, p2 and p3 taught by freedrive into `Taught`. |
| 15:11, 15:18, 15:23 | `Taught` dry, at 20% and at 50%: DONE, every target reached within 0.26 mm. |
