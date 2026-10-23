# Connecting to the arm

The arm is driven by an **NTRO series controller**, the box with the power button and E-stop.

1. **Wire it up.** Arm cable to *Robot Connector*. External E-stop switch (or the shorting plug from the box) to *Expansion E-STOP*. The two Wi-Fi antennas go on the top-left and top-right posts; the middle post is for the optional wireless E-stop. Power cable to *Power Supply Socket*.
2. **Boot.** Press the power button once and wait about 25 seconds. The indicator breathes quickly (1 s cycle) while booting and slowly (3 s cycle) when ready. A fast 0.3 s blink means an error: check that the E-stop isn't pressed and the cables are seated.
3. **Network.** Use either:
   - **Wi-Fi:** join the controller's `NTRO_xxx` network. The default password is `12345678`. This repo's controller broadcasts `NTRO_43`, sits at `192.168.2.1` and gives the PC a `192.168.2.x` address. It offers no internet gateway, so the PC keeps its normal connection for internet traffic.
   - **Ethernet:** run a cable from the controller's RJ45 port to the PC, with the PC's adapter set to obtain an IP address automatically.
   - **Your router (best for long sessions):** cable the controller's RJ45 port to your router and use its address there (`192.168.1.43` for this repo's controller). On 2026-10-21 its Wi-Fi dropped the status stream for about 3 s every 33 s, enough to time out commands and once a STOP's reply; on the cable the stream never missed a frame. The catch: on your home network the controller's command port and Umbratek Studio are open to every device, with no password.
4. **Umbratek Studio (optional).** Browse to the controller's address (`http://192.168.2.1` over Wi-Fi, or the address printed on the label on its interface panel) and click *Connect* for Umbratek's web UI.

## Using Umbratek Studio

Any browser on the controller's network works, including a phone or tablet joined to `NTRO_43`. Clicking *connect* only opens a session; it doesn't enable or move the arm. Then:

- **Live Control** shows the tool position and joint angles, with jog panels for joints (left) and for the tool in X/Y/Z and roll/pitch/yaw (right).
  - **Set the speed first.** The *Vel* boxes default to 100 °/s and 100 mm/s; something like 10 °/s and 20 mm/s is easier to watch and stop.
  - **A short click on −/+** moves one step: 1° per joint click, 10 mm or 5° per tool click. The gear icons change the step.
  - **Holding a −/+ button keeps the joint moving** until you let go.
  - **Dragging a slider or typing a joint value** sends the arm straight to that value at the panel speed.
  - **Zero Position** stands the arm straight up, about 1.1 m tall, and **Init Position** goes to a pose saved in Setting. Clear the arm's reach before using either.
  - **Clear Error** does the same as `scripts/arm.py reset`. **Joint Tech** is a hand-guiding mode for dragging the arm by hand.
- The red **E-STOP** button in the top bar stops motion in software and clears queued commands. The hardware E-stop is the one that cuts the arm's power.
- **Blocky** builds programs from blocks (moves, speeds, gripper, GPIO, loops). **Setting** holds motion parameters (TCP payload and offset, mount direction, speeds), GPIO, versions and servo temperatures under *Info*, and network, reboot and shutdown under *System*. Leave the calibration and upgrade buttons under *Info* alone.
- Don't move the arm from Studio and from Python at the same time.

Once this PC reaches the controller, [the jog panel](jog-panel.md) and [`scripts/arm.py`](python.md) can drive the arm; [troubleshooting](troubleshooting.md) helps when they can't.
