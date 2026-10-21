"""Read-only probe of the controller's identity and settings and each joint servo's registers, for the
machine-tending featureset work. Nothing is written and nothing moves. Needs the controller's single command
connection, so run it while the jog panel server is stopped. Writes settings.json to the capture folder.

    python tools/capture/settings_probe.py [--ip 192.168.2.1] [--dir data/captures/<today>]
"""
import argparse
import json
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import arm as armcli  # noqa: E402

_parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
_parser.add_argument("--ip", default="192.168.2.1")
_parser.add_argument("--dir", default=os.path.join(REPO, "data", "captures", time.strftime("%Y-%m-%d")))
_args = _parser.parse_args()
IP = _args.ip
OUT = _args.dir
os.makedirs(OUT, exist_ok=True)
LINE = 92  # the joint servos' RS485 line, through the controller
ARM_GETTERS = ("get_uuid", "get_sw_version", "get_hw_version", "get_motion_mode", "get_motion_enable",
               "get_brake_enable", "get_error_code", "get_servo_msg", "get_joint_maxacc", "get_joint_jerk",
               "get_tcp_maxacc", "get_tcp_jerk", "get_tcp_offset", "get_tcp_load", "get_gravity_dir",
               "get_collis_sens", "get_teach_sens", "get_tcp_target_pos", "get_tcp_actual_pos",
               "get_tgpio_sw_version", "get_tgpio_hw_version", "get_cgpio_sw_version", "get_cgpio_hw_version")
TEXT = (("UUID", 0x01), ("SW_VERSION", 0x02), ("HW_VERSION", 0x03), ("MULTI_VERSION", 0x04))
BYTES = (("MOTION_DIR", 0x11), ("MOTION_ENABLE", 0x21), ("BRAKE_ENABLE", 0x22), ("ERROR_CODE", 0x2F),
         ("POS_SMOOTH_CYC", 0x36), ("VEL_SMOOTH_CYC", 0x47), ("TAU_SMOOTH_CYC", 0x57))
FLOATS = (("MECH_RATIO", 0x05), ("ELEC_RATIO", 0x10), ("CURR_LIMIT", 0x1A), ("TEMP_DRIVER", 0x28),
          ("TEMP_MOTOR", 0x29), ("BUS_VOLT", 0x2A), ("BUS_CURR", 0x2B), ("POS_CURRENT", 0x31),
          ("POS_LIMIT_MAX", 0x32), ("POS_LIMIT_MIN", 0x33), ("POS_LIMIT_DIFF", 0x34), ("POS_PIDP", 0x35),
          ("VEL_CURRENT", 0x41), ("VEL_LIMIT_MAX", 0x42), ("VEL_LIMIT_MIN", 0x43), ("VEL_LIMIT_DIFF", 0x44),
          ("VEL_PIDP", 0x45), ("VEL_PIDI", 0x46), ("TAU_CURRENT", 0x51), ("TAU_LIMIT_MAX", 0x52),
          ("TAU_LIMIT_MIN", 0x53), ("TAU_LIMIT_DIFF", 0x54), ("TAU_PIDP", 0x55), ("TAU_PIDI", 0x56))


def plain(value):
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    return value if isinstance(value, (int, float, str, type(None))) else repr(value)


def text(value):
    try:
        raw = bytes(int(v) & 0xFF for v in value)
        return raw.split(b"\0")[0].decode("ascii", "replace")
    except Exception:
        return plain(value)


a = armcli.open_arm(IP)
result = {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "arm": {}, "servos": {}}
try:
    for name in ARM_GETTERS:
        getter = getattr(a, name, None)
        if getter is None:
            result["arm"][name] = "not in this SDK"
            continue
        try:
            out = getter()
        except Exception as e:  # recorded, not fatal
            out = f"error: {e!r}"
        result["arm"][name] = plain(out)
        print(f"{name}: {result['arm'][name]}")
    for sid in range(1, 7):
        regs, started = {}, time.monotonic()
        for name, reg in TEXT:
            ret, value = a.get_utrc_int8n_now(LINE, sid, reg, 12)
            regs[name] = text(value) if ret == 0 else f"ret {ret}"
        for name, reg in BYTES:
            ret, value = a.get_utrc_int8_now(LINE, sid, reg)
            regs[name] = plain(value) if ret == 0 else f"ret {ret}"
        for name, reg in FLOATS:
            ret, value = a.get_utrc_float_now(LINE, sid, reg)
            regs[name] = plain(value) if ret == 0 else f"ret {ret}"
        regs["readSeconds"] = round(time.monotonic() - started, 2)
        result["servos"][str(sid)] = regs
        print(f"servo {sid}: {json.dumps(regs)}")
        if sid == 1 and all(isinstance(v, str) and v.startswith("ret") for k, v in regs.items() if k != "readSeconds"):
            print("servo 1 doesn't answer; stopping.")
            break
finally:
    with open(os.path.join(OUT, "settings.json"), "w") as f:
        json.dump(result, f, indent=1)
    try:
        a.socket_fp.close()
    except Exception as e:  # the process exits anyway
        print(f"close: {e!r}")
    sys.stdout.flush()
    os._exit(0)  # the SDK's receive thread would keep the process alive
