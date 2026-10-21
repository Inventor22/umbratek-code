"""Read or set the controller's collision sensitivity (0 = off, 1 to 5 = least to most sensitive) and teach
(freedrive) sensitivity (1 to 5), for the morning test plan (docs/morning-test-plan.md). Nothing moves. It needs the
controller's single command connection, so quit the jog panel first; the jog panel sets the teach sensitivity itself
each time it starts freedrive.

    python tools/capture/sensitivity.py [--ip 192.168.2.1]          # read both
    python tools/capture/sensitivity.py --collision 3               # set, then read back
    python tools/capture/sensitivity.py --teach 2

Whether the controller keeps a setting through a power cycle is one of the things the test plan finds out.
"""
from __future__ import annotations

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import arm as armcli  # noqa: E402


def level(text, low):
    try:
        value = int(text)
    except ValueError:
        value = -1
    if not low <= value <= 5:
        raise argparse.ArgumentTypeError(f"expected a whole number from {low} to 5")
    return value


def read(a):
    ret_c, collision = a.get_collis_sens()
    ret_t, teach = a.get_teach_sens()
    if ret_c not in armcli.OK_CODES or ret_t not in armcli.OK_CODES:
        raise armcli.ArmError(f"The controller didn't answer the reads (codes {ret_c}, {ret_t}).")
    return int(collision), int(teach)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--ip", default="192.168.2.1")
    p.add_argument("--collision", type=lambda t: level(t, 0), help="0 (off) to 5")
    p.add_argument("--teach", type=lambda t: level(t, 1), help="1 to 5")
    args = p.parse_args(argv)
    try:
        a = armcli.open_arm(args.ip)
    except armcli.ArmError as e:
        print(e, file=sys.stderr)
        return 1
    try:
        before = read(a)
        print(f"Before: collision sensitivity {before[0]}, teach sensitivity {before[1]}")
        for value, setter, name in ((args.collision, a.set_collis_sens, "collision"), (args.teach, a.set_teach_sens, "teach")):
            if value is None:
                continue
            ret = setter(value)
            if ret not in armcli.OK_CODES:
                print(f"The controller refused {name} sensitivity {value} (code {ret}).", file=sys.stderr)
                return 1
        after = read(a)
        if args.collision is not None or args.teach is not None:
            print(f"After:  collision sensitivity {after[0]}, teach sensitivity {after[1]}")
            wanted = (args.collision if args.collision is not None else before[0],
                      args.teach if args.teach is not None else before[1])
            if after != wanted:
                print(f"The read-back {after} isn't what was set {wanted}.", file=sys.stderr)
                return 1
        return 0
    except armcli.ArmError as e:
        print(e, file=sys.stderr)
        return 1
    finally:
        try:
            a.socket_fp.close()
        except Exception:
            pass


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # the SDK's receive thread would keep the process alive
