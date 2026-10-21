"""Record the controller's status stream (port 30001) and config stream (port 30003) to a folder until Ctrl+C, or for
--seconds. It only listens: it sends the arm nothing, and runs alongside the jog panel (the controller serves a second
client on 30001; capture 2026-10-21). It reconnects if the link drops.

    python tools/capture/record.py data/captures/<folder> [--ip 192.168.2.1] [--seconds S]

Files, in the same format as tools/capture/capture.py: status10hz.bin and config.bin, each repeated (int64 wall-clock
ns, int64 monotonic ns, uint16 frame length, frame), appended and flushed as frames arrive. Standard library only.
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import threading
import time

STREAMS = ((30001, "status10hz"), (30003, "config"))


def record(ip, port, path, stop, counts, notes):
    while not stop.is_set():
        try:
            with socket.create_connection((ip, port), timeout=3.0) as sock, open(path, "ab") as f:
                sock.settimeout(0.5)
                buf = b""
                while not stop.is_set():
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError(f"the controller closed port {port}")
                    wall, mono = time.time_ns(), time.perf_counter_ns()
                    buf += chunk
                    while len(buf) >= 2:
                        length = struct.unpack_from("<H", buf)[0]
                        if not 20 <= length <= 512:
                            buf = buf[1:]
                            continue
                        if len(buf) < length:
                            break
                        f.write(struct.pack("<qqH", wall, mono, length) + buf[:length])
                        buf = buf[length:]
                        counts[port] += 1
                    f.flush()
        except OSError as e:
            notes.append(f"{time.strftime('%H:%M:%S')} port {port}: {e!r}; retrying in 1 s")
            stop.wait(1.0)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("folder")
    p.add_argument("--ip", default="192.168.2.1")
    p.add_argument("--seconds", type=float, default=None)
    args = p.parse_args(argv)
    os.makedirs(args.folder, exist_ok=True)
    stop, counts, notes = threading.Event(), {port: 0 for port, _ in STREAMS}, []
    threads = [threading.Thread(target=record, daemon=True,
                                args=(args.ip, port, os.path.join(args.folder, name + ".bin"), stop, counts, notes))
               for port, name in STREAMS]
    for t in threads:
        t.start()
    started = time.monotonic()
    print(f"Recording {args.ip} ports 30001 and 30003 to {args.folder} (Ctrl+C to stop)", flush=True)
    try:
        while args.seconds is None or time.monotonic() - started < args.seconds:
            time.sleep(10.0 if args.seconds is None else min(1.0, args.seconds))
            while notes:
                print(notes.pop(0), flush=True)
            print(f"{time.strftime('%H:%M:%S')} status frames {counts[30001]}, config frames {counts[30003]}", flush=True)
    except KeyboardInterrupt:
        pass
    stop.set()
    for t in threads:
        t.join(2.0)
    print(f"Stopped: status frames {counts[30001]}, config frames {counts[30003]}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
