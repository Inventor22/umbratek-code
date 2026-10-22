"""The spine's end of the link to the brain: asks it one question at a time over HTTP on this PC (brain_protocol.py).

The brain is a server on a loopback address, so it may run on Windows or in WSL2 (whose default networking forwards
Windows' localhost to servers in WSL2, docs/decisions.md D41). The link has no authentication, so the spine talks only
to loopback addresses, and treats every reply as untrusted: brain_protocol checks its shape and staleness, and the
safety shell (safety.py) everything it proposes. Each question has a deadline, and waiting can be abandoned at once
(for STOP or Pause); a reply that comes later is dropped, since the next question has a new id. Standard library only.
"""
from __future__ import annotations

import http.client
import ipaddress
import itertools
import socket
import threading
import time
import urllib.parse

import brain_protocol as bp

DEFAULT_URL = "http://127.0.0.1:8770"
CONNECT_TIMEOUT_S = 1.0  # on loopback a listening brain takes the connection at once; Windows takes ~2 s to refuse
HELLO_TIMEOUT_S = 2.0
PLAN_TIMEOUT_S = 10.0  # a planner may take a while on its first call (cuRobo compiles its kernels then)
ACT_TIMEOUT_S = 2.0  # a learned skill acts at 10 Hz or slower, so it must answer quickly
END_TIMEOUT_S = 1.0
HELLO_EVERY_S = 5.0  # how often the server asks who the brain is, for the panel
POLL_S = 0.05
SPINE = "umbratek-code jog_server"


class BrainError(Exception):
    """The brain couldn't be asked: not running, not answering in time, or answering with an HTTP error."""


class Abandoned(Exception):
    """The spine stopped waiting for the brain (STOP or Pause); a reply that comes later is dropped."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def loopback(url):
    """(host, port) from a brain's address, which must be plain http to a loopback address (127.x.x.x, ::1 or
    localhost) with a port: the brain must be on this PC. ValueError says why otherwise."""
    try:
        parts = urllib.parse.urlsplit(url)
        port = parts.port
    except ValueError:
        parts, port = None, None
    if parts is None or parts.scheme != "http" or parts.username or parts.password or parts.path not in ("", "/") \
            or parts.query or parts.fragment or not parts.hostname or port is None:
        raise ValueError(f"the brain's address should look like {DEFAULT_URL}, not {url!r}")
    host = parts.hostname
    if host != "localhost":
        try:
            loop = ipaddress.ip_address(host).is_loopback
        except ValueError:
            loop = False
        if not loop:
            raise ValueError(f"the brain must be on this PC, at a loopback address such as 127.0.0.1, not {host}")
    return host, port


class BrainLink:
    """Asks the brain at `url` (a loopback address) one question at a time, and remembers its last hello."""

    def __init__(self, url=DEFAULT_URL, plan_timeout=PLAN_TIMEOUT_S):
        self.url = url
        self.host, self.port = loopback(url)
        self.timeouts = {"hello": HELLO_TIMEOUT_S, "plan": float(plan_timeout), "act": ACT_TIMEOUT_S, "end": END_TIMEOUT_S}
        self._ids = itertools.count(1)
        self._tag = f"{int(time.time()) % 1000000:06d}"  # so ids stay unique across server restarts
        self.known = None  # the last hello's outcome, for the panel

    @property
    def plan_timeout(self):
        return self.timeouts["plan"]

    def ask(self, kind, fields, should_abort=None):
        """Ask the brain one question. Returns (the request, the checked reply). Raises BrainError (no usable answer),
        bp.ProtocolError (an answer that breaks the protocol, said as "the brain's reply ...") or Abandoned (when
        should_abort(), polled while waiting, gives a reason)."""
        request = bp.message(kind, f"{kind}-{self._tag}-{next(self._ids)}", **fields)
        body = bp.encode(request)
        timeout = self.timeouts[kind]
        box = {}
        worker = threading.Thread(target=self._exchange, args=(kind, body, timeout, box), daemon=True)
        started = time.monotonic()
        worker.start()
        while True:
            worker.join(POLL_S)
            if not worker.is_alive():
                break
            why = should_abort() if should_abort is not None else None
            if why:
                raise Abandoned(why)
            if time.monotonic() - started > timeout:
                raise BrainError(f"the brain at {self.url} didn't answer within {timeout:g} s")
        if "error" in box:
            raise box["error"]
        try:
            return request, bp.check_reply(request, bp.decode(box["data"]))
        except bp.ProtocolError as e:
            raise bp.ProtocolError(f"the brain's reply {e}")

    def _exchange(self, kind, body, timeout, box):
        conn = http.client.HTTPConnection(self.host, self.port, timeout=CONNECT_TIMEOUT_S)
        try:
            conn.connect()
            conn.sock.settimeout(timeout + 1.0)  # ask() keeps the deadline itself
            conn.request("POST", bp.PATHS[kind], body, {"Content-Type": "application/json"})
            response = conn.getresponse()
            data = response.read(bp.MAX_BYTES + 1)
            if response.status != 200:
                box["error"] = BrainError(f"the brain at {self.url} answered HTTP {response.status} to {kind}")
            else:
                box["data"] = data
        except (OSError, http.client.HTTPException) as e:
            if conn.sock is None and isinstance(e, ConnectionRefusedError):
                why = "nothing is listening there"
            elif conn.sock is None and isinstance(e, (TimeoutError, socket.timeout)):
                why = f"nothing took the connection within {CONNECT_TIMEOUT_S:g} s"
            else:
                why = str(e) or type(e).__name__
            box["error"] = BrainError(f"the brain at {self.url} isn't answering ({why})")
        finally:
            conn.close()

    def hello(self):
        """Ask who the brain is and what it can do. Returns, and remembers for the panel, {"ok", "brain", "skills",
        "planner", "error", "at"}."""
        try:
            _, answer = self.ask("hello", {"spine": SPINE, "versions": [bp.VERSION]})
            known = dict(answer, ok=True, error="")
        except (BrainError, bp.ProtocolError) as e:
            known = {"ok": False, "brain": "", "skills": {}, "planner": False, "error": str(e)}
        known["at"] = time.monotonic()
        self.known = known
        return known

    def end(self, fields):
        """Tell the brain how a learned skill's episode ended, without waiting; it's a courtesy, so failures pass."""
        def tell():
            try:
                self.ask("end", fields)
            except (BrainError, bp.ProtocolError):
                pass
        threading.Thread(target=tell, daemon=True).start()

    def status(self):
        """The last hello, for the panel's status."""
        known = self.known
        out = {"url": self.url, "ok": None if known is None else known["ok"]}
        if known is not None:
            out.update(brain=known["brain"], skills=known["skills"], planner=known["planner"], error=known["error"],
                       ageS=round(time.monotonic() - known["at"], 1))
        return out
