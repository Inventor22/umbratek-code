"""The HTTP server class every HTTP server here is built on: the jog panel's, the brains' (the stub's and the
planner's), the twin's and perception's. All of them listen on this PC's loopback addresses only.

It's http.server's ThreadingHTTPServer, without the reverse DNS lookup that class makes when it binds.
HTTPServer.server_bind asks for its address's name (socket.getfqdn), only to keep it as server_name, which none of
these servers uses. For 127.0.0.1 the hosts file answers at once. For the other loopback addresses the test suites use
it's a DNS query, and where DNS doesn't answer one (in WSL2 with a VPN on, for one) the server waits about 10 s for it
before it listens. Standard-library Python, like the rest of the spine.
"""
import socketserver
from http.server import ThreadingHTTPServer


class Server(ThreadingHTTPServer):
    """http.server.ThreadingHTTPServer, without the reverse DNS lookup of its address when it binds."""

    def server_bind(self):
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]
