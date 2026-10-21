"""Motion profiles shared by the simulator and its fit to the capture: rest-to-rest point-to-point moves.

double_s() is the jerk-limited ("double S") profile of Biagiotti and Melchiorri, Trajectory Planning for Automatic
Machines and Robots (Springer, 2008), section 3.4, for a move that starts and ends at rest. trapezoid() is the same
without the jerk limit. Each returns a Profile: its duration and position(t), the distance covered at time t.
Standard library only.
"""
import math


class Profile:
    """A rest-to-rest move: jerk phases of tj, an acceleration phase of ta (tj at each end), tv at speed v_lim, then
    the acceleration phase mirrored. tj is 0 for a trapezoid."""

    def __init__(self, distance, tj, ta, tv, j, a_lim, v_lim):
        self.distance = distance
        self.tj, self.ta, self.tv = tj, ta, tv
        self.j, self.a_lim, self.v_lim = j, a_lim, v_lim
        self.duration = 2 * ta + tv

    def _ramp(self, t):
        """Distance covered t into the acceleration phase (0 <= t <= ta)."""
        j, a, tj, ta, v = self.j, self.a_lim, self.tj, self.ta, self.v_lim
        if tj <= 0.0:
            return 0.5 * a * t * t
        if t <= tj:  # jerk +j
            return j * t ** 3 / 6.0
        s1, v1 = j * tj ** 3 / 6.0, j * tj * tj / 2.0
        if t <= ta - tj:  # constant acceleration a
            u = t - tj
            return s1 + v1 * u + 0.5 * a * u * u
        mid = ta - 2 * tj  # jerk -j: speed v - j (ta - t)^2 / 2
        s2 = s1 + v1 * mid + 0.5 * a * mid * mid
        u = t - (ta - tj)
        return s2 + v * u - j * (tj ** 3 - (tj - u) ** 3) / 6.0

    def position(self, t):
        if t <= 0.0:
            return 0.0
        if t >= self.duration:
            return self.distance
        if t <= self.ta:
            return self._ramp(t)
        if t <= self.ta + self.tv:
            return self._ramp(self.ta) + self.v_lim * (t - self.ta)
        return self.distance - self._ramp(self.duration - t)

    def speed(self, t, h=1e-5):
        return (self.position(t + h) - self.position(t - h)) / (2 * h)


def trapezoid(distance, v, a):
    """Rest-to-rest move over |distance| at up to speed v and acceleration a, without a jerk limit."""
    distance = abs(distance)
    if distance <= 0.0:
        return Profile(0.0, 0.0, 0.0, 0.0, 0.0, a, 0.0)
    if distance >= v * v / a:
        ta = v / a
        return Profile(distance, 0.0, ta, distance / v - ta, 0.0, a, v)
    ta = math.sqrt(distance / a)
    return Profile(distance, 0.0, ta, 0.0, 0.0, a, a * ta)


def double_s(distance, v, a, j):
    """Rest-to-rest move over |distance| limited to speed v, acceleration a and jerk j (Biagiotti, section 3.4)."""
    distance = abs(distance)
    if not j or j <= 0.0 or not math.isfinite(j):
        return trapezoid(distance, v, a)
    if distance <= 0.0:
        return Profile(0.0, 0.0, 0.0, 0.0, j, a, 0.0)
    if v * j < a * a:  # v is reached before a is (eq. 3.19)
        tj = math.sqrt(v / j)
        ta, a_reached = 2 * tj, j * tj
    else:
        tj = a / j
        ta, a_reached = tj + v / a, a
    tv = distance / v - ta
    if tv >= 0.0:
        return Profile(distance, tj, ta, tv, j, a_reached, v)
    tj = a / j  # v isn't reached: try with the full acceleration (eq. 3.26a, rest to rest)
    ta = (a * a / j + math.sqrt(a ** 4 / j ** 2 + 4.0 * a * distance)) / (2.0 * a)
    if ta >= 2 * tj:
        return Profile(distance, tj, ta, 0.0, j, a, a * (ta - tj))
    tj = (distance / (2.0 * j)) ** (1.0 / 3.0)  # nor a: jerk phases only
    return Profile(distance, tj, 2 * tj, 0.0, j, j * tj, j * tj * tj)
