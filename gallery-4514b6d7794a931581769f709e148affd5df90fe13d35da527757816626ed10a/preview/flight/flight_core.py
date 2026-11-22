"""Fixed-step arcade helicopter simulation, independent of UI and renderer."""
from json import dumps
from math import exp, hypot, tau

STEP = 1 / 120
MAX_ALTITUDE = 36.0
MAX_RADIUS = 64.0
MAX_SPEED = 9.6
KEYS = frozenset(("Space", "KeyF", "KeyW", "KeyA", "KeyS", "KeyD"))
FIELDS = ("x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "power", "mainAngle", "tailAngle")


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def smooth(value, target, rate):
    return target + (value - target) * exp(-rate * STEP)


class Flight:
    def __init__(self):
        self.reset()

    def reset(self):
        for field in FIELDS:
            setattr(self, field, 0.0)
        self.takeoff = False
        self.accumulator = 0.0
        self.held = set()
        self.pulses = {}

    def press(self, code):
        if code not in KEYS:
            return
        self.held.add(code)
        if code == "Space" and self.y < 0.25:
            self.takeoff = True
        if code == "KeyF":
            self.takeoff = False
        self.pulses[code] = 52 if code == "KeyF" else 29

    def release(self, code):
        self.held.discard(code)

    def clear_input(self):
        self.held.clear()
        self.pulses.clear()
        self.takeoff = False

    def pressed(self, code):
        return code in self.held or code in self.pulses

    def step(self):
        climbing, descending = self.pressed("Space"), self.pressed("KeyF")
        active = self.y > .01 or climbing or self.takeoff
        self.power = smooth(self.power, float(active), 2.7 if active else 1.8)
        self.mainAngle = (self.mainAngle + 17 * self.power * STEP) % tau
        self.tailAngle = (self.tailAngle + 66 * self.power * STEP) % tau
        if self.y >= 2.05:
            self.takeoff = False
        lift = 0.0
        if not descending and self.power > .61:
            lift = 3.5 if climbing else 2.4 if self.takeoff else 0.0
        lower = min(2.8, .58 + self.y * 1.05) if descending else 0.0
        self.vy = smooth(self.vy, lift - lower, 5.1)
        self.y = clamp(self.y + self.vy * STEP, 0, MAX_ALTITUDE)
        if self.y in (0, MAX_ALTITUDE):
            self.vy = 0.0
        forward = self.pressed("KeyW") - self.pressed("KeyS")
        right = self.pressed("KeyD") - self.pressed("KeyA")
        speed = MAX_SPEED * clamp((self.y - .19) / .70, 0, 1) / max(hypot(forward, right), 1)
        self.vx = smooth(self.vx, forward * speed, 3.5)
        self.vz = smooth(self.vz, right * speed, 3.5)
        if self.y < .03:
            self.vx = self.vz = 0.0
        self.x += self.vx * STEP
        self.z += self.vz * STEP
        radius = hypot(self.x, self.z)
        if radius > MAX_RADIUS:
            nx, nz = self.x / radius, self.z / radius
            outward = max(self.vx * nx + self.vz * nz, 0)
            self.x, self.z = nx * MAX_RADIUS, nz * MAX_RADIUS
            self.vx -= outward * nx
            self.vz -= outward * nz
        lean = clamp((self.y - .19) / 1.4, 0, 1)
        self.pitch = smooth(self.pitch, -.17 * lean * (forward * .72 + self.vx / MAX_SPEED * .28), 5)
        self.roll = smooth(self.roll, .18 * lean * (right * .72 + self.vz / MAX_SPEED * .28), 5)
        self.pulses = {key: ticks - 1 for key, ticks in self.pulses.items() if ticks > 1}

    def advance(self, elapsed):
        self.accumulator += clamp(elapsed, 0, .15)
        while self.accumulator + 1e-12 >= STEP:
            self.step()
            self.accumulator -= STEP
        return self.state_json()

    def state_json(self):
        return dumps({field: getattr(self, field) for field in FIELDS}, separators=(",", ":"))