"""Pool simulator for src/auton.py - runs the real auton.run() against a fake boat.

    python tools/sim_auton.py            # 100 random matches per case, prints success rates
    python tools/sim_auton.py --one 7    # one match (seed 7) with a printout every second

The fake boat is a guess (speed, turn rate, radio noise), so use this to catch logic bugs
and compare settings, not to predict the real success rate. Real numbers come from logs/.
"""

import math
import random
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import auton  # noqa: E402

DT = 0.02
MATCHES = 100
CAPTURE_R = 0.10  # Beacon centre this close to the jaw point when the arm closes = held
                  # (set per case through play(capture=...))


class FakeIMU:
    def __init__(self, sim):
        self.sim = sim

    def read(self):
        s = self.sim
        return SimpleNamespace(ok=True, age=5, timestamp=int(s.t * 1000),
                               accel_x=0.0, accel_y=0.0, accel_z=-1.0,
                               gyro_x=0.0, gyro_y=0.0,
                               gyro_z=s.yaw_rate + s.gyro_bias + s.rng.gauss(0, 0.5),
                               mag_x=0, mag_y=0, mag_z=0)


class FakeRadio:
    def __init__(self, sim):
        self.sim = sim
        self.rx = 0
        self.last = SimpleNamespace(ok=False, age=9999, timestamp=0, rssi=None, snr=None,
                                    beacon_id=None, rx_count=0)

    def read(self):
        r = self.last
        age = int((self.sim.t * 1000) - r.timestamp)
        return SimpleNamespace(ok=age < 250, age=age, timestamp=r.timestamp, rssi=r.rssi,
                               snr=r.snr, beacon_id=r.beacon_id, rx_count=self.rx)

    def transmit(self, beacon_id, rssi):
        self.rx += 1
        self.last = SimpleNamespace(timestamp=int(self.sim.t * 1000), rssi=round(rssi),
                                    snr=8.0, beacon_id=beacon_id)


class Part:
    def __init__(self):
        self.duty = 0.0
        self.deg = 90.0

    def set_duty(self, d):
        self.duty = max(-100.0, min(100.0, d))

    def angle(self, d):
        self.deg = max(0.0, min(180.0, d))

    def set_speed(self, _):
        pass


class Sim:
    """Pool frame: x from our wall toward the field, y to the right, heading clockwise."""

    def __init__(self, seed, start_heading, start_y, beacon_xy, rate_hz=5.0, noise_db=3.0,
                 neighbour=True, speed_error=0.10):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.x, self.y, self.h = auton.START_X, start_y, start_heading
        self.speed = self.yaw_rate = 0.0
        self.bx, self.by = beacon_xy
        self.held = False
        # the real boat differs from the settings by up to speed_error
        self.vmax = auton.SPEED_MPS * self.rng.uniform(1 - speed_error, 1 + speed_error)
        self.turn_max = auton.TURN_RATE_DPS * self.rng.uniform(1 - speed_error, 1 + speed_error)
        self.gyro_bias = self.rng.uniform(-1.0, 1.0)
        self.drift = (self.rng.gauss(0, 0.01), self.rng.gauss(0, 0.01))
        self.rate_hz, self.noise_db, self.neighbour = rate_hz, noise_db, neighbour
        self.next_tx = self.rng.uniform(0, 1 / rate_hz)
        self.next_nb = 0.3
        self.imu_sensor, self.radio = FakeIMU(self), FakeRadio(self)
        self.parts = {}
        self.controller = None
        self.steps = 0

    # --- the robot API used by auton.run() ---
    def thruster(self, port):
        return self.parts.setdefault(port, Part())

    servo = thruster

    def imu(self, _):
        return self.imu_sensor

    def radiobeacon(self, _):
        return self.radio

    @property
    def running(self):
        if self.steps:
            self._physics()
        self.steps += 1
        return self.t < 21.0

    def clock(self):
        return self.t

    # --- physics ---
    def _physics(self):
        left = self.parts.get("L", Part()).duty
        right = self.parts.get("R", Part()).duty
        surge, yaw = (left + right) / 2, (left - right) / 2
        v_target = self.vmax * surge / 100 * (1 if surge >= 0 else auton.REVERSE_FACTOR)
        self.speed += (v_target - self.speed) * DT / 0.5
        self.yaw_rate += (self.turn_max * yaw / auton.YAW_MAX - self.yaw_rate) * DT / 0.3
        self.h += self.yaw_rate * DT
        hr = math.radians(self.h)
        nx = self.x + (self.speed * math.cos(hr) + self.drift[0]) * DT
        ny = self.y + (self.speed * math.sin(hr) + self.drift[1]) * DT
        if self._free(nx, ny, hr):
            self.x, self.y = nx, ny
        elif self._free(nx, self.y, hr):  # slide along whatever it hit
            self.x, self.speed = nx, self.speed * 0.5
        elif self._free(self.x, ny, hr):
            self.y, self.speed = ny, self.speed * 0.5
        else:
            self.speed = 0.0
        self._beacon(hr)
        self.t += DT
        if self.t >= self.next_tx:
            self.next_tx += 1 / self.rate_hz * self.rng.uniform(0.8, 1.2)
            if self.rng.random() > 0.05:  # 5 % of messages lost
                self.radio.transmit(17, self._rssi())
        if self.neighbour and self.t >= self.next_nb:  # another pool's beacon, far away
            self.next_nb += 0.7
            self.radio.transmit(23, -88 + self.rng.gauss(0, 3))

    def _bow(self, x, y, hr):
        return x + auton.BOW_OFFSET * math.cos(hr), y + auton.BOW_OFFSET * math.sin(hr)

    def _free(self, x, y, hr):
        for px, py in ((x, y), self._bow(x, y, hr),
                       (x - auton.STERN_OFFSET * math.cos(hr), y - auton.STERN_OFFSET * math.sin(hr))):
            if not (0.05 < px < auton.POOL_LENGTH - 0.05 and abs(py) < auton.POOL_HALF_WIDTH - 0.05):
                return False
            if (abs(px - auton.CENTER_X) < auton.ANCHORAGE_HALF_LENGTH
                    and abs(py) < auton.ANCHORAGE_HALF_WIDTH):
                return False
        return True

    def _beacon(self, hr):
        jaw_x, jaw_y = self._bow(self.x, self.y, hr)
        jaw_x -= 0.08 * math.cos(hr)
        jaw_y -= 0.08 * math.sin(hr)
        arm = self.parts.get("A", Part()).deg
        closed = abs(arm - auton.ARM_CLOSED_DEG) < 20
        d = math.dist((jaw_x, jaw_y), (self.bx, self.by))
        if closed and not self.held and d < CAPTURE_R:
            self.held = True
        if not closed:
            self.held = False
        if self.held:
            self.bx, self.by = jaw_x, jaw_y
            return
        # the hull pushes the beacon away; an open arm lets it into the mouth up to the jaw point
        reach = math.dist((self.x, self.y), (jaw_x, jaw_y))
        hull = [(self.x, self.y, 0.12)] if not closed else [(self.x, self.y, 0.12), (jaw_x, jaw_y, 0.08)]
        if not closed and d > CAPTURE_R:
            # beside the mouth: the arm's outside pushes it
            side = abs((self.bx - self.x) * -math.sin(hr) + (self.by - self.y) * math.cos(hr))
            along = (self.bx - self.x) * math.cos(hr) + (self.by - self.y) * math.sin(hr)
            if side > CAPTURE_R and 0 < along < reach + 0.075:
                hull.append((jaw_x, jaw_y, 0.08))
        for px, py, r in hull:
            gap = math.dist((px, py), (self.bx, self.by))
            if 1e-6 < gap < 0.075 + r:
                push = (0.075 + r - gap) / gap
                self.bx += (self.bx - px) * push
                self.by += (self.by - py) * push

    def _rssi(self):
        hr = math.radians(self.h)
        ax, ay = self.x - 0.15 * math.cos(hr), self.y - 0.15 * math.sin(hr)  # antenna near stern
        d = max(0.1, math.dist((ax, ay), (self.bx, self.by)))
        return -40 - 25 * math.log10(d / 0.35) + self.rng.gauss(0, self.noise_db)

    def scored(self):
        return self.bx - 0.075 <= 0.40


def make(seed, heading_error=5.0, corner_start=True, **kw):
    rng = random.Random(seed * 7919)
    side = rng.choice((-1, 1))
    bow = rng.choice((90.0, -90.0))
    heading = bow + rng.uniform(-heading_error, heading_error)
    if corner_start:  # at the end of our wall, bow pointing along it (recommended)
        start_y = -math.copysign(0.45, bow) + rng.uniform(-0.04, 0.02)
    else:
        start_y = rng.uniform(-0.5, 0.5)
    beacon = (auton.CENTER_X, side * rng.uniform(0.22, 0.70))
    return Sim(seed, heading, start_y, beacon, **kw), heading, start_y


def play(seed, verbose=False, capture=0.10, **kw):
    global CAPTURE_R
    CAPTURE_R = capture
    sim, heading, start_y = make(seed, **kw)
    # the team sets the start pose by eye: a few degrees / cm off
    auton.START_HEADING = round(heading / 90) * 90
    auton.START_Y = round(start_y, 1)
    auton._still_imu_readings[:] = [sim.imu_sensor.read() for _ in range(20)]
    ports = dict(imu_port="I", radio_port="B", arm_port="A", left_port="L", right_port="R")
    search, radio = auton.run(sim, **ports, balance=lambda a, b: (a, b), ramp_step=5.0,
                              clock=sim.clock, save_logs=False)
    if verbose:
        print(f"seed {seed}: start heading {heading:.0f}, y {start_y:.2f}; "
              f"beacon at y {make(seed)[0].by:.2f}")
        print(radio.summary(20))
        print(f"end: state {search.state}, held {sim.held}, beacon x {sim.bx:.2f} "
              f"-> {'SCORED' if sim.scored() else 'not scored'}")
    return sim.scored(), search.state, radio


if __name__ == "__main__":
    if "--one" in sys.argv:
        play(int(sys.argv[sys.argv.index("--one") + 1]), verbose=True)
        sys.exit()
    for label, kw in (("calibrated, corner start, 5 msg/s, 3 dB", {}),
                      ("calibrated, corner start, 2 msg/s", {"rate_hz": 2.0}),
                      ("calibrated, corner start, 5 dB noise", {"noise_db": 5.0}),
                      ("calibrated, start anywhere on the wall", {"corner_start": False}),
                      ("NOT calibrated (speed +/-25 %, heading +/-10 deg)",
                       {"speed_error": 0.25, "heading_error": 10.0}),
                      ("calibrated, WIDE mouth (grabs within 0.20 m instead of 0.10)",
                       {"capture": 0.20})):
        results = [play(s, **kw) for s in range(MATCHES)]
        wins = sum(r[0] for r in results)
        states = {}
        for _, state, _ in results:
            states[state] = states.get(state, 0) + 1
        locked_wrong = sum(1 for r in results if r[2].locked_id != 17)
        print(f"{label}: scored {wins}/{MATCHES} ({100 * wins / MATCHES:.0f} %), end states {states}, "
              f"wrong beacon locked {locked_wrong}")
