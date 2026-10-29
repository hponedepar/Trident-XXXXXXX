"""Autonomous Period (first 20 s): find the Beacon by radio, grab it and bring it home.

Scoring: 10 points if the Beacon body is partly inside our Starting Area (the 0.4 m strip
along our wall) when the Autonomous Period is scored (Game Manual <SC2>, <SC6>).

What the rules tell us (Game Manual v1.0, Appendix A6):
  - Pool inside is about 2.58 m x 1.55 m. Our wall is x = 0, the Centerline is at x = 1.29.
  - The Beacon is put at a random spot on the Centerline.
  - The Anchorage (about 0.85 x 0.25 m, 6 posts) sits in the middle, across the Centerline.
  - The robot must start inside the 0.4 m Starting Area. A robot longer than 0.4 m starts
    sideways along the wall, so set START_HEADING (and START_Y) before every match.

Sensors:
  - Radio Receiver: signal strength only (rssi in dBm, closer to 0 = stronger). No direction.
    read() only holds the LATEST message, so BeaconListener reads it every loop and uses
    rx_count to catch every new message and count any that were missed between reads.
  - IMU gyro: heading. Position is estimated from the heading and the commanded speed.

How it finds the Beacon:
  One rssi reading is too noisy to steer by (a few dB of noise, a few messages per second).
  So every message is stored together with where the boat was when it arrived, and all of
  them are fitted to "signal drops with distance" for every possible Beacon spot on the
  Centerline. The spot that explains the messages best is the Beacon estimate.
    OUT      curve out from our wall, bow first, onto the sweep line (x = SWEEP_X, in front of
             the Anchorage)
    SWEEP    follow the sweep line the way the bow pointed at the start, collecting messages
             from many angles, until the estimate is well behind the boat or the far end
    APPROACH line up with the estimate along the sweep line (backing up if we passed it)
    FINAL    go straight in toward the estimate (still refitting)
    CLOSE    close the arm
    RETURN   drive to our wall, bow first if there is time (Beacon at the bow), else reverse
    PIN      keep the Beacon pushed against our wall until the period ends

Every message and a 50 Hz trace are saved in logs/ after each run. Numbers marked MEASURE
are guesses until a pool test (see the checklist at the bottom). tools/sim_auton.py runs
this file against a simulated pool.
"""

import csv
import math
import time
from pathlib import Path

# ---------------- start pose - SET BEFORE EVERY MATCH ----------------
# Bow direction at the start, degrees clockwise from "facing straight at the field"
# (seen from behind our wall). 0 = facing the field, +90 = bow points right along the wall,
# -90 = bow points left along the wall. The sweep goes the way the bow points, so the best
# start is at one END of our wall with the bow pointing along the wall toward the other end
# (e.g. START_Y = -0.45 with START_HEADING = +90: stern almost touching the left wall).
START_HEADING = 90.0
START_X = 0.20  # m from our wall to the robot centre
START_Y = 0.0   # m sideways from the middle of our wall (+ = right, wall ends are +/-0.78)

# ---------------- radio ----------------
BEACON_ID = None     # None = use the strongest beacon heard in the first LOCK_S seconds
LOCK_S = 0.5
PATH_LOSS = (1.5, 2.0, 2.5, 3.0, 3.5)  # signal-drop steepness values tried in the fit
CLOSE_RSSI = -38.0   # dBm with the Beacon inside the open arm - MEASURE (ends FINAL early)
ANTENNA_OFFSET = 0.0  # m from the robot centre to the radio module, + toward the bow

# ---------------- boat motion - MEASURE ----------------
SPEED_MPS = 0.35        # straight-line speed at full power
REVERSE_FACTOR = 0.85   # reverse speed / forward speed
SPEED_TAU = 0.5         # s to pick up speed
TURN_RATE_DPS = 90.0    # spin-on-the-spot rate at YAW_MAX
YAW_SIGN = None         # None = automatic from the IMU's down axis; +1 / -1 to force it

# ---------------- search tuning ----------------
CRUISE = 100.0
SWEEP_X = 0.55       # sweep line (robot centre), clear of the Anchorage
SWEEP_HALF_WIDTH = 0.50  # sweep ends at y = +/-0.50 (robot centre)
MIN_SWEEP = 0.45     # m to sweep before trusting the estimate
PASSED_BY = 0.25     # stop sweeping once the estimate is this far behind the boat
SWEEP_DEADLINE = 10.0  # s, stop sweeping by then whatever the estimate looks like
FIT_EVERY = 0.4      # s between fits
MAX_ESTIMATE_STEP = 0.08  # m the estimate may move per fit once the boat is heading for it
KP_HEAD = 2.5        # turn power per degree of heading error
YAW_MAX = 70.0
SPIN_DEG = 60.0      # turn on the spot when the heading is off by more than this
ARC_POWER = 55.0     # forward power while turning away from our wall (no room to spin there)
JAW_OFFSET = 0.18    # m from the robot centre to where the Beacon sits in the arm
LINED_UP = 0.05      # m sideways from the estimate that counts as lined up
APPROACH_MAX_S = 4.0
LINE_GAIN = 150.0    # degrees of heading correction per m off the sweep line
FINAL_POWER = 60.0
FINAL_MAX_S = 5.0
FINAL_WEAVE_DEG = 30.0
FINAL_WEAVE_S = 0.6
CLOSE_S = 0.4        # s for the arm to close before moving
PIN_POWER = 35.0     # gentle push against our wall at the end
RETURN_MARGIN_S = 0.5

# ---------------- arm - MEASURE ----------------
# In the driver code the arm sits at 0 and goes to 180 while LEFT_TRIGGER is held.
# Set which angle holds the Beacon. Effectors go idle when a period ends (an idle servo sits
# at 90), so the boat also pushes the Beacon against our wall in case the arm lets go.
ARM_OPEN_DEG = 180.0
ARM_CLOSED_DEG = 0.0
ARM_SPEED = 360.0

# ---------------- pool (Game Manual Appendix A6) ----------------
T_AUTON = 20.0
POOL_LENGTH = 2.576
POOL_HALF_WIDTH = 0.777
CENTER_X = POOL_LENGTH / 2
ANCHORAGE_HALF_LENGTH = 0.43
ANCHORAGE_HALF_WIDTH = 0.13
BOW_OFFSET = 0.24    # robot centre to bow
STERN_OFFSET = 0.24

LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
BEACON_Y_GRID = [i / 50.0 for i in range(-37, 38)]  # possible Beacon spots, 2 cm apart


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value, low, high):
    return max(low, min(high, value))


def bearing(from_x, from_y, to_x, to_y):
    return math.degrees(math.atan2(to_y - from_y, to_x - from_x))


def fit_beacon_y(samples):
    """Best Beacon y on the Centerline for [(x, y, rssi)], or None with too few messages.

    Model: rssi = A - 10 * n * log10(distance). A (strength at 1 m) is solved exactly for
    each try; n is tried from PATH_LOSS; y is tried every 2 cm.
    """
    if len(samples) < 6:
        return None
    count = len(samples)
    rssi = [s[2] for s in samples]
    mean_r = sum(rssi) / count
    best_cost, best_y = None, None
    for beacon_y in BEACON_Y_GRID:
        logs = [math.log10(max(0.05, math.hypot(CENTER_X - x, beacon_y - y))) for x, y, _ in samples]
        mean_l = sum(logs) / count
        for n in PATH_LOSS:
            a = mean_r + 10 * n * mean_l
            cost = sum((r - a + 10 * n * l) ** 2 for r, l in zip(rssi, logs))
            if best_cost is None or cost < best_cost:
                best_cost, best_y = cost, beacon_y
    return best_y


class BeaconListener:
    """Collects every Beacon message. read() only keeps the latest one, so read it every loop."""

    def __init__(self, radio, beacon_id=BEACON_ID):
        self.radio = radio
        self.locked_id = beacon_id
        self.last_rx = None
        self.missed = 0
        self.log = []       # every message: (t, rx_count, beacon_id, rssi, snr, age_ms, missed)
        self._waiting = []  # messages heard before choosing the beacon

    def poll(self, t):
        """New messages from our beacon since the last call, as [(t, rssi)]."""
        reading = self.radio.read()
        rx = getattr(reading, "rx_count", None)
        if rx is None or rx == self.last_rx:
            return []
        missed = 0 if self.last_rx is None else max(0, rx - self.last_rx - 1)
        self.last_rx = rx
        self.missed += missed
        rssi = getattr(reading, "rssi", None)
        age = getattr(reading, "age", 0) or 0
        if rssi is None or age > 250:
            return []
        beacon = getattr(reading, "beacon_id", None)
        heard = t - age / 1000.0
        self.log.append((round(heard, 3), rx, beacon, rssi, getattr(reading, "snr", None), age, missed))

        if self.locked_id is None:
            self._waiting.append((heard, beacon, rssi))
            if t < LOCK_S:
                return []
            self.locked_id = self._strongest()
            ours = [(h, r) for h, b, r in self._waiting if b == self.locked_id]
            self._waiting = []
            return ours
        return [(heard, rssi)] if beacon == self.locked_id else []

    def _strongest(self):
        by_beacon = {}
        for _, beacon, rssi in self._waiting:
            by_beacon.setdefault(beacon, []).append(rssi)
        return max(by_beacon, key=lambda b: sorted(by_beacon[b])[len(by_beacon[b]) // 2])

    def summary(self, seconds):
        counts = {}
        for row in self.log:
            counts[row[2]] = counts.get(row[2], 0) + 1
        rate = len(self.log) / seconds if seconds > 0 else 0.0
        return (f"RADIO: {len(self.log)} messages ({rate:.1f} per s), missed {self.missed}, "
                f"per beacon_id {counts}, following {self.locked_id}")


class Heading:
    """Heading from the IMU gyro in degrees, clockwise = +, 0 = facing the field."""

    def __init__(self, imu, start_heading):
        self.imu = imu
        self.heading = start_heading
        self.axis = None
        self.sign = YAW_SIGN
        self.bias = 0.0
        self._last_timestamp = None

    def calibrate(self, readings):
        """Readings taken while the boat is still: finds the vertical axis and the gyro bias."""
        good = [r for r in readings if r.ok]
        if not good:
            return False
        accel = {a: sum(getattr(r, "accel_" + a) for r in good) / len(good) for a in "xyz"}
        self.axis = max(accel, key=lambda a: abs(accel[a]))
        if YAW_SIGN is None:
            # The axis pointing down reads -1 g, and clockwise seen from above is + about it.
            self.sign = 1.0 if accel[self.axis] < 0 else -1.0
        self.bias = sum(getattr(r, "gyro_" + self.axis) for r in good) / len(good)
        return True

    def update(self):
        reading = self.imu.read()
        if not reading.ok or reading.timestamp == self._last_timestamp:
            return self.heading
        if self.axis is None:
            self.calibrate([reading])
            self.bias = 0.0  # a moving boat gives no usable bias
        if self._last_timestamp is not None:
            dt = (reading.timestamp - self._last_timestamp) / 1000.0
            if 0 < dt < 0.25:
                rate = getattr(reading, "gyro_" + self.axis) - self.bias
                self.heading += self.sign * rate * dt
        self._last_timestamp = reading.timestamp
        return self.heading


class BeaconSearch:
    """Out / sweep / approach / grab / return. step() runs every loop and touches no hardware."""

    def __init__(self, start_heading=None, start_x=None, start_y=None):
        # read the settings now, not when the file was loaded
        start_heading = START_HEADING if start_heading is None else start_heading
        start_x = START_X if start_x is None else start_x
        start_y = START_Y if start_y is None else start_y
        self.state = "OUT"
        self.state_start = 0.0
        self.x, self.y, self.speed = start_x, start_y, 0.0
        self.heading = start_heading
        self.surge = self.yaw = 0.0
        self.samples = []      # (x, y, rssi) of every message from our beacon
        self.rssi = None       # latest, for the logs
        self.close_count = 0
        self.beacon_y = None   # current estimate on the Centerline
        self.last_fit = -9.0
        # sweep the way the bow points along our wall (facing the field: toward the far side)
        if abs(wrap(start_heading)) > 20:
            self.sweep_dir = 1.0 if wrap(start_heading) > 0 else -1.0
        else:
            self.sweep_dir = 1.0 if start_y < 0 else -1.0
        self.sweep_from = None
        self.return_mode = "bow"

    def _message(self, rssi):
        ax = self.x + ANTENNA_OFFSET * math.cos(math.radians(self.heading))
        ay = self.y + ANTENNA_OFFSET * math.sin(math.radians(self.heading))
        self.samples.append((ax, ay, rssi))
        self.rssi = rssi
        self.close_count = self.close_count + 1 if rssi >= CLOSE_RSSI else 0

    def _dead_reckon(self, dt, heading):
        factor = 1.0 if self.surge >= 0 else REVERSE_FACTOR
        target = SPEED_MPS * factor * self.surge / 100.0
        self.speed += (target - self.speed) * min(1.0, dt / SPEED_TAU)
        h = math.radians(heading)
        self.x = clamp(self.x + self.speed * math.cos(h) * dt, 0.1, POOL_LENGTH - 0.1)
        self.y = clamp(self.y + self.speed * math.sin(h) * dt, -POOL_HALF_WIDTH + 0.1, POOL_HALF_WIDTH - 0.1)
        self.heading = heading

    def _go(self, state, t):
        self.state, self.state_start = state, t

    def _steer(self, target, power, turning_power=0.0):
        """turning_power > 0 turns in an arc instead of on the spot (no room next to a wall)."""
        error = wrap(target - self.heading)
        yaw = clamp(KP_HEAD * error, -YAW_MAX, YAW_MAX)
        if abs(error) > SPIN_DEG:
            return turning_power, yaw
        return power * math.cos(math.radians(error)), yaw

    def _time_home(self, mode):
        if mode == "bow":
            turn = abs(wrap(180.0 - self.heading)) / TURN_RATE_DPS
            return turn + max(0.0, self.x - BOW_OFFSET) / SPEED_MPS
        turn = abs(wrap(0.0 - self.heading)) / TURN_RATE_DPS
        return turn + max(0.0, self.x - STERN_OFFSET) / (SPEED_MPS * REVERSE_FACTOR)

    def _must_close(self, t):
        """True when there is only just enough time left to get home."""
        quickest = min(self._time_home("bow"), self._time_home("stern"))
        return T_AUTON - t - RETURN_MARGIN_S < quickest + CLOSE_S

    def step(self, t, dt, heading, messages):
        """Returns (surge, yaw, arm_angle). surge / yaw are -100..100, + yaw = turn right."""
        self._dead_reckon(dt, heading)
        for _, rssi in messages:
            self._message(rssi)
        if t - self.last_fit >= FIT_EVERY and self.state in ("OUT", "SWEEP", "APPROACH", "FINAL"):
            self.last_fit = t
            estimate = fit_beacon_y(self.samples)
            if estimate is not None and self.state == "FINAL":
                # already going in: let the estimate move, but not jump across the pool
                step = clamp(estimate - self.beacon_y, -MAX_ESTIMATE_STEP, MAX_ESTIMATE_STEP)
                self.beacon_y += step
            elif estimate is not None:
                self.beacon_y = estimate
        getattr(self, "_" + self.state.lower())(t)
        # scale down here (not later) so the dead reckoning uses the power the fans really get
        scale = max(1.0, (abs(self.surge) + abs(self.yaw)) / 100.0)
        self.surge, self.yaw = self.surge / scale, self.yaw / scale
        holding = self.state in ("CLOSE", "RETURN", "PIN")
        return self.surge, self.yaw, ARM_CLOSED_DEG if holding else ARM_OPEN_DEG

    def _line_heading(self):
        """Heading along the sweep line, bent toward the line when the boat is off it."""
        correction = clamp((SWEEP_X - self.x) * LINE_GAIN, -45.0, 45.0)
        return self.sweep_dir * (90.0 - correction)

    def _out(self, t):
        # curve out from our wall bow-first, then along the sweep line
        if self.x >= SWEEP_X - 0.10:
            self.sweep_from = self.y
            self._go("SWEEP", t)
            return self._sweep(t)
        self.surge, self.yaw = self._steer(self.sweep_dir * 45.0, CRUISE, turning_power=ARC_POWER)

    def _sweep(self, t):
        swept = abs(self.y - self.sweep_from)
        at_end = self.y * self.sweep_dir >= SWEEP_HALF_WIDTH
        passed = (self.beacon_y is not None and swept >= MIN_SWEEP
                  and (self.beacon_y - self.y) * self.sweep_dir < -PASSED_BY)
        if passed or at_end or t >= SWEEP_DEADLINE:
            self._go("APPROACH", t)
            return self._approach(t)
        self.surge, self.yaw = self._steer(self._line_heading(), CRUISE)

    def _approach(self, t):
        if self.beacon_y is None:
            self.beacon_y = self.y
        # line up on the sweep line (clear of the Anchorage), backing up if we went past,
        # then turn in and go straight at it
        # (sideways along the line the boat can't get closer to a side wall than the line's end)
        goal_y = clamp(self._clear_of_anchorage(self.beacon_y), -SWEEP_HALF_WIDTH, SWEEP_HALF_WIDTH)
        ahead = (goal_y - self.y) * self.sweep_dir
        if abs(ahead) < LINED_UP or t - self.state_start > APPROACH_MAX_S:
            self._go("FINAL", t)
            return self._final(t)
        if self._must_close(t):
            self._go("CLOSE", t)
            return self._close(t)
        power = clamp(ahead / 0.25, -1.0, 1.0) * CRUISE
        power = math.copysign(max(abs(power), 35.0), power)
        self.surge, self.yaw = self._steer(self._line_heading(), power)

    def _final(self, t):
        beacon_y = self._clear_of_anchorage(self.beacon_y)
        reach = math.hypot(CENTER_X - self.x, beacon_y - self.y)
        if (reach <= JAW_OFFSET + 0.02 or self.close_count >= 2
                or t - self.state_start >= FINAL_MAX_S or self._must_close(t)):
            self._go("CLOSE", t)
            return self._close(t)
        # weave a little: messages from both sides of the line pin down the sideways error
        weave = FINAL_WEAVE_DEG if (t - self.state_start) % (2 * FINAL_WEAVE_S) < FINAL_WEAVE_S else -FINAL_WEAVE_DEG
        aim = bearing(self.x, self.y, CENTER_X, beacon_y)
        self.surge, self.yaw = self._steer(aim + weave, FINAL_POWER)

    @staticmethod
    def _clear_of_anchorage(beacon_y):
        """The Beacon can't be inside the Anchorage; keep the aim point beside it."""
        edge = ANCHORAGE_HALF_WIDTH + 0.09
        if abs(beacon_y) < edge:
            return edge if beacon_y >= 0 else -edge
        return beacon_y

    def _close(self, t):
        self.surge = self.yaw = 0.0
        if t - self.state_start >= CLOSE_S:
            time_left = T_AUTON - t - 0.3
            bow_time, stern_time = self._time_home("bow"), self._time_home("stern")
            if bow_time <= time_left or bow_time <= stern_time:
                self.return_mode = "bow"
            else:
                self.return_mode = "stern"
            self._go("RETURN", t)

    def _return(self, t):
        if self.return_mode == "bow":
            self.surge, self.yaw = self._steer(180.0, 100.0)
            home = self.x <= BOW_OFFSET + 0.05
        else:
            self.surge, self.yaw = self._steer(0.0, -100.0)
            home = self.x <= STERN_OFFSET + 0.05
        if home:
            self._go("PIN", t)

    def _pin(self, t):
        if self.return_mode == "bow":
            self.surge, self.yaw = self._steer(180.0, PIN_POWER)
        else:
            self.surge, self.yaw = self._steer(0.0, -PIN_POWER)


# ---------------- robot glue (called from main.py) ----------------
_still_imu_readings = []


def calibrate_imu(robot, imu_port, seconds=1.0):
    """Call while the boat is still (in initialize): samples the IMU for the gyro bias."""
    imu = robot.imu(imu_port)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        reading = imu.read()
        if reading.ok:
            _still_imu_readings.append(reading)
        time.sleep(0.02)


def run(robot, imu_port, radio_port, arm_port, left_port, right_port, balance,
        ramp_step=5.0, clock=time.monotonic, save_logs=True):
    """Runs the search until the period ends. balance(left, right) -> (left, right) duties."""
    start = clock()
    radio = BeaconListener(robot.radiobeacon(radio_port))
    heading = Heading(robot.imu(imu_port), START_HEADING)
    heading.calibrate(_still_imu_readings)
    search = BeaconSearch()
    robot.servo(arm_port).set_speed(ARM_SPEED)
    left_now = right_now = 0.0
    trace = []
    last = start
    while robot.running:
        now = clock()
        t, dt = now - start, now - last
        last = now
        if t >= T_AUTON:  # pressing Auton in VS Code may not stop at 20 s by itself
            break
        surge, yaw, arm = search.step(t, dt, heading.update(), radio.poll(t))
        left, right = balance(surge + yaw, surge - yaw)
        left_now += clamp(left - left_now, -ramp_step, ramp_step)
        right_now += clamp(right - right_now, -ramp_step, ramp_step)
        robot.thruster(left_port).set_duty(left_now)
        robot.thruster(right_port).set_duty(right_now)
        robot.servo(arm_port).angle(arm)
        trace.append((round(t, 3), search.state, round(search.x, 3), round(search.y, 3),
                      round(search.heading, 1), search.beacon_y, search.rssi,
                      round(left_now), round(right_now)))
    robot.thruster(left_port).set_duty(0)
    robot.thruster(right_port).set_duty(0)
    if save_logs:
        print(radio.summary(clock() - start))
        print(f"AUTON finished in state {search.state}, Beacon estimated at y = {search.beacon_y}")
        save_log(radio.log, trace)
    return search, radio


def save_log(messages, trace):
    try:
        LOG_DIR.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        with open(LOG_DIR / f"beacon_{stamp}.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "rx_count", "beacon_id", "rssi", "snr", "age_ms", "missed_before"])
            writer.writerows(messages)
        if trace:
            with open(LOG_DIR / f"auton_{stamp}.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t", "state", "x", "y", "heading", "beacon_y", "rssi", "fan1", "fan3"])
                writer.writerows(trace)
    except OSError as error:
        print("Could not save the auton log:", error)

# Pool checklist (once, with the real Beacon):
#   1. Run Auton with the boat held still: logs/beacon_*.csv shows messages per second and
#      rssi. "missed" > 0 means the Beacon sends faster than we read it.
#   2. Beacon inside the open arm: lowest rssi seen there -> CLOSE_RSSI.
#   3. Time 2 m straight at full power -> SPEED_MPS. Time one full spin -> TURN_RATE_DPS.
#      These two matter most: the position estimate is built on them.
#   4. Run Auton with the boat out of the water and turn it right by hand: the heading column
#      in logs/auton_*.csv must go UP. If it goes down, set YAW_SIGN = -1.
#   5. If the log shows more than one beacon_id (other pools nearby), put ours in BEACON_ID.
#   6. Measure where the radio module sits: ANTENNA_OFFSET (m ahead of the robot centre).
