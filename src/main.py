"""A MARLIN robot with tank drive controls, servo control, and sensor reading printouts."""

import json
import time
from pathlib import Path
from marlin import Robot, Thruster, Servo, IMU, RadioBeacon

import auton

# Brain port definitions
IMU_PORT = "S1"
RADIOBEACON_PORT = "S6"
ARM = "M2"

# Which thruster port each joystick drives. Swap these two lines if they feel the wrong way round.
# Auton assumes stick 1's fan is the LEFT one and stick 3's fan the RIGHT one.
STICK1_THRUSTER = "M8"
STICK3_THRUSTER = "M1"

# Stick sensitivity, set SEPARATELY for forward (stick pushed up) and backward (stick pulled down).
# Changing a forward value never affects backward, and the other way round.
# 1.0 = normal. Above 1.0 the thruster reaches full power before the stick is pushed all the way.
FORWARD_GAIN = {"ONE": 3.0, "THREE": 3.0}
BACKWARD_GAIN = {"ONE": 1.0, "THREE": 1.0}

# Thruster balance, one value for forward and one for backward. 0.0 = no correction.
# Positive = stick 3's thruster is turned down. Negative = stick 1's thruster is turned down.
# Tuned live with the D-pad and saved to balance.json, which is loaded on the next run.
BALANCE_FILE = Path(__file__).resolve().parent.parent / "balance.json"
BALANCE_STEP = 0.02
BALANCE_LIMIT = 0.5

# Press A to switch between slow mode (thrusters capped at SLOW_POWER) and full power.
SLOW_POWER = 0.6

# Press LEFT_TRIGGER to switch the stick cap on/off. While on, sticks 1 and 3 are capped at
# +CAP_LIMIT forward and -CAP_LIMIT backward.
CAP_LIMIT = 50

# Smoothing: how fast a fan's power may change, in duty per second. 250 = 0 to full in 0.4 s.
# Stops the fans jerking when the stick wobbles. Lower = smoother but slower to respond.
RAMP_PER_SECOND = 250
RAMP_STEP = RAMP_PER_SECOND / 50  # the loop runs 50 times a second

# Fan test (press X to start, X again to cancel): runs each fan ALONE at fixed, steady power,
# ignoring the sticks. If a fan still vibrates here, the cause is the fan, not the code or stick.
# Each step is (fan, duty, seconds). Keep the boat held or in the water while it runs.
FAN_TEST = [
    ("fan1", 15, 3), ("fan1", 30, 3), ("fan1", 60, 3), ("fan1", 100, 3), ("fan1", 0, 2),
    ("fan3", 15, 3), ("fan3", 30, 3), ("fan3", 60, 3), ("fan3", 100, 3), ("fan3", 0, 2),
    ("fan1", -60, 3), ("fan1", 0, 2), ("fan3", -60, 3),
]

# Every button on the controller.
BUTTONS = ("UP", "DOWN", "LEFT", "RIGHT", "X", "Y", "A", "B", "LEFT_TRIGGER", "RIGHT_TRIGGER")

# Stick values smaller than this are treated as 0 so the thrusters don't creep.
# The sticks were measured resting at up to 13 away from 0, so this needs to be above that.
DEADZONE = 15

# The status line only prints when something changes, at most every PRINT_EVERY loops
# (50 loops = 1 second). Stick changes smaller than PRINT_STEP are ignored.
PRINT_EVERY = 10
PRINT_STEP = 10
PRINT_SENSORS = False

robot = Robot()

robot.configure(
    sensors={
        IMU_PORT: IMU,
        RADIOBEACON_PORT: RadioBeacon
    },
    effectors={
        STICK1_THRUSTER: Thruster,
        STICK3_THRUSTER: Thruster,
        ARM: Servo
    },
)


def deadzone(value):
    """0 inside the deadzone, then rises smoothly from 0 to 100 so no power is lost at the top."""
    if abs(value) < DEADZONE:
        return 0.0
    sign = 1 if value > 0 else -1
    return sign * min(100.0, (abs(value) - DEADZONE) / (100 - DEADZONE) * 100)


def load_balance():
    """Return {"forward": x, "reverse": y} from balance.json, or zeros if there is no file yet."""
    try:
        saved = json.loads(BALANCE_FILE.read_text())
        return {"forward": float(saved["forward"]), "reverse": float(saved["reverse"])}
    except (OSError, ValueError, KeyError, TypeError):
        return {"forward": 0.0, "reverse": 0.0}


def save_balance(balance):
    try:
        BALANCE_FILE.write_text(json.dumps(balance, indent=2))
    except OSError as error:
        print("Could not save balance:", error)


def read_stick(robot, name):
    """Stick value with its own forward or backward sensitivity applied, then the deadzone."""
    value = robot.controller.joystick(name).value
    gain = FORWARD_GAIN[name] if value >= 0 else BACKWARD_GAIN[name]
    return deadzone(value * gain)


def balance_scales(balance):
    """Turn a balance value into (stick1_scale, stick3_scale). The weaker side always stays at 1.0."""
    stick1_scale = 1.0 - max(0.0, -balance)
    stick3_scale = 1.0 - max(0.0, balance)
    return stick1_scale, stick3_scale


def balanced(duty1, duty3, balance):
    """Apply the forward or backward balance to each fan, depending on which way it pushes."""
    stick1_scale = balance_scales(balance["forward" if duty1 >= 0 else "reverse"])[0]
    stick3_scale = balance_scales(balance["forward" if duty3 >= 0 else "reverse"])[1]
    return duty1 * stick1_scale, duty3 * stick3_scale


def describe_balance(balance):
    stick1_scale, stick3_scale = balance_scales(balance)
    return f"stick 1 fan {stick1_scale:.0%}, stick 3 fan {stick3_scale:.0%}"


def initialize(robot):
    time.sleep(0.5)
    print("Battery:", robot.battery_voltage, "V")

    # Set Servo speed to 120 deg/s
    robot.servo(ARM).set_speed(120)

    # The boat is still here: measure the gyro bias for the auton heading.
    auton.calibrate_imu(robot, IMU_PORT)


def autonomous(robot):
    # Find the Beacon by radio, grab it and bring it home. Settings are at the top of auton.py.
    balance = load_balance()
    auton.run(
        robot,
        imu_port=IMU_PORT,
        radio_port=RADIOBEACON_PORT,
        arm_port=ARM,
        left_port=STICK1_THRUSTER,
        right_port=STICK3_THRUSTER,
        balance=lambda duty1, duty3: balanced(duty1, duty3, balance),
        ramp_step=RAMP_STEP,
    )


def driver(robot):
    balance = load_balance()
    slow_mode = False
    capped = False
    was_down = {name: False for name in BUTTONS}
    loop_count = 0
    smooth1 = 0.0
    smooth3 = 0.0
    test_step = None   # index into FAN_TEST while the fan test runs, else None
    test_loops = 0     # loops left in the current test step
    last_printed = None
    last_print_loop = -PRINT_EVERY

    print("Driver started. Press A to switch between 60% and full power.")
    print(f"BALANCE forward: {describe_balance(balance['forward'])}")
    print(f"BALANCE backward: {describe_balance(balance['reverse'])}")

    while robot.running:

        # Forward and backward each use their own sensitivity (FORWARD_GAIN / BACKWARD_GAIN).
        stick1 = read_stick(robot, "ONE")
        stick3 = read_stick(robot, "THREE")

        # Print every new button press, so each button can be checked by hand.
        # A: switch slow mode on/off (like caps lock).
        # D-pad: press the side of the stick whose fan is TOO STRONG to turn it down 2%.
        #   LEFT = turn down stick 1's fan, RIGHT = turn down stick 3's fan, DOWN = reset.
        # Changes the FORWARD balance. Hold RIGHT_TRIGGER to change the BACKWARD balance instead.
        for name in BUTTONS:
            is_down = robot.controller.button(name).is_down
            if is_down and not was_down[name]:
                print("Pressed:", name)
                direction = "reverse" if robot.controller.button("RIGHT_TRIGGER").is_down else "forward"
                if name == "X":
                    if test_step is None:
                        test_step, test_loops = 0, 0
                        print("FAN TEST started (press X again to cancel)")
                    else:
                        test_step = None
                        print("FAN TEST cancelled")
                elif name == "A":
                    slow_mode = not slow_mode
                    print("POWER:", f"{SLOW_POWER:.0%} (slow mode)" if slow_mode else "100% (full power)")
                elif name == "LEFT_TRIGGER":
                    capped = not capped
                    print("CAP:", f"sticks capped at +/-{CAP_LIMIT}" if capped else "off (up to 100)")
                elif name == "LEFT":
                    balance[direction] = max(-BALANCE_LIMIT, balance[direction] - BALANCE_STEP)
                elif name == "RIGHT":
                    balance[direction] = min(BALANCE_LIMIT, balance[direction] + BALANCE_STEP)
                elif name == "DOWN":
                    balance[direction] = 0.0
                if name in ("LEFT", "RIGHT", "DOWN"):
                    balance[direction] = round(balance[direction], 2)
                    save_balance(balance)
                    label = "backward" if direction == "reverse" else "forward"
                    print(f"BALANCE {label}: {describe_balance(balance[direction])} (saved)")
            was_down[name] = is_down

        # The cap applies to the sticks before the balance, so the balance still works while capped.
        if capped:
            stick1 = max(-CAP_LIMIT, min(CAP_LIMIT, stick1))
            stick3 = max(-CAP_LIMIT, min(CAP_LIMIT, stick3))

        power = SLOW_POWER if slow_mode else 1.0
        # Each thruster uses the forward or backward balance depending on which way it is pushing.
        duty1, duty3 = balanced(stick1 * power, stick3 * power, balance)

        # Fan test overrides the sticks with fixed power on one fan at a time.
        if test_step is not None:
            if test_loops == 0:
                if test_step >= len(FAN_TEST):
                    test_step = None
                    print("FAN TEST finished")
                else:
                    fan, test_duty, seconds = FAN_TEST[test_step]
                    test_loops = seconds * 50
                    port = STICK1_THRUSTER if fan == "fan1" else STICK3_THRUSTER
                    print(f"FAN TEST step {test_step + 1}/{len(FAN_TEST)}: {fan} (stick {fan[-1]}, {port}) at {test_duty}")
            if test_step is not None:
                fan, test_duty, _ = FAN_TEST[test_step]
                duty1 = test_duty if fan == "fan1" else 0.0
                duty3 = test_duty if fan == "fan3" else 0.0
                test_loops -= 1
                if test_loops == 0:
                    test_step += 1

        # Move each fan towards its target power by at most RAMP_STEP per loop.
        smooth1 += max(-RAMP_STEP, min(RAMP_STEP, duty1 - smooth1))
        smooth3 += max(-RAMP_STEP, min(RAMP_STEP, duty3 - smooth3))
        duty1, duty3 = smooth1, smooth3

        robot.thruster(STICK1_THRUSTER).set_duty(duty1)
        robot.thruster(STICK3_THRUSTER).set_duty(duty3)

        if robot.controller.button("LEFT_TRIGGER").is_down:
            robot.servo(ARM).angle(180)
        else:
            robot.servo(ARM).angle(0)

        # Status line: only when a stick has moved by PRINT_STEP or more, and not too often.
        loop_count += 1
        c = robot.controller
        sticks = tuple(c.joystick(n).value for n in ("ONE", "TWO", "THREE", "FOUR"))
        rounded = tuple(round(v / PRINT_STEP) for v in sticks)
        if rounded != last_printed and loop_count - last_print_loop >= PRINT_EVERY:
            last_printed = rounded
            last_print_loop = loop_count
            print(
                f"joysticks 1={sticks[0]:4.0f} 2={sticks[1]:4.0f} 3={sticks[2]:4.0f} 4={sticks[3]:4.0f} | "
                f"fan1={duty1:4.0f} fan3={duty3:4.0f} | power={power:.0%} cap={'on' if capped else 'off'} "
                f"balance fwd={balance['forward']:+.2f} back={balance['reverse']:+.2f}"
            )

            if PRINT_SENSORS:
                # print IMU readings
                imu_reading = robot.imu(IMU_PORT).read()

                if imu_reading.ok:
                    print(imu_reading)

                # print RadioBeacon readings
                radiobeacon_reading = robot.radiobeacon(RADIOBEACON_PORT).read()

                if radiobeacon_reading.ok:
                    print(radiobeacon_reading)


robot.run(
    initialize=initialize,
    autonomous=autonomous,
    driver=driver,
)
