# Trident-X 2026 — MARLIN boat code

Driver and autonomous code for our MARLIN Robotics *Link Lockdown* boat (Trident-X 2026).

Open this folder in VS Code with the MARLIN Controller extension installed (the folder that
contains `project.marlin`), connect the Controller and Robot Brain, then press **Driver** or **Auton**.

## Files

| File | What it is |
|---|---|
| `src/main.py` | Robot code: tank drive, fan balance, speed modes, fan test |
| `src/auton.py` | Autonomous Period: find the Beacon by radio, grab it, bring it to our Starting Area |
| `tools/sim_auton.py` | Runs `auton.py` against a simulated pool (`python tools/sim_auton.py`) |
| `project.marlin` | Marks this as a MARLIN project; also stores the joystick calibration |
| `balance.json` | Fan balance tuned with the D-pad (loaded automatically) |
| `logs/` | Written after every Auton run: every Beacon message and a 50 Hz trace (not committed) |

## Autonomous (first 20 s)

Before **every** match, set the start pose at the top of `src/auton.py`:

- `START_HEADING`: which way the bow points. `+90` = along our wall to the right, `-90` = to the left,
  `0` = straight at the field. The boat is longer than the 0.4 m Starting Area, so it starts sideways.
- `START_Y`: where along our wall (`-0.45` = left end, `0` = middle, `+0.45` = right end).
  The best start is at one end of the wall with the bow pointing toward the other end.

What it does: it curves out to a line in front of the Anchorage and sweeps across the pool. Every radio
message is stored together with where the boat was when it arrived, and all of them are fitted to
"signal drops with distance" to estimate where on the Centerline the Beacon is. Then it lines up, goes
in, closes the arm and drives back bow first, pushing the Beacon against our wall until the period ends.

The radio only gives signal strength (`rssi`), and `read()` only holds the latest message, so the code
reads it every loop and uses `rx_count` to catch every message. The log reports how many were missed.

**Calibrate in the pool first.** The checklist is at the bottom of `src/auton.py`. The most important
values are the speed and turn rate (`SPEED_MPS`, `TURN_RATE_DPS`), the arm angles (`ARM_OPEN_DEG`,
`ARM_CLOSED_DEG`), `CLOSE_RSSI`, and checking that the heading goes up when the boat turns right.

Simulated success rate (`tools/sim_auton.py`, guessed boat and radio): about 5-10 % with a grab zone
of 0.10 m around the arm, about 30 % with 0.20 m. A wide mouth on the arm matters more than tuning.

## Controller

| Control | Action |
|---|---|
| Joystick 1 / 3 | Drive the two fans (tank drive) |
| A | Toggle slow mode (60% power) |
| Left trigger | Toggle cap (sticks limited to ±50) |
| D-pad LEFT / RIGHT | Turn down stick 1's / stick 3's fan by 2% (forward balance) |
| Right trigger + D-pad | Same, for the backward balance |
| D-pad DOWN | Reset balance (hold right trigger for backward) |
| X | Run / cancel the fan test (each fan alone at fixed power) |

Settings such as stick sensitivity (`FORWARD_GAIN`, `BACKWARD_GAIN`), smoothing (`RAMP_PER_SECOND`)
and which port each stick drives (`STICK1_THRUSTER`, `STICK3_THRUSTER`) are at the top of `src/main.py`.
