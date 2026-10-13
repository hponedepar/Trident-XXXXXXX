# Trident-X 2026 — MARLIN boat code

Driver and autonomous code for our MARLIN Robotics *Link Lockdown* boat (Trident-X 2026).

Open this folder in VS Code with the MARLIN Controller extension installed (the folder that
contains `project.marlin`), connect the Controller and Robot Brain, then press **Driver** or **Auton**.

## Files

| File | What it is |
|---|---|
| `src/main.py` | Robot code: tank drive, fan balance, speed modes, fan test |
| `project.marlin` | Marks this as a MARLIN project; also stores the joystick calibration |
| `balance.json` | Fan balance tuned with the D-pad (loaded automatically) |

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
