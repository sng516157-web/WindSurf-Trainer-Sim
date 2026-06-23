# WindSurf Trainer Sim

Land-based **windsurf / sailing-board trainer** simulation built in [Webots R2025a](https://cyberbotics.com/). A six-degree-of-freedom Stewart platform reproduces sea conditions while a keyboard-driven athlete model shifts centre of gravity and applies moments on the deck—standing in for a real sailor on a physical motion platform.

## Features

- **6-DOF Stewart platform** with inverse kinematics for six linear pistons
- **Sea waves** — configurable multi-frequency surge, sway, heave, roll, pitch, yaw (`SEA_STATE`)
- **Buoyancy model** — spring–damper waterline dynamics (always active); waves toggle independently
- **Keyboard athlete** — CoG shift and applied moments with smooth return to neutral stance
- **Admittance control** — athlete wrench perturbs deck pose on top of waves
- **Pose feedback** — deck `GPS` + `InertialUnit` compared to commanded pose (HUD + console)
- **Force sensing** — `TouchSensor` (force-3d) between athlete and deck
- **Optional live plots** — matplotlib force/torque charts (`ENABLE_LIVE_PLOTS`)

## Requirements

- [Webots R2025a](https://cyberbotics.com/download) (or compatible R2025a build)
- Python 3.x (used by Webots controllers)
- Optional: `matplotlib` for live plots  
  ```bash
  pip install -r controllers/sailing_board_platform/requirements.txt
  ```

## Quick start

1. Clone this repository.
2. Open **`worlds/stewart_platform.wbt`** in Webots.
3. Confirm the `STEWART_PLATFORM` robot **controller** is set to `sailing_board_platform`.
4. Press **Play** (real-time or fast mode).
5. **Click the 3D view** so it has focus, then use the keyboard controls below.

## Keyboard controls

Focus the **3D simulation window** before typing.

| Keys | Action |
|------|--------|
| **A / D** | Shift CoG sideways (sway) |
| **W / S** | Shift CoG fore/aft (surge) |
| **Q / E** | Shift CoG up/down (heave) |
| **← / →** | Apply roll moment |
| **↑ / ↓** | Apply pitch moment |
| **Z / X** | Apply yaw moment |
| **P** | Toggle **waves** on/off (buoyancy stays on) |

CoG and applied moments **relax smoothly** to neutral when keys are released.

## On-screen feedback

The HUD (top-left) shows:

- Wave on/off and buoyancy status
- Athlete CoG offset
- **Cmd deck** — commanded 6-DOF pose
- **Meas deck** — GPS + IMU measured pose
- **Track err** — command vs measurement (xyz in mm, rpy in degrees)

Console logs once per second include piston saturation count (`0–6`); values near **6/6** mean the Stewart workspace limit (±0.4 m piston travel) is reached.

## Project layout

```
WindSurf-Trainer-Sim/
├── README.md
├── worlds/
│   └── stewart_platform.wbt      # Main simulation world
└── controllers/
    └── sailing_board_platform/
        ├── sailing_board_platform.py   # Main supervisor controller
        ├── athlete_input.py            # Keyboard CoG + moments
        ├── admittance.py               # Wrench → pose offset
        ├── buoyancy.py                 # Floating deck dynamics
        ├── force_plotter.py            # Optional matplotlib plots
        └── requirements.txt
```

## Tuning

Main constants live at the top of `controllers/sailing_board_platform/sailing_board_platform.py`:

| Constant | Purpose |
|----------|---------|
| `SEA_STATE` | Wave amplitudes and periods per axis |
| `COG_LIMITS`, `COG_RATE` | Athlete CoG range and speed |
| `MOMENT_LIMIT`, `MOMENT_RATE` | Applied torque limits |
| `TRANSLATION_ADMITTANCE`, `ROTATION_ADMITTANCE` | How strongly input moves the deck |
| `TRACK_STIFFNESS`, `BUOYANCY_STIFFNESS`, `HYDRO_DAMPING` | Water feel (oscillation vs damping) |
| `ENABLE_LIVE_PLOTS` | matplotlib plots (off by default for performance) |

**More oscillation:** lower `HYDRO_DAMPING`, slightly raise stiffness.  
**Stronger athlete effect:** raise admittance gains or `COG_LIMITS` (watch piston saturation).

## Architecture

```
SEA_STATE (waves) ──► equilibrium pose ──┐
                                         ├──► buoyancy dynamics ──► command pose ──► IK ──► 6 pistons
keyboard athlete ──► wrench ──► admittance ──┘
                              ▲
                    GPS + IMU ├── tracking / HUD
                    TouchSensor
```

## Roadmap

- [ ] CSV session logging (command vs measured pose)
- [ ] Sea presets (calm / moderate / rough)
- [ ] Sailing-board mesh and dedicated world
- [ ] Closed-loop pose tracking for hardware
- [ ] Real 6-axis load-cell input (replace keyboard)

## License

Controller files include the Webots/Cyberbotics Apache 2.0 header where derived from Webots examples. See file headers for details.

## Acknowledgements

- [Cyberbotics Webots](https://cyberbotics.com/) — simulation platform and Stewart platform demo world basis
