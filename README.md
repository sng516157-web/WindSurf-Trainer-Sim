# WindSurf Trainer Sim

**Repository:** [github.com/sng516157-web/WindSurf-Trainer-Sim](https://github.com/sng516157-web/WindSurf-Trainer-Sim)

Land-based **windsurf / sailing-board trainer** simulation built in [Webots R2025a](https://cyberbotics.com/). A six-degree-of-freedom Stewart platform reproduces sea conditions while athlete input—keyboard or **force/torque sensor**—and an optional **sail load model** perturb the deck via admittance control.

## Features

- **6-DOF Stewart platform** with inverse kinematics for six linear pistons
- **Sea waves** — configurable multi-frequency surge, sway, heave, roll, pitch, yaw (`SEA_STATE`)
- **Buoyancy model** — spring–damper waterline dynamics (always active); waves toggle independently
- **Minimal sail model** — wind, sheet trim, gusts; separate admittance path with stability limits (`sail_force.py`)
- **Keyboard or F/T athlete input** — switchable in the controller (`ATHLETE_INPUT_MODE`)
- **Hardware presets** — full sim and scaled **desktop** geometry (`hardware_config.py`, `HARDWARE_SCALE = 0.15`)
- **Admittance control** — athlete + sail wrenches → pose deltas on top of waves
- **Pose feedback** — deck `GPS` + `InertialUnit` compared to commanded pose (HUD + console)
- **CSV session logging** — F/T wrench + command/measured pose for sim validation and hardware bring-up
- **Live plots** — matplotlib command wrench (solid) + sensor force overlay (dotted); `ENABLE_LIVE_PLOTS`

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

## Controller configuration

Edit the flags at the top of `controllers/sailing_board_platform/sailing_board_platform.py`:

| Flag | Values | Purpose |
|------|--------|---------|
| `PLATFORM_PRESET` | `"sim_full"`, `"desktop_hardware"` | Geometry, sea, admittance, sail scaling |
| `ATHLETE_INPUT_MODE` | `"keyboard"`, `"ft_sensor"` | Keyboard CoG/moments vs F/T wrench |
| `ENABLE_SAIL` | `True` / `False` | Include sail aerodynamic load model |
| `ENABLE_CSV_LOGGING` | `True` / `False` | Write `logs/session_*.csv` each run |
| `ENABLE_LIVE_PLOTS` | `True` / `False` | matplotlib charts (slower in real-time) |
| `PLOT_UPDATE_INTERVAL` | e.g. `0.25` | Seconds between plot refreshes |

**F/T mode** reads the Webots `athlete_ft_sensor` (force-3d), estimates torque via `r × F`, calibrates bias at startup, then drives admittance—the same path planned for a physical 6-axis sensor.

CSV logs land in `controllers/sailing_board_platform/logs/` (gitignored).

## Keyboard controls

Focus the **3D simulation window** before typing.

### Athlete

| Keys | Action |
|------|--------|
| **A / D** | Shift CoG sideways (sway) |
| **W / S** | Shift CoG fore/aft (surge) |
| **Q / E** | Shift CoG up/down (heave) |
| **← / →** | Apply roll moment |
| **↑ / ↓** | Apply pitch moment |
| **Z / X** | Apply yaw moment |
| **P** | Toggle **waves** on/off (buoyancy stays on) |

### Sail (`ENABLE_SAIL = True`)

| Keys | Action |
|------|--------|
| **R / C** | Sheet in / out (more / less power) |
| **L** | Toggle **sail load** on/off |

CoG and applied moments **relax smoothly** to neutral when keys are released (keyboard input abstraction only—not full sail physics).

**Balancing sail:** Sail side force heels the board to starboard; hike to port (**A**) and roll moment (**←**) to counter. Depower with **C** or disable sail with **L** while tuning.

## On-screen feedback

The HUD (top-left) shows:

- Wave on/off, buoyancy status, sail on/off, input mode
- Sail line: wind speed, apparent wind angle, sheet, side force, roll torque (when enabled)
- Athlete CoG (keyboard) or live F/T wrench (sensor mode)
- **Cmd deck** — commanded 6-DOF pose
- **Meas deck** — GPS + IMU measured pose
- **Track err** — command vs measurement (xyz in mm, rpy in degrees)

Console logs once per second include piston saturation count (`0–6`).

## Project layout

```
WindSurf-Trainer-Sim/
├── README.md
├── worlds/
│   └── stewart_platform.wbt
└── controllers/
    └── sailing_board_platform/
        ├── sailing_board_platform.py   # Main supervisor controller
        ├── athlete_input.py            # Keyboard CoG + moments
        ├── athlete_ft_input.py         # F/T sensor input (hardware + sim)
        ├── hardware_config.py          # sim_full + desktop_hardware presets
        ├── session_logger.py           # CSV F/T + pose logging
        ├── sail_force.py               # Minimal sail aerodynamic load
        ├── admittance.py
        ├── buoyancy.py
        ├── force_plotter.py
        └── requirements.txt
```

## Tuning

Presets in `hardware_config.py` bundle geometry, `SEA_STATE`, admittance, buoyancy, and F/T filter settings.

| Constant | Purpose |
|----------|---------|
| `SEA_STATE` | Wave amplitudes and periods per axis |
| `TRANSLATION_ADMITTANCE`, `ROTATION_ADMITTANCE` | Athlete wrench → deck motion |
| `sail_admittance_factor` | Sail wrench gain (default 0.30 of athlete admittance) |
| `sail_force_scale`, `sail_torque_scale` | Per-preset sail load scaling |
| `sail_max_side_force`, `sail_max_roll_torque` | Sail wrench caps |
| `TRACK_STIFFNESS`, `BUOYANCY_STIFFNESS`, `HYDRO_DAMPING` | Water feel |
| `ft_force_deadband`, `ft_torque_deadband` | Ignore sensor noise at rest |

**More oscillation:** lower `HYDRO_DAMPING`, slightly raise stiffness.  
**Stronger athlete effect:** raise admittance gains (watch piston saturation).  
**Sail too aggressive:** lower `sail_admittance_factor` or press **C** / **L** to depower or disable sail.

Sail stability guards in software: separate sail admittance, per-step pose caps, command roll/pitch clamp (±32°), heel-dependent force fade (no sign flip past 90°).

## Architecture

```
SEA_STATE (waves) ──► equilibrium pose ──┐
                                         ├──► buoyancy ──► command pose ──► IK ──► 6 pistons
keyboard / F/T ──► athlete admittance ───┤
sail (wind)    ──► sail admittance   ────┘
                              ▲
                    GPS + IMU ├── tracking / HUD / CSV
                    TouchSensor or 6-axis F/T
```

## Live plots

Set `ENABLE_LIVE_PLOTS = True` and install matplotlib. Plots show:

- **Solid lines** — combined command force/torque (athlete + sail)
- **Dotted lines** — TouchSensor force (may read ~0 in Webots due to ball-joint placement)

If the plot window freezes, keep `plt.pause` enabled in `force_plotter.py` (default). Increase `PLOT_UPDATE_INTERVAL` if performance suffers.

---

## Physical prototype plans

Two build targets share the same control stack (`athlete_ft_input` → admittance → pose → IK → actuators). The sim presets map directly to each scale.

### Plan A — Bench / lab Stewart (training-scale path)

**Goal:** Prove full **6-DOF** windsurf training on a reduced platform before a room-sized rig.

| Parameter | Full sim | Bench target |
|-----------|----------|--------------|
| Base footprint | ~3.7 m | **1.0–1.5 m** |
| Piston stroke | ±400 mm | ±80–120 mm |
| Payload | 70 kg athlete | **30–70 kg** |
| Tilt range | ~±30° | **±25–30°** |
| Input | F/T under board | 6-axis sensor (ATI Mini40 class) |

**Mechanism:** Scale the existing Stewart geometry (~0.35–0.40×) or use a commercial mini hexapod frame. Reuse `BASE_ANCHORS` / `PLATFORM_LOCAL_ANCHORS` from `hardware_config.py` with a new preset.

**Electronics:**

- 6× linear actuators with position feedback (ball screw or electric cylinder)
- Motor drivers with current limit + **hardware E-stop** on enable
- Deck IMU (BNO085 / ICM-42688) + actuator encoders
- 6-axis F/T (e.g. ATI Mini40, OnRobot HEX) — target ±120 N, ±8 N·m
- Real-time host (PC or Raspberry Pi 5) at **100 Hz** supervisor, **1 kHz** per-axis position loops on drives

**Control loop (100 Hz):**

1. Read F/T → bias subtract → low-pass → `(F, τ)`
2. `Δpose = AdmittanceControl.pose_delta(F, τ)`
3. `pose_cmd = sea + buoyancy + Δpose` (+ optional `Kp·(cmd − meas)`)
4. IK → actuator commands, workspace limits, CSV log

**Phases:**

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| 0 — Sim parity | 1–2 wk | F/T mode + CSV in Webots; replay logs |
| 1 — Fixed deck + F/T | 2–3 wk | Calibrated sensor on rigid table; hiking force baselines |
| 2 — Open-loop 6-DOF | 4–6 wk | Frame built; IMU tracks commanded pose |
| 3 — Closed-loop admittance | 2–4 wk | Human on board; sea toggle; tune gains |
| 4 — Training UX | ongoing | Session scoring, presets, optional VR sea view |

**Budget (indicative):** $8k–20k (F/T + actuators dominate).

**Success criteria:** Predictable heel from weight shift without limit-cycle oscillation; track error < 5 mm / 2° in calm seas; hardware F/T log replays in Webots with similar pose response.

---

### Plan B — Desktop prototype (~600 mm)

**Goal:** **Desk-sized demonstrator** for admittance + sea motion + F/T control—fast to build, fits a lab bench.

| Parameter | Value |
|-----------|-------|
| Base span | **~600 mm** (`HARDWARE_SCALE = 0.15`) |
| DOF (MVP) | **3-DOF** roll + pitch + heave (upgrade to 6-DOF later) |
| Deck travel | ±12 mm heave, ±20–25° tilt |
| Payload | Hands + **5–15 kg** dummy mass (not full human) |
| Control rate | 100 Hz supervisor |

**Mechanism (recommended MVP):** Triangular **3-actuator** table—not a full Stewart yet.

```
         actuator ×3 (120° on base plate)
              /    |    \
         base plate ─── top plate (deck)
              │
         [6-axis F/T or 3× load cell DIY]
              │
         small board / MDF deck
```

**Why 3-DOF first:** Covers most windsurf balance cues (heel, trim, chop) at ~30% cost/complexity of a hexapod. Software preset `desktop_hardware` already scales sea and admittance; IK module can be swapped for a 3-leg solver while keeping `athlete_ft_input` and `session_logger` unchanged.

**Desktop BOM (indicative):**

| Item | Spec |
|------|------|
| Linear actuators ×3 | 50–100 mm stroke, 300–800 N |
| F/T sensor | ATI Nano17 / Robotiq FT 300, or DIY 3-cell |
| IMU | BNO085 on deck |
| Controller | Raspberry Pi 5 or laptop + USB |
| Frame | 2020 extrusion + 10 mm Al top plate |
| Safety | E-stop, stroke limits, max tilt clamp |

**Scaled geometry (`desktop_hardware` preset):**

Base anchors (m, world frame, scale 0.15):

| ID | x | y | z |
|----|------|------|------|
| B0 | −0.1814 | +0.2362 | +0.0181 |
| B1 | −0.1814 | −0.2362 | +0.0181 |
| B2 | −0.1138 | −0.2752 | +0.0181 |
| B3 | +0.2953 | −0.0390 | +0.0181 |
| B4 | +0.2953 | +0.0390 | +0.0181 |
| B5 | −0.1138 | +0.2752 | +0.0181 |

Platform local anchors (m, body frame, scale 0.15):

| ID | x | y | z |
|----|------|------|------|
| P0 | +0.2203 | +0.1568 | −0.1894 |
| P1 | +0.2115 | +0.2069 | −0.1600 |
| P2 | +0.2115 | +0.2069 | +0.0617 |
| P3 | +0.2203 | +0.1568 | +0.0911 |
| P4 | +0.2536 | −0.0323 | −0.0198 |
| P5 | +0.2536 | −0.0323 | −0.0785 |

Neutral platform centre: **[+0.0771, +0.0353, +0.1377] m**. Piston stroke: **±60 mm**. Neutral leg length: **0.42 m**.

Print full geometry from Python: `python -c "from hardware_config import desktop_anchor_table; print(desktop_anchor_table())"` (run from `controllers/sailing_board_platform/`).

**F/T tuning (desktop):** `ft_force_deadband = 1.5 N`, `ft_torque_deadband = 0.08 N·m`, rotation admittance ~10× stiffer than full sim—tune until ~5 N at the deck edge yields ~2–5° tilt.

**Phases:**

| Phase | Deliverable |
|-------|-------------|
| 1 | F/T on fixed desk; CSV logging; bias calibration |
| 2 | 3-DOF frame; open-loop pose tracking |
| 3 | `ATHLETE_INPUT_MODE = "ft_sensor"` on hardware port of `athlete_ft_input.py` |
| 4 | Sea motion feedforward; optional 6-DOF Stewart upgrade using same anchors |

**Budget (indicative):** $2.5k–6k (sensor + 3 actuators + frame).

**Bring-up checklist:**

- [ ] Set `PLATFORM_PRESET = "desktop_hardware"` in sim to validate scaled seas
- [ ] Run with `ATHLETE_INPUT_MODE = "ft_sensor"`; confirm CSV columns match hardware logger
- [ ] Hardware E-stop tested before human contact
- [ ] Replay CSV wrench through Webots for sim/hardware parity

---

## Roadmap

- [x] CSV session logging (command vs measured pose + F/T)
- [x] F/T athlete input module (`athlete_ft_input.py`)
- [x] Desktop hardware preset (`HARDWARE_SCALE = 0.15`)
- [x] Minimal sail force module (`sail_force.py`)
- [ ] Sea presets (calm / moderate / rough)
- [ ] 3-DOF desktop IK module
- [ ] Sailing-board mesh and dedicated world
- [ ] Closed-loop pose tracking for hardware (`Kp·(cmd − meas)`)
- [ ] Hardware I/O layer (actuators + IMU + real F/T SDK)

## License

Controller files include the Webots/Cyberbotics Apache 2.0 header where derived from Webots examples. See file headers for details.

## Acknowledgements

- [Cyberbotics Webots](https://cyberbotics.com/) — simulation platform and Stewart platform demo world basis
