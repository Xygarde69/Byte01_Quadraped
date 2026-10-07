# AK60 Quadruped – MuJoCo Simulation

A MuJoCo model of a 12-DoF quadruped whose joints are modelled on the **CubeMars AK60-6 V3.0 (KV80)** actuator with an additional **5.8:1 external reduction (70 % efficiency)**. A Python script drives the robot from the all-zero joint pose to a standing pose with a torque-level PD controller, holds it, and shows everything in a real-time viewer with a live, scrolling plot dashboard.

> **Status:** the model and script were written against the MuJoCo XML reference and checked for syntax, but have not been run against the original STL meshes. Treat the first run as a commissioning step (see [Troubleshooting](#troubleshooting)).

---

## Contents

| File | Purpose |
|---|---|
| `../Models/byte01.xml` | MJCF robot model (links, joints, motors, sensors, floor) |
| `main.py` | Real-time sim + PD/feedforward controller + live matplotlib dashboard |
| `meshes/` | **You provide:** `base_link.STL`, `hip_*.STL`, `thigh_*.STL` / `Thigh_fr.STL`, `calf_*.STL` |

## Requirements

- Python 3.9+
- MuJoCo >= 3.0 (needed for the `implicitfast` integrator and the viewer)
- `numpy`, `matplotlib`

### Installation

Recommended: use a virtual environment.

```bash
# create and activate a virtual environment
python -m venv .venv
source .venv/bin/activate          # Linux / macOS
# .venv\Scripts\activate           # Windows (cmd / PowerShell)

# install dependencies
python -m pip install --upgrade pip
pip install "mujoco>=3.0" numpy matplotlib
```

Or without a virtual environment:

```bash
pip install mujoco numpy matplotlib
```

Optional: pin everything in a `requirements.txt` and install with `pip install -r requirements.txt`:

```
mujoco>=3.0
numpy
matplotlib
```

Check the install:

```bash
python -c "import mujoco, numpy, matplotlib; print('MuJoCo', mujoco.__version__)"
```

On Linux the viewer and matplotlib windows need a GUI-capable Python. If `import tkinter` fails (needed by the TkAgg backend), install it with `sudo apt install python3-tk`.

---

## Quick start

```bash
python main.py          # Linux / Windows
mjpython main.py        # macOS (required by the MuJoCo viewer)
```

Two windows open: the MuJoCo 3D viewer and a matplotlib dashboard. Close the **viewer** to stop the simulation; the plot window then stays open until you close it.

If the plot window misbehaves on macOS, add `matplotlib.use("TkAgg")` before `import matplotlib.pyplot`.

---

## What the simulation does

1. The robot starts with every joint at **0 rad**, base 0.55 m above the floor, and drops.
2. From t = 0 the PD target is the **standing pose** (thigh ±0.6 rad, knee ±1.2 rad, abduction 0). The initial error is large, so the torque saturates at the actuator limit and the legs move as fast as the model allows.
3. The controller then holds the pose.
4. The sim runs in **real time** (physics is stepped to catch up with the wall clock) and the plots update live.

The script prints the time at which all joints first come within 0.05 rad of their targets.

### Joint sign handling

Axis conventions differ between legs in the model, so the script finds the correct sign of each joint automatically: it nudges each thigh/knee joint and checks which way the calf moves, then picks signs that give *thigh forward, knee bent back* on every leg. "Forward" is taken as **−x** (the front hips sit at x = −0.1595).

---

## Robot model

### Bodies and masses

| Body | Mass |
|---|---|
| `base_link` | 10 kg (inertia computed from mesh convex hull) |
| each hip / thigh / calf (12 links) | 1.38 kg = 0.38 kg (AK60) + 1.0 kg |
| **Total** | **26.56 kg** |

Link centres of mass are unchanged from the original model; link inertias were scaled by `1.38 / original_mass`.

### Joints (per leg, 4 legs: `fr`, `fl`, `rr`, `rl`)

| Joint | Range |
|---|---|
| `hip_<leg>` (abduction) | ±0.8 rad |
| `hip_<leg>_2` (thigh) | ±1.57 rad |
| `knee_<leg>` | ±2.5 rad |

### AK60 actuator parameters (joint side)

Assumptions: 48 V bus, 6:1 internal gearbox, 5.8:1 external stage, 70 % efficiency applied to the external stage.

| Quantity | Derivation | Value |
|---|---|---|
| Total gear ratio | 6 × 5.8 | 34.8 |
| Peak torque | 9 Nm × 5.8 × 0.70 | **36.54 Nm** |
| Rated torque | 3 Nm × 5.8 × 0.70 | 12.18 Nm |
| Max (no-load) speed | 640 rpm ÷ 5.8 | **11.55 rad/s** (5.78 rad/s at 24 V) |
| Rated speed | 490 rpm ÷ 5.8 | 8.85 rad/s |
| Joint `armature` | 2.435e-5 kg·m² × 34.8² | 0.0295 kg·m² |
| Joint `frictionloss` | 0.2 Nm × 5.8 ÷ 0.70 | 1.66 Nm |
| Joint `damping` | estimate | 0.1 N·m·s/rad |

These are stored in the XML as defaults and in `custom/numeric` `ak60_joint_limits` (peak torque, rated torque, no-load speed, rated speed, total ratio).

### Actuators

Twelve `motor` actuators (direct torque, `ctrl` in **Nm**, clamped to ±36.54 Nm). The joint `actuatorfrcrange` is set to the same limit. `ctrl` order:

```
hip_fr, hip_fr_2, knee_fr,  hip_fl, hip_fl_2, knee_fl,
hip_rr, hip_rr_2, knee_rr,  hip_rl, hip_rl_2, knee_rl
```

The script maps joints to actuators through the transmission, so the order in the XML is not critical.

### Collision

All geoms are white. Only the **base** and the **calves** collide, and only with the floor (`contype=1`, `conaffinity=0` on those geoms; the floor is `1/1`). Hips and thighs are visual only. This avoids self-collision jitter from overlapping meshes at the zero pose.

### Sensors

Per joint: position (`q_*`), velocity (`dq_*`), actuator torque (`tau_*`). Plus an IMU at the base (`imu_acc`, `imu_gyro`, `imu_quat`).

### Simulation settings

Timestep 2 ms, `implicitfast` integrator.

---

## Controller

Runs in Python **every physics step**:

```
tau = KP·(q_des − q) + KD·(dq_des − dq) + KI·∫e dt + tau_ff
tau = clip(tau, −36.54, 36.54)
```

| Term | Details |
|---|---|
| `KP`, `KD` | 100 Nm/rad, 4 N·m·s/rad |
| `KI` | 30, integral clamped to ±10 Nm, frozen while a joint is saturated (anti-windup) |
| `tau_ff` | Holding torque from **inverse dynamics** with `qacc = 0` at the current pose (leg weight + contact forces carrying the body). Without this, a pure PD law only outputs torque proportional to error and the robot sags. |

Tunables are the constants at the top of `run_sim.py`:

| Constant | Meaning |
|---|---|
| `KP`, `KD`, `KI`, `I_MAX` | Controller gains |
| `USE_FF` | Enable/disable the inverse-dynamics feedforward |
| `STAND_ANGLE` | Thigh angle of the standing pose (knee = 2×) |
| `WINDOW_S`, `PLOT_HZ`, `DECIMATE` | Plot history length, refresh rate, sim-step decimation |
| `SETTLE_TOL` | Tolerance for the "target reached" message |

---

## Live dashboard

A 4 × 3 grid, one row per leg:

| Column | Content |
|---|---|
| Position [rad] | Actual joint angle, dashed line = target |
| Velocity [rad/s] | Joint velocity, black lines at ±11.55 rad/s |
| Torque command [Nm] | `ctrl` sent to the motors, black lines at ±36.54 Nm |

The window scrolls over the last 5 s.

---

## Using your own controller

Replace `pd_control()` in `run_sim.py`. It must write 12 torques (Nm) into `data.ctrl[ctrl_idx]` and return them (for logging). For a learned policy running slower than the physics, keep the PD loop at the sim rate and let the policy update `q_des`, `dq_des`, `tau_ff` and gains. An explicit PD at a slow rate with high `KD` will chatter (the stability limit is roughly `KD·Δt / J < 1` with `J ≈ 0.04 kg·m²`).

To get the AK60's MIT-mode behaviour inside MuJoCo instead, replace the `motor` actuators with `pid` actuators (`input="pos vel ff"`, `kp`, `kv`, `forcerange`); the control vector then has three entries per joint.

---

## Known limitations

- **No hard velocity limit.** MuJoCo has no joint speed limit, and the motors have no torque-speed curve; 11.55 rad/s is only a plotting/reference line. If you need it, derate torque with speed in your controller.
- **Efficiency** is applied as a constant 70 % derate of the external stage's torque; real efficiency varies with load and speed.
- **Torque constant:** 0.135 Nm/A is interpreted as the motor-side value (consistent with the 9 Nm peak at ~11 A through the 6:1 gearbox). No electrical model is simulated.
- **Joint axes** are copied from the original model; several are not mirror-symmetric between left and right legs. The sign probe compensates for the target pose, but your own controllers must account for it.
- **Frictionloss of 1.66 Nm** is a conservative back-drive estimate and creates a small dead-band; reduce it in the XML `<default><joint .../>` for freer joints.
- Feet have no dedicated geoms; the calf meshes are the contact surfaces.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `Could not find mesh` / file errors | `meshes/` folder missing or file names don't match (case-sensitive on Linux, e.g. `Thigh_fr.STL`) |
| Compile error on mesh inertia/`inertia` attribute | MuJoCo too old; upgrade or remove `inertia="convex"` |
| Base clips into floor or hovers high | Adjust `pos="0 0 0.55"` of `base_link` in the XML to your mesh dimensions |
| Legs fold the wrong way | Check the sign probe (forward assumed to be −x); swap signs in `target` |
| Robot sags or knee torque pins at ±36.54 Nm | Pose too crouched for the payload; lower `STAND_ANGLE`, or raise `KP`/`KI` |
| Chatter / vibration | Reduce `KD` or `KP`, or reduce the timestep |
| Viewer stutters | Lower `PLOT_HZ` or raise `DECIMATE` |
| macOS: viewer error | Use `mjpython`, not `python` |
