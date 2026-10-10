# Byte01 Quadruped MuJoCo Simulation

This project contains MuJoCo models and Python controllers for the Byte01 four-legged robot. The robot has four legs with three actuated joints per leg: hip abduction, thigh, and knee (12 actuators total).

## Repository layout

```text
Model/
├── V1/
│   ├── byte01.xml       # V1 MuJoCo model
│   └── meshes/           # V1 STL meshes
└── V2/
    ├── V2.xml           # V2 MuJoCo model
    ├── DOggo_assem.urdf  # Source/ reference URDF
    └── meshes/           # V2 STL meshes

scripts/
├── main.py              # Controller launcher
├── prev_versions/
│   ├── V1.py             # V1 controller and live dashboard
│   └── V2.py             # V2 controller and live dashboard
└── MUJOCO_LOG.TXT
```

## Requirements

- Python 3.9 or newer
- MuJoCo Python bindings
- NumPy
- Matplotlib

Install the dependencies in a virtual environment:

```bash
cd Byte01_Quadraped
python -m venv .venv
source .venv/bin/activate       # Linux/macOS
# .venv\Scripts\activate        # Windows
python -m pip install --upgrade pip
python -m pip install mujoco numpy matplotlib
```

The viewer and Matplotlib require a desktop/GUI session. On Linux, install Tk if Matplotlib cannot open a window:

```bash
sudo apt install python3-tk
```

## Running the simulation

Run the current V2 controller from the project root:

```bash
python scripts/main.py
```

Run a specific model/controller version:

```bash
python scripts/main.py v2   # default; loads Model/V2/V2.xml
python scripts/main.py v1   # loads Model/V1/byte01.xml
```

Any additional arguments are forwarded to the selected controller script. On macOS, use `mjpython` if the MuJoCo viewer requires it:

```bash
mjpython scripts/main.py v2
```

The simulation opens a MuJoCo viewer and a live Matplotlib dashboard. Close the MuJoCo viewer to end the simulation; the plot window can then be closed separately.

## How the controller works

Both controller versions:

1. Load their corresponding XML model.
2. Reset the robot to zero joint positions.
3. Compute a standing target pose.
4. Apply torque control at each physics step using PD control plus integral correction.
5. Optionally add inverse-dynamics feedforward torque to help hold the robot upright.
6. Display joint position, velocity, and commanded torque in a scrolling dashboard.

The controller automatically maps joints to actuators through each actuator transmission, so it does not depend on actuator ordering in the XML. It also probes joint directions at startup to account for model-specific axis signs.

## Controller settings

Tune the constants near the top of `scripts/prev_versions/V1.py` or `scripts/prev_versions/V2.py`:

- `KP`, `KD`, `KI`: position, velocity, and integral gains
- `I_MAX`: integral torque limit
- `USE_FF`: enable inverse-dynamics holding-torque feedforward
- `TAU_MAX`: controller torque limit
- `STAND_ANGLE`: target thigh angle; the knee target is twice this value
- `WINDOW_S`, `PLOT_HZ`, `DECIMATE`: dashboard history and update settings
- `SETTLE_TOL`: tolerance used by the settling message

V1 and V2 are separate model/controller revisions. V1 names the upper-leg joint `hip_<leg>_2`; V2 names it `thigh_<leg>`.

## Model details

The four legs are named `fr`, `fl`, `rr`, and `rl`. Each leg contains:

```text
hip_<leg>
thigh_<leg>   # V2
knee_<leg>
```

The V1 model uses `hip_<leg>_2` in place of `thigh_<leg>`. The XML files define the robot geometry, STL mesh assets, joints, actuators, contacts, and simulation settings. Mesh paths are relative to each XML file, so keep each model's `meshes/` directory in place.

## Troubleshooting

- **Mesh or XML file not found:** run the command from the project root and verify that the selected model's `meshes/` directory exists.
- **Wrong version loaded:** use `python scripts/main.py v1` or `v2`; the default is V2.
- **Viewer does not open:** use a GUI-capable Python session. On macOS, try `mjpython`.
- **Matplotlib backend errors:** install Tk (`python3-tk` on Debian/Ubuntu) or configure a backend supported by your desktop environment.
- **Robot moves in the wrong direction:** check the joint sign probe and target-angle convention in the selected controller.
- **Excessive vibration or saturation:** reduce `KP`/`KD`, lower the target pose, or adjust the torque limit and model parameters.

## Status

The repository contains two controller/model revisions. V2 is the default entry point; V1 is retained for comparison and regression testing. Validate mesh paths, contacts, and controller gains on the target machine before using the simulation for hardware decisions.
