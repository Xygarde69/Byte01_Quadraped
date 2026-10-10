"""
Real-time MuJoCo sim of the AK60 quadruped with a Python PD controller (torque control)
and a live, scrolling matplotlib dashboard.

The robot starts from all-zero joint angles, the PD target is the standing pose from
t = 0 (step -> torque saturates -> fastest possible motion), then it holds the pose.

Run:   python run_sim.py        (macOS: mjpython run_sim.py)
Close the MuJoCo viewer window to stop.
"""
import time
from collections import deque
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt

XML_PATH = str(Path(__file__).resolve().parents[2] / "Model" / "V2" / "V2.xml")

# ---- controller settings ---------------------------------------------------------------
KP = 10.0                # Nm/rad
KD = 10.0                  # Nm*s/rad
KI = 20.0                 # Nm/(rad*s), removes leftover steady-state error
I_MAX = 10.0              # Nm, anti-windup clamp on the integral torque
USE_FF = True             # hold-torque feedforward from inverse dynamics (qacc = 0)
TAU_MAX = 42           # Nm, AK60 peak at the joint (9 * 5.8 * 0.7)
STAND_ANGLE = -0.6         # thigh angle [rad]; knee uses 2x this

# ---- display settings ------------------------------------------------------------------
WINDOW_S = 5.0            # seconds of history shown
PLOT_HZ = 10              # plot refresh rate (wall clock)
DECIMATE = 5              # plot every Nth sim step
SETTLE_TOL = 0.05         # rad

LEGS = ["fr", "fl", "rr", "rl"]
JOINTS = [n for leg in LEGS for n in (f"hip_{leg}", f"thigh_{leg}", f"knee_{leg}")]
NJ = len(JOINTS)

model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)
assert model.nu == NJ, f"expected {NJ} motors, got {model.nu}"

# map joint -> ctrl index through the actuator transmission (order-independent)
jid = [model.joint(j).id for j in JOINTS]
ctrl_idx = np.array([int(np.where(model.actuator_trnid[:, 0] == i)[0][0]) for i in jid])
qadr = np.array([model.joint(j).qposadr[0] for j in JOINTS])
vadr = np.array([model.joint(j).dofadr[0] for j in JOINTS])
tau_lim = np.minimum(model.actuator_ctrlrange[ctrl_idx, 1], TAU_MAX)


def joint_sign_probe(leg):
    """Signs giving 'thigh forward, knee bent back' for this leg (forward = +y),
    found by nudging each joint and watching the calf move."""
    cid = model.body(f"calf_{leg}").id
    signs = []
    for jname, use_com, want_increase in ((f"thigh_{leg}", False, True),
                                          (f"knee_{leg}", True, False)):
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        x0 = (data.xipos if use_com else data.xpos)[cid][1]
        data.qpos[model.joint(jname).qposadr[0]] = 0.1
        mujoco.mj_forward(model, data)
        x1 = (data.xipos if use_com else data.xpos)[cid][1]
        signs.append(1.0 if ((x1 > x0) == want_increase) else -1.0)
    return signs


target = {}
for leg in LEGS:
    s_thigh, s_knee = joint_sign_probe(leg)
    target[f"hip_{leg}"] = 0.0
    target[f"thigh_{leg}"] = s_thigh * STAND_ANGLE
    target[f"knee_{leg}"] = s_knee * 2 * STAND_ANGLE
q_des = np.array([target[j] for j in JOINTS])
dq_des = np.zeros(NJ)
tau_ff = np.zeros(NJ)

# ---- initial state: zeros ----------------------------------------------------------------
mujoco.mj_resetData(model, data)
data.qpos[qadr] = 0.0
mujoco.mj_forward(model, data)


ff_data = mujoco.MjData(model)     # scratch data for the inverse-dynamics feedforward
integ = np.zeros(NJ)               # integral torque state [Nm]
dt = model.opt.timestep


def holding_torque():
    """Joint torques needed to keep the CURRENT configuration static (qacc = 0),
    including leg weight and the contact forces carrying the body."""
    ff_data.qpos[:] = data.qpos
    ff_data.qvel[:] = 0.0
    ff_data.qacc[:] = 0.0
    ff_data.ctrl[:] = 0.0
    mujoco.mj_inverse(model, ff_data)
    return ff_data.qfrc_inverse[vadr].copy()


def pd_control():
    global integ
    q = data.qpos[qadr]
    dq = data.qvel[vadr]
    ff = tau_ff + (holding_torque() if USE_FF else 0.0)
    tau_unsat = KP * (q_des - q) + KD * (dq_des - dq) + integ + ff
    tau = np.clip(tau_unsat, -tau_lim, tau_lim)
    # integrate only where the actuator is not saturated (anti-windup)
    free = np.abs(tau_unsat) < tau_lim
    integ = np.clip(integ + np.where(free, KI * (q_des - q) * dt, 0.0), -I_MAX, I_MAX)
    data.ctrl[ctrl_idx] = tau
    return tau


# ---- live plot ----------------------------------------------------------------------------
N = int(WINDOW_S / (model.opt.timestep * DECIMATE))
buf_t = deque(maxlen=N)
buf_q = deque(maxlen=N)
buf_dq = deque(maxlen=N)
buf_tau = deque(maxlen=N)

plt.ion()
fig, axs = plt.subplots(4, 3, sharex=True, figsize=(14, 9))
lines_q, lines_dq, lines_tau, lines_tgt = {}, {}, {}, {}
colors = ["tab:blue", "tab:orange", "tab:green"]
for r, leg in enumerate(LEGS):
    for k, name in enumerate((f"hip_{leg}", f"thigh_{leg}", f"knee_{leg}")):
        i = JOINTS.index(name)
        (lines_q[i],) = axs[r, 0].plot([], [], color=colors[k], label=name)
        (lines_tgt[i],) = axs[r, 0].plot([], [], "--", color=colors[k], lw=0.8)
        (lines_dq[i],) = axs[r, 1].plot([], [], color=colors[k], label=name)
        (lines_tau[i],) = axs[r, 2].plot([], [], color=colors[k], label=name)
    axs[r, 0].set_ylabel(leg.upper())
    axs[r, 0].set_ylim(-2.7, 2.7)
    axs[r, 1].set_ylim(-16, 16)
    axs[r, 2].set_ylim(-40, 40)
    axs[r, 1].axhline(11.55, color="k", lw=0.5)
    axs[r, 1].axhline(-11.55, color="k", lw=0.5)
    axs[r, 2].axhline(TAU_MAX, color="k", lw=0.5)
    axs[r, 2].axhline(-TAU_MAX, color="k", lw=0.5)
    for c in range(3):
        axs[r, c].grid(True)
    axs[r, 0].legend(fontsize=6, loc="upper right")
axs[0, 0].set_title("position [rad]  (dashed = target)")
axs[0, 1].set_title("velocity [rad/s]")
axs[0, 2].set_title("torque command = ctrl [Nm]")
for c in range(3):
    axs[3, c].set_xlabel("time [s]")
fig.tight_layout()
plt.show(block=False)


def refresh_plot():
    if not buf_t:
        return
    t = np.fromiter(buf_t, float)
    q = np.array(buf_q)
    dq = np.array(buf_dq)
    tau = np.array(buf_tau)
    for i in range(NJ):
        lines_q[i].set_data(t, q[:, i])
        lines_tgt[i].set_data(t[[0, -1]], [q_des[i], q_des[i]])
        lines_dq[i].set_data(t, dq[:, i])
        lines_tau[i].set_data(t, tau[:, i])
    axs[0, 0].set_xlim(max(0.0, t[-1] - WINDOW_S), max(WINDOW_S, t[-1]))
    fig.canvas.draw_idle()
    fig.canvas.flush_events()


# ---- real-time loop -----------------------------------------------------------------------
reported = False
step_count = 0
with mujoco.viewer.launch_passive(model, data) as viewer:
    wall0 = time.perf_counter()
    next_plot = 0.0
    while viewer.is_running():
        wall = time.perf_counter() - wall0
        # catch up to wall-clock time (capped so a slow frame can't snowball)
        target_time = min(wall, data.time + 0.1)
        while data.time < target_time:
            tau = pd_control()
            mujoco.mj_step(model, data)
            step_count += 1
            if step_count % DECIMATE == 0:
                buf_t.append(data.time)
                buf_q.append(data.qpos[qadr].copy())
                buf_dq.append(data.qvel[vadr].copy())
                buf_tau.append(tau.copy())

        if not reported and np.max(np.abs(data.qpos[qadr] - q_des)) < SETTLE_TOL:
            print(f"All joints within {SETTLE_TOL} rad of target at t = {data.time:.3f} s")
            reported = True

        viewer.sync()
        if wall >= next_plot:
            refresh_plot()
            next_plot = wall + 1.0 / PLOT_HZ
        time.sleep(0.001)

plt.ioff()
plt.show()
