# Planar 3R Robot Arm: Forward Kinematics w/ MuJoCo
A personal reference for the planar 3R (three revolute joints) arm project. It contains the math, codes, instructions to run each script, what each MuJoCo call does, personalized notes and comments.

---

## What this project does
1. **Forward Kinematics (FK)**: given three joint angles, compute where each joint and the end-effector sit in the XY plane, using homogeneous transform matrices (`fk_3R.py`).
2. **Workspace scan**: sweep a grid of joint angles and scatter-plot every end-effector position the arm can reach (`workspace_3R.py`).
3. **MuJoCo w/ cross-check**: show the arm in a MuJoCo window and compare MuJoCo's own end-effector position against Python FK computation for the same pose (`mj_planar_3R_fk.py`).

---

## Setup
```bash
python -m venv .venv
# Windows: .venv\Scripts\activate     Mac/Linux: source .venv/bin/activate
pip install mujoco numpy matplotlib
```
Note: `pip install mujoco` also pulls `glfw` package, which `from mujoco.glfw import glfw` uses. No separate GLFW install in needed. 

Check your MuJoCo version, API differs between versions (see Troubleshooting):

```bash
pip show mujoco
```

---

## How to Run
| Goal | Command |
|---|---|
| Plot the reachable workspace | `python path/to/workspace_3R.py` |
| MuJoCo window + FK comparison | `python path/to/mj_planar_3R_fk.py` |

### Window Controls (`mj_planar_3R_fk.py`)
| Input | Action |
|---|---|
| Left-drag | Rotate camera |
| Right-drag | Pan camera; hold **Shift** for easier vertical pan |
| Scroll wheel | Zoom (scroll upwards = zoom in, downwards = zoom out) |
| Backspace | Reset the data (re-applies the same keyframe to the next frame, expect no visible changes) |

---

## About the Robot

Planar 3R arm: three revolute joints, all rotating about **z** (Right-Hand Rule), links along the **x**.

| Parameter | Value (m) |
|---|---|
| `l1` | 1 (frame 1 to frame 2) |
| `l2` | 1 (frame 2 to frame 3) | 
| `l3` | 0.25 (frame 3 to end-effector; no rotation of its own) |
| Max reach | `l1 + l2 + l3 = 2.25` |

---
