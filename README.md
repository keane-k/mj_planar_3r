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

## What each script does

### `fk_3R.py`
- `forward_kinematics(angles, params)` takes `angles = [θ1, θ2, θ3]` (in rads) and `params = (l1, l2, l3)`.
- It returns a `SimpleNamespace` with `point1_xy`, `point2_xy`, `point3_xy`, `pointend_xy`. Each of which is a 2-element array `[x, y]`.
- `animate(coords)` draws the arm using `matplotlib` from the above coordinates.

### `workspace_3R.py`
- Joint ranges: θ1 in [0°, 90°], θ2 and θ3 in [−90°, 90°].
- `np.linspace(lower_limit, higher_limit, 30)` per joint, then a triple nested loop: 30 x 30 x 30 = **27,000** poses combination.
- Collects `pointend_xy` for every pose combination and draws a ***scatter*** plot (`plt.scatter`).

### `mj_planar_3R_fk.py`
1. Loads `planar_3R.xml`, creates `MjModel`, `MjData`, camera, visualization options.
2. Opens a GLFW window and installs mouse/keyboard callbacks.
3. Looks up the keyframe in `planar_3R.xml`: `key_qpos = model.key_qpos[model.key('home_pos').id]`.
4. Each frame:
    - `data.qpos = key_qpos.copy()` puts the arm in the specified home pose.
    - `mj.mj_forward(model, data)` recomputes positions.
    - Compares end_effector's pose computation between MuJoCo and Python Script
      - Reads MuJoCo's computation of end-effector pose: `data.site('end_effector').xpos`.
      - Runs `forward_kinematics(angles, params)` where `angles = data.qpos.copy()` and reads `coords.pointend_xy`.
      - Prints both. They should match.
5. Renders the scene.


