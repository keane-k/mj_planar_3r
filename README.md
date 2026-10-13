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
