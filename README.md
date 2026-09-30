## Overview
Inverse kinematics (IK) implementation on a 2D articulated arm, written in MATLAB. The goal is for the end effector of the arm to reach a random target position, different on every execution.

## How it works

**Forward kinematics**

* The arm is a hierarchy of joints defined by local offsets and rotations
* Each joint is computed as a 3x3 homogeneous transformation matrix, chained from its parent
* The end effector position is read from the last joint of the chain

**Jacobian matrix**

* Built numerically: each joint angle is perturbed by a small amount and the resulting change in end effector position is measured (finite differences)
* Gives a 2 x N matrix relating changes in joint angles to changes in end effector position
* Recomputed at every iteration, since it depends on the current pose

**Incremental solver**

* The pseudoinverse of the Jacobian (`pinv`) maps the position error to angle increments
* Angles are updated with a learning rate (`lr`) on each step instead of jumping directly to the solution
* The process repeats until the end effector is within a configurable error tolerance of the target
<p align = "center">
  <img width="530" height="532" alt="IK" src="https://github.com/user-attachments/assets/50d2b2b8-414b-4425-9805-496f7c650699" />
</p>
