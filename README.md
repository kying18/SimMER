# SimMER

**Simulated Minimalist Engine for Robotics**

SimMER is a tiny, educational robot arm simulation engine built from first principles. The goal is to make the core math of robotics (kinematics, Jacobians, inverse kinematics) transparent and hackable.

If you can read ~300 lines of Python, you can understand the whole engine.

## What's Inside

- **`Joint`** — a revolute joint with an angle (the tunable parameter)
- **`Link`** — a rigid link with a fixed length
- **`Transform`** — a 3×3 homogeneous transformation matrix encoding rotation + translation in one shot
- **`Chain`** — a sequence of joints and links with forward kinematics, Jacobian, and IK solvers
- **`viz`** — matplotlib-based visualization: static snapshots and IK animation

## Core Idea

A robot arm is a chain of transforms. The end-effector position is:

```
T_total = T1 @ T2 @ ... @ Tn
EE = T_total @ [0, 0, 1]
```

Each `T_i` is a homogeneous transform for joint `i`:

```
T = [[cos θ, -sin θ, l·cos θ],
     [sin θ,  cos θ, l·sin θ],
     [0,      0,     1      ]]
```

Angles are relative — each joint angle is measured from the previous link's direction.

The Jacobian maps joint velocities to end-effector velocities. Column `i` is the perpendicular to the vector from joint `i` to the EE:

```
J[:,i] = [-(y_ee - y_i), (x_ee - x_i)]
```

## IK Solvers

Two iterative solvers are included:

- **Gradient descent** — `J^T @ e`, simple and robust
- **Damped pseudoinverse** — `J^T (J J^T + λ²I)^{-1} @ e`, faster convergence and stable near singularities

## Roadmap

- [x] Forward kinematics
- [x] Geometric Jacobian
- [x] Inverse kinematics (gradient descent)
- [x] Inverse kinematics (damped pseudoinverse)
- [x] Visualization (matplotlib)
- [ ] Joint limits
- [ ] Null space control (redundant arms)
- [ ] 3D extension (SE(3), 4×4 transforms)
- [ ] Simple dynamics
