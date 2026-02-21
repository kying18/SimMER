# SimMER

**Simulated Minimalist Engine for Robotics**

SimMER is a tiny, educational robot arm simulation engine built from first principles. The goal is to make the core math of robotics (kinematics, Jacobians, inverse kinematics) transparent and hackable.

## What's Inside

- **`Joint`** — a revolute joint with an angle (the tunable parameter)
- **`Link`** — a rigid link with a fixed length
- **`Transform`** — a 3×3 homogeneous transformation matrix encoding rotation + translation in one shot
- **`Chain`** — a sequence of joints and links with forward kinematics via matrix chaining

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

Angles are relative (each joint angle is measured from the previous link's direction).

## Roadmap

- [x] Forward kinematics
- [ ] Jacobian computation
- [ ] Inverse kinematics (Jacobian pseudoinverse)
- [ ] Visualization (matplotlib)
- [ ] 3D extension (SE(3), 4×4 transforms)
- [ ] Simple dynamics
