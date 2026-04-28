# Quadruped Trajectory Tracking using MPC, LQG and PMP

This project implements **trajectory tracking for a quadruped robot** in MuJoCo using **waypoint-based navigation** and multiple control strategies.

The system combines:

* Waypoint-based trajectory generation
* Model-based controllers (PMP, LQG, MPC)
* Gait scheduling and swing leg control

---

# Objective

To design and evaluate controllers capable of making a quadruped robot:

* Follow a trajectory defined by **waypoints**
* Maintain **stability during locomotion**
* Compare performance between control strategies

---

# Waypoint-Based Trajectory Implementation

The trajectory is defined as a sequence of 3D waypoints:

```python
waypoints = [
    [0.0, 0.0, h],
    [1.0, 0.0, h],
    [1.0, 1.0, h],
    [0.0, 1.0, h]
]
```

A custom class:

```python
WaypointTracker
```

generates a **reference state** based on the current robot position.

### How it works

* Computes error:
  [
  e = x_{target} - x
  ]

* Switches waypoint when:
  [
  ||e|| < threshold
  ]

* Generates:

  * Desired position (smoothed)
  * Desired velocity (directional)

This ensures **continuous and stable trajectory tracking**.

---

# Controller Integration

The system supports three controllers:

| Controller | Description                                            |
| ---------- | ------------------------------------------------------ |
| PMP        | Optimal control using Pontryagin’s principle           |
| LQG        | Linear Quadratic Gaussian (state estimation + control) |
| MPC        | Model Predictive Control with constraints              |

Each controller is integrated into the same pipeline:

```text
State → WaypointTracker → x_ref → Controller → GRFs → Torques → Robot
```

---

## Gait and Locomotion

A **trot gait** is implemented:

* Phase 1: FL + RR
* Phase 2: FR + RL

Swing legs:

* No contact force
* Controlled using PD
* Follow a sinusoidal trajectory

---

# Performance Evaluation

Performance is evaluated using:

### 1. Position Error

[
||x - x_{ref}||
]

### 2. Velocity Error

[
||v - v_{ref}||
]

### 3. Control Effort

[
||u|| = ||GRFs||
]

---

## Output

Each run generates:

* Position tracking
* Velocity tracking
* Orientation
* Control forces

Saved automatically in:

```text
results/
```

---

## Controller Comparison

Run:

```bash
python examples/run_mujoco.py --controller all
```

This produces:

* RMSE comparison
* Control effort comparison

Example output:

```text
Controller     RMSE     Mean ||u||
PMP            0.xxx     xxx
LQG            0.xxx     xxx
MPC            0.xxx     xxx
```

---

# Usage

Run a specific controller:

```bash
python examples/run_mujoco.py --controller lqg --robot-name mini_cheetah --disturbance none
```

Other options:

```bash
--controller lqg
--controller pmp
--controller all
```

Optional:

```bash
--teleop
--disturbance impulse
--no-render
```

---

# Key Implementation Details

### Waypoint Tracking

Implemented in:

```text
src/waypoint_tracker.py
```

* Smooth reference generation
* Direction-based velocity

---

### Control Loop

Located in:

```text
examples/run_mujoco.py
```

Includes:

* State estimation
* Contact scheduling
* MPC / LQG / PMP execution
* GRF to torque mapping

---

### Swing Control

* PD control in joint space
* Activated only when leg is not in contact
* Generates basic stepping motion

---

# Limitations and Observed Issues

Although a waypoint-based trajectory tracking framework was successfully implemented, **the robot did not achieve reliable trajectory following in practice**.

## Observed Behavior

During simulations:

* The robot tends to **prioritize balance over movement**
* In many cases, the legs **fail to lift properly**, resulting in dragging motion
* When forward velocity is increased, the robot **loses stability and falls**
* Trajectory tracking is inconsistent, especially during turns or waypoint transitions

---

## Root Cause Analysis

The main issue lies in the **decoupled control architecture**:

### 1. Conflict between MPC and Swing Control

* The **MPC controller** assumes all contact forces contribute to stability
* The **swing leg controller (PD)** tries to move legs independently

This creates a conflict:

```text
MPC → tries to keep feet on the ground (stability)
PD  → tries to lift the legs (motion)
```

Result:

* Swing legs are not able to properly detach from the ground
* Locomotion becomes inefficient or fails

---

### 2. Lack of Foot Position Control (No IK)

The system controls **joint angles**, not **foot positions in space**.

This means:

* No precise foot placement
* No control of step location
* Poor coordination between body motion and leg motion

---

### 3. Simplified Gait Model

The gait implementation:

* Uses a fixed timing pattern
* Does not adapt to robot dynamics
* Does not account for balance margins

---

### 4. No Dynamic Foot Placement Strategy

Modern quadruped locomotion requires:

* Predicting center of mass motion
* Adjusting foot placement accordingly

This is not implemented in the current system.

---

