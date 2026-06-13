# Navigation Scenario

## Overview

This document describes the 2D ground-vehicle navigation scenario designed to demonstrate GPS-denied navigation system behavior.

## Vehicle Kinematics

### State Vector

The vehicle state consists of four components:

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ v \\ \psi \end{bmatrix}$$

Where:
- **x, y**: Cartesian position (meters)
- **v**: Longitudinal velocity (m/s)
- **ψ**: Heading angle (radians, absolute orientation)

### Motion Model

The vehicle follows nonlinear unicycle-like kinematics with two control inputs:

$$\dot{x} = v \cos(\psi)$$
$$\dot{y} = v \sin(\psi)$$
$$\dot{v} = a$$
$$\dot{\psi} = r$$

Where:
- **a**: Longitudinal acceleration (m/s²)
- **r**: Yaw rate (rad/s)

This model captures:
- Position changes due to velocity and heading
- Velocity changes due to acceleration
- Heading changes due to yaw rate
- Coupling between velocity and heading

### Discrete-Time Propagation

At each IMU update (Δt = 0.02 s), the state is propagated:

$$x_{k+1} = x_k + v_k \cos(\psi_k) \Delta t + \frac{1}{2} a_k \cos(\psi_k) \Delta t^2$$
$$y_{k+1} = y_k + v_k \sin(\psi_k) \Delta t + \frac{1}{2} a_k \sin(\psi_k) \Delta t^2$$
$$v_{k+1} = v_k + a_k \Delta t$$
$$\psi_{k+1} = \psi_k + r_k \Delta t$$

## Trajectory Profile

The 140-second simulation includes a carefully designed maneuver profile:

### Phase 1: Acceleration (0–20 s)
- **a = 0.08 m/s²**: Vehicle accelerates from initial 4 m/s
- **r = 0 rad/s**: Straight-line motion
- **Purpose**: Establish baseline performance under GPS availability

### Phase 2: Constant Velocity (20–35 s)
- **a = 0 m/s²**: Constant speed cruise
- **r = 0 rad/s**: Straight motion
- **Purpose**: Steady-state operation before outage

### Phase 3: Deceleration with Turn (35–55 s)
- **a = -0.06 m/s²**: Vehicle decelerates
- **r = 2.6 deg/s**: Left turn (overlaps with GPS outage period)
- **Purpose**: Demonstrates challenging maneuver during partial outage

### Phase 4: Acceleration with Opposite Turn (55–75 s)
- **a = 0.05 m/s²**: Re-acceleration after outage
- **r = -2.0 rad/s**: Right turn (immediate post-outage)
- **Purpose**: Tests filter recovery during active maneuvering

### Phase 5: Cruising (75–95 s)
- **a = 0 m/s²**: Constant velocity
- **r = 0 rad/s**: Straight motion
- **Purpose**: Filter convergence period after recovery

### Phase 6: Gentle Deceleration (95–115 s)
- **a = -0.03 m/s²**: Slow deceleration
- **r = 0 rad/s**: Straight motion
- **Purpose**: Final approach phase

### Phase 7: Final Acceleration (115–140 s)
- **a = 0.02 m/s²**: Gentle acceleration
- **r = 1.6 deg/s**: Gentle left turn
- **Purpose**: Extended steady-state observation

## GPS Outage Timing

The outage interval is strategically placed:

- **[0, 30) s**: GPS available – filter initialization and pre-outage baseline
- **[30, 50) s**: GPS denied – error growth with deceleration and turn
- **[50, 140] s**: GPS available – recovery and long-term convergence

Duration of outage: **20 seconds**

This duration is long enough to:
- Show significant IMU drift accumulation
- Demonstrate filter prediction-only behavior
- Test recovery effectiveness

But short enough that the vehicle remains in a reasonable geographic area for visualization.

## Key Design Decisions

### Mixed Maneuvers

The trajectory includes:
- **Straight segments**: Test basic dead reckoning accuracy
- **Turns**: Challenge heading estimation and nonlinear model
- **Acceleration/deceleration**: Vary velocity to expose acceleration bias effects

### Outage During Active Maneuver

The GPS outage intentionally overlaps with:
- A turn (r = 2.6 deg/s from t=35 to t=45)
- A deceleration (a = -0.06 from t=35 to t=55)

This tests the filter's ability to maintain heading and velocity estimates without GPS corrections.

### Immediate Post-Outage Challenge

Upon GPS recovery (t=50 s), the vehicle enters a right turn (r = -2.0 rad/s) that immediately challenges filter lock-on with a maneuver.

## Initial Conditions

```matlab
x0 = [0, 0, 4, 5°]
```

- Starting position: (0, 0)
- Initial velocity: 4 m/s
- Initial heading: 5° (northeast)

The small initial heading provides visible non-zero trajectory curvature.

## Numerical Integration

All state propagation uses explicit first-order Euler integration:

```matlab
x_next = x_current + f(x_current, u, dt) * dt
```

Where:
- f: State rate equations (kinematics)
- u: Input vector [a, r]
- dt: 0.02 s (IMU period)

This is appropriate for:
- Short integration steps (0.02 s)
- Stable nonlinear system
- Sufficient accuracy for navigation analysis

## Reproducibility

Random seed: `cfg.rngSeed = 9`

All noise sources use this seed for reproducible sensor measurements.

## Visualization Interpretation

When viewing the trajectory plots:

- **Ground Truth (black)**: Actual vehicle path (noise-free kinematics)
- **EKF Solution (blue)**: Filtered estimate (GPS + IMU fusion)
- **IMU Dead Reckoning (red dashed)**: Accumulating drift (no GPS corrections)
- **GPS Measurements (green triangles)**: Noisy position observations

The divergence between EKF and dead reckoning during the outage interval illustrates the value of EKF prediction-only mode versus pure integration.

---

**Next Steps**: See `gps_outage_analysis.md` for detailed outage mechanics and recovery behavior.
