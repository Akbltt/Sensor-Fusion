# System Model

## State

The EKF estimates:

- x position [m]
- y position [m]
- velocity [m/s]
- heading psi [rad]

State vector:

x_k = [x, y, v, psi]^T

## Motion Model

A nonlinear unicycle-like kinematic model with longitudinal acceleration a and yaw rate r:

- x_{k+1} = x_k + v_k cos(psi_k) dt + 0.5 a_k cos(psi_k) dt^2
- y_{k+1} = y_k + v_k sin(psi_k) dt + 0.5 a_k sin(psi_k) dt^2
- v_{k+1} = v_k + a_k dt
- psi_{k+1} = wrap(psi_k + r_k dt)

The model is nonlinear in heading and requires Jacobian linearization for EKF prediction.

## Assumptions

- Planar motion with no roll/pitch dynamics.
- IMU acceleration approximates longitudinal acceleration in vehicle frame.
- Yaw-rate-driven heading propagation is sufficient for this 2D study.
- GPS directly measures global position.
