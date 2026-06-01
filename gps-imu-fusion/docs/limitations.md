# Limitations

## Current Scope

- 2D planar kinematics only
- No slip dynamics or tire model
- IMU biases are simulated but not explicitly estimated in state
- No map constraints or lane priors
- No absolute heading sensor (e.g., magnetometer)

## Expected Behavior

- Long GPS outages lead to increased EKF drift due to prediction-only intervals.
- Aggressive maneuvers and model mismatch can increase heading and velocity errors.
- GPS spikes can transiently pull estimate if R is under-tuned.

## Extension Paths

- Add IMU bias states for joint estimation.
- Transition to Error-State Kalman Filter (ESKF).
- Add additional sensors (wheel odometry, UWB, lidar, vision).
- Extend to 3D navigation and full inertial mechanization.
