# GPS-Denied Navigation

Engineering analysis of navigation system behavior during GPS outages using Extended Kalman Filter (EKF) fusion.

## Overview

This project investigates how a GPS-IMU fusion system behaves when GPS measurements become temporarily unavailable. It demonstrates:

- **Normal operation**: GPS-IMU EKF navigation with both sensors available
- **GPS outage**: IMU-only dead reckoning with error accumulation
- **EKF prediction**: How the filter behaves when GPS corrections stop
- **Recovery**: How quickly the system converges back to ground truth when GPS returns

**Key Question**: How does GPS denial affect navigation accuracy, and how effectively does EKF recover when the signal returns?

## Motivation

GPS is the foundation of modern navigation systems, but signal denial—whether from intentional jamming, urban canyons, or tunnels—creates critical system vulnerabilities. Understanding filter behavior during outages is essential for robust navigation design.

This project extends the [GPS-IMU-Fusion-EKF](../gps-imu-fusion/) framework to study GPS-denied conditions systematically.

## Scenario

A ground vehicle operates in a 2D environment with:

- **Initial phase**: GPS available, normal EKF fusion (0–30 s)
- **Outage phase**: GPS denied, IMU-only dead reckoning (30–50 s)
- **Recovery phase**: GPS returns, EKF re-converges (50–end)

## System

### State Estimation

State vector: `[x, y, v, ψ]` (position, velocity, heading)

### Sensors

- **GPS**: Low-rate (1 Hz), noisy position measurements
- **IMU**: High-rate (50 Hz), acceleration and yaw rate with noise and optional bias drift

### Filter

Extended Kalman Filter (EKF) with:
- Nonlinear motion prediction using IMU
- GPS position corrections (disabled during outage)
- Adaptive handling of asynchronous measurement rates

## Key Findings

See [results/](results/) for detailed analysis.

The analysis reveals:
- **Error growth rate** during GPS-denied periods
- **IMU bias impact** on long-duration outage performance
- **Recovery characteristics** when GPS returns
- **Covariance behavior** during transitions

## How to Run

**Option 1: From project root directory**
```matlab
cd gps-denied-navigation
run_project
```

**Option 2: From src directory**
```matlab
cd gps-denied-navigation/src
run_project
```

Either method works—the script automatically detects the project structure. Expected runtime: **~10–20 seconds**

## Outputs

### Plots

- **Trajectory comparison**: Ground truth, GPS measurements, IMU dead reckoning, EKF estimate
- **Position error**: With shaded outage interval highlighting
- **Velocity error**: Including recovery phase
- **Heading error**: Angular tracking performance
- **Covariance evolution**: Uncertainty growth and recovery

### Animation

GIF animation (`results/fusion_animation.gif`) showing:
- Vehicle trajectory
- GPS measurements
- EKF estimates
- IMU dead reckoning
- GPS outage interval (highlighted)

### Summary Metrics

CSV file with error statistics and key performance indicators.

## Documentation

| Document | Purpose |
|----------|---------|
| [navigation_scenario.md](docs/navigation_scenario.md) | Trajectory design and maneuver profile |
| [gps_outage_analysis.md](docs/gps_outage_analysis.md) | Outage mechanics, drift mechanisms, recovery behavior |
| [ekf_behavior.md](docs/ekf_behavior.md) | EKF equations, prediction vs. correction, tuning observations |
| [limitations.md](docs/limitations.md) | Assumptions, model limitations, future directions |

## Project Structure

```
gps-denied-navigation/
  LICENSE
  README.md
  run_project.m
  src/
    default_config.m              # Central configuration
    generate_ground_truth.m       # Trajectory generation
    simulate_sensors.m            # Sensor simulation
    ekf_fusion.m                  # EKF with outage handling
    imu_dead_reckoning.m          # Baseline comparison
    propagate_state.m             # Nonlinear motion model
    state_jacobian.m              # EKF linearization
    compute_errors.m              # Error analysis
    plot_results.m                # Engineering plots
    animate_results.m             # GIF/MP4 export
    wrap_to_pi.m                  # Angle normalization
  docs/
    navigation_scenario.md
    gps_outage_analysis.md
    ekf_behavior.md
    limitations.md
  results/
    fusion_trajectory.png
    position_error.png
    velocity_error.png
    heading_error.png
    covariance_evolution.png
    fusion_animation.gif
    summary_metrics.txt
    error_analysis.csv
```

## Design Philosophy

This project emphasizes:

- **Clarity**: Code is readable and modular; no unnecessary abstraction
- **Reuse**: Leverages proven components from GPS-IMU-Fusion-EKF
- **Engineering focus**: Plots and analysis answer concrete questions about system behavior
- **Professionalism**: Suitable for GitHub portfolio and team collaboration

## Next Steps

This project provides a foundation for:

- **UWB-IMU Fusion**: Adding ultra-wideband as GPS alternative
- **Multi-Sensor Fusion**: Incorporating additional sensors (LiDAR, visual odometry)
- **Error-State Kalman Filter**: Advanced ESKF formulation for larger-state systems
- **Adaptive Tuning**: Real-time filter parameter adjustment during outages
- **Robustness Analysis**: Quantifying system performance across outage durations

## References

- **EKF Theory**: Extended Kalman Filtering and Nonlinear Estimation (Welch & Bishop)
- **Vehicle Kinematics**: Unicycle model with steering-rate and acceleration inputs
- **Practical Navigation**: Inertial Navigation Systems, 2nd Ed. (Titterton & Weston)

---

**Author**: Navigation & Sensor Fusion Study  
**Status**: Active Development  
**Last Updated**: 2026
