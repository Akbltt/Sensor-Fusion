# GPS-IMU-Fusion-EKF

Engineering-oriented MATLAB project for 2D ground-vehicle localization using GPS, IMU, and Extended Kalman Filter (EKF) fusion.

## Overview

This project simulates a realistic localization scenario where a vehicle moves through straight and curved segments with changing velocity. It compares:

- Ground Truth
- GPS-only localization
- IMU dead reckoning only
- GPS-IMU EKF fusion

The outputs are designed to clearly show GPS noise limitations, IMU drift accumulation, and EKF fusion benefits under mixed sensor rates and GPS outages.

## Problem Definition

State to estimate:

- x position
- y position
- velocity
- heading

Sensors:

- GPS: low-rate noisy position (x, y)
- IMU: high-rate acceleration and yaw rate with noise and optional bias drift

Fusion method:

- Nonlinear motion prediction with IMU
- Position correction with GPS
- EKF linearization through Jacobians
- Proper handling of asynchronous update rates

## Key Features

- Nonlinear 2D vehicle simulation with mixed maneuvers
- Modular EKF implementation (predict, Jacobians, update)
- Comparison against GPS-only and IMU-only baselines
- GPS outage experiment support
- Sensor noise and update-rate experiments
- Engineering plots and RMSE summaries
- Animation export to GIF and optional MP4
- Optional covariance ellipse and covariance evolution visualization

## Project Structure

- docs/ : deeper technical notes and tuning observations
- src/ : MATLAB source code for simulation and EKF
- results/ : generated figures, animations, and summary files
- run_project.m : end-to-end entry point

## How To Run

1. Open this folder in MATLAB.
2. Run `run_project` from the project root.
3. Review generated outputs under `results/`.

## Typical Outputs

- trajectory_comparison.png
- position_error.png
- velocity_heading_error.png
- covariance_evolution.png
- experiment_rmse_comparison.png
- fusion_animation.gif
- summary_metrics.txt
- experiment_summary.csv

## Notes

This project is intentionally structured as a foundation for future fusion work such as GPS-denied navigation, UWB-IMU fusion, multi-sensor systems, and Error-State Kalman Filter variants.
