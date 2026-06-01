# Tuning Analysis

## Main Tunables

- GPS measurement noise covariance R
- IMU process noise terms mapped through Q
- Initial covariance P0
- Sensor update rates (dtImu, dtGps)

## Observations

- Increasing GPS noise degrades correction quality but EKF still generally outperforms IMU-only dead reckoning over long horizons.
- Increasing IMU bias random walk rapidly increases dead-reckoning error; EKF contains growth when GPS updates are present.
- Slower GPS rate increases uncertainty growth between updates; EKF trajectory remains smoother than GPS-only hold behavior.
- During GPS outages, EKF behaves like constrained dead reckoning and covariance grows until GPS resumes.

## Practical Heuristic

- Start with realistic sensor sigmas.
- Adjust Q upward if EKF appears overconfident and slow to react.
- Adjust R upward if EKF overfits noisy GPS fixes.
- Use outage windows to validate robustness and uncertainty growth behavior.
