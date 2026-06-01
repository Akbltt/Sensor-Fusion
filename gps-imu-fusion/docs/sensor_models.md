# Sensor Models

## GPS Model

Measurements:

z_gps = [x, y]^T + noise

Characteristics in this project:

- Low update rate (default 1 Hz)
- Additive white Gaussian position noise
- Optional outage windows (measurement missing)

GPS-only localization is formed by zero-order hold of last available fix, highlighting sparse update effects.

## IMU Model

Measurements:

- Longitudinal acceleration a_m
- Yaw rate r_m

Model:

- a_m = a_true + b_a + n_a
- r_m = r_true + b_r + n_r

with optional random-walk biases:

- b_a(k+1) = b_a(k) + w_ba
- b_r(k+1) = b_r(k) + w_br

Characteristics:

- High update rate (default 50 Hz)
- Measurement noise
- Optional bias drift to emulate dead-reckoning divergence
