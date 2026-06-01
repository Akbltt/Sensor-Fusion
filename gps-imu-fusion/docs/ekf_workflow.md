# EKF Workflow

## Prediction (IMU rate)

At every IMU sample:

1. Propagate state with nonlinear vehicle model using measured acceleration and yaw rate.
2. Compute Jacobians F (state) and G (input noise mapping).
3. Propagate covariance:

P_pred = F P F^T + G Q G^T

## Correction (GPS events only)

When GPS is available:

1. Innovation: y = z - H x_pred
2. Innovation covariance: S = H P_pred H^T + R
3. Kalman gain: K = P_pred H^T S^{-1}
4. State update: x = x_pred + K y
5. Covariance update (Joseph form for stability):

P = (I - K H) P_pred (I - K H)^T + K R K^T

GPS outage handling is naturally managed by skipping correction and running prediction-only until GPS resumes.

## Measurement Model

GPS position measurement:

z = H x + v

with

H = [1 0 0 0;
     0 1 0 0]
