# EKF Behavior During GPS Outage

## Overview

This document describes the mathematical foundation of the Extended Kalman Filter (EKF) used in this project, with emphasis on how it behaves during GPS-denied periods.

## System Model

### State Dynamics

Continuous time:
$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}) = \begin{bmatrix}
v \cos(\psi) \\
v \sin(\psi) \\
a \\
r
\end{bmatrix}$$

Discrete time (Euler integration):
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k) \Delta t$$

### Process Noise Model

The discrete time system with noise:
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k) \Delta t + \mathbf{G}(\mathbf{x}_k) \mathbf{w}_k$$

Where:
- $\mathbf{w}_k = [w_a, w_r]^T$ are process noise components (zero-mean, white)
- $\mathbf{G}$ is the process noise input matrix

The assumed noise affects acceleration and yaw rate:
$$\mathbf{u}_k = \mathbf{u}_{true,k} + \mathbf{w}_k$$

### Measurement Model

GPS provides position measurements:
$$\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k$$

Where:
$$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

And $\mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R})$ is measurement noise.

During GPS outage, no measurement is available, so the update step is skipped.

## EKF Equations

### Predict Step

**State propagation:**
$$\hat{\mathbf{x}}_k^- = \hat{\mathbf{x}}_{k-1} + \mathbf{f}(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1}) \Delta t$$

**Covariance propagation:**
$$\mathbf{P}_k^- = \mathbf{F}_{k-1} \mathbf{P}_{k-1} \mathbf{F}_{k-1}^T + \mathbf{G}_{k-1} \mathbf{Q} \mathbf{G}_{k-1}^T$$

Where:
- $\mathbf{F}$ is the state Jacobian: $\mathbf{F} = \frac{\partial \mathbf{f}}{\partial \mathbf{x}} \bigg|_{\hat{\mathbf{x}}_{k-1}}$
- $\mathbf{G}$ is the noise Jacobian: $\mathbf{G} = \frac{\partial \mathbf{f}}{\partial \mathbf{w}}$
- $\mathbf{Q}$ is the process noise covariance

### Update Step (When GPS Available)

**Innovation (measurement residual):**
$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_k^-$$

**Innovation covariance:**
$$\mathbf{S}_k = \mathbf{H} \mathbf{P}_k^- \mathbf{H}^T + \mathbf{R}$$

**Kalman gain:**
$$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}^T \mathbf{S}_k^{-1}$$

**State update:**
$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k \mathbf{y}_k$$

**Covariance update (Joseph form for numerical stability):**
$$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k \mathbf{R} \mathbf{K}_k^T$$

### No Update (GPS Outage)

When GPS is unavailable:
$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^-$$
$$\mathbf{P}_k = \mathbf{P}_k^-$$

The state and covariance from prediction are used directly. Covariance grows without bound (in theory) if outage is infinite.

## State Jacobian Derivation

The state Jacobian $\mathbf{F} = \partial \mathbf{f} / \partial \mathbf{x}$ at state $\mathbf{x} = [x, y, v, \psi]^T$:

$$\mathbf{F} = \begin{bmatrix}
1 & 0 & \cos(\psi) \Delta t & -v \sin(\psi) \Delta t - \frac{1}{2} a \sin(\psi) \Delta t^2 \\
0 & 1 & \sin(\psi) \Delta t & v \cos(\psi) \Delta t + \frac{1}{2} a \cos(\psi) \Delta t^2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

Key observations:
- **F(1,3)**: Position change due to velocity
- **F(1,4)** and **F(2,4)**: Heading couples to position (nonlinearity)
- **F(3,3)**: Velocity propagates independently
- **F(4,4)**: Heading propagates independently from other states

## Noise Jacobian

$$\mathbf{G} = \begin{bmatrix}
\frac{1}{2} \cos(\psi) \Delta t^2 & 0 \\
\frac{1}{2} \sin(\psi) \Delta t^2 & 0 \\
\Delta t & 0 \\
0 & \Delta t
\end{bmatrix}$$

Interpretation:
- **G(1:2, 1)**: Acceleration noise affects position quadratically
- **G(3, 1)**: Acceleration noise affects velocity linearly
- **G(4, 2)**: Yaw rate noise directly affects heading

## Covariance Behavior Analysis

### Steady-State GPS-IMU Fusion

Before outage, with GPS corrections, covariance reaches steady-state:

$$\mathbf{P}_{\infty} \text{ minimizes: } \lim_{k \to \infty} \mathbf{P}_k$$

The steady-state satisfies:
$$\mathbf{P}_{\infty} = \mathbf{F} \mathbf{P}_{\infty} \mathbf{F}^T + \mathbf{G} \mathbf{Q} \mathbf{G}^T - \mathbf{F} \mathbf{P}_{\infty} \mathbf{H}^T (\mathbf{H} \mathbf{P}_{\infty} \mathbf{H}^T + \mathbf{R})^{-1} \mathbf{H} \mathbf{P}_{\infty} \mathbf{F}^T$$

This is solved iteratively during filter operation.

### Covariance During Outage

Without GPS updates:
$$\mathbf{P}_k^- = \mathbf{F}_{k-1} \mathbf{P}_{k-1} \mathbf{F}_{k-1}^T + \mathbf{G}_{k-1} \mathbf{Q} \mathbf{G}_{k-1}^T$$

Growth analysis (simplified linear case):
- Eigenvalues of $\mathbf{F}$ near 1 → covariance growth is linear in time
- Process noise $\mathbf{Q}$ provides baseline growth term
- Without external measurements, uncertainty increases unbounded

### Covariance After Outage Recovery

When GPS measurements resume:
$$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_k^-$$

Since $\mathbf{K}_k > 0$:
- $(\mathbf{I} - \mathbf{K}_k \mathbf{H})$ reduces covariance
- Repeated corrections drive P back toward steady-state
- Convergence rate depends on $\mathbf{K}_k$ magnitude

## Filter Stability and Consistency

### Observability

The system is observable to GPS measurements:
- GPS directly measures [x, y]
- Velocity and heading are unobservable from GPS alone
- But IMU provides indirect observability through state coupling

### Filter Stability Conditions

For EKF stability:

1. **System is detectable**: Unobservable modes are stable
   - Velocity and heading are stable (propagate with F)
   
2. **Process noise Q is positive definite**: Uncertainty has bounded growth
   - Ensures covariance remains symmetric positive definite

3. **Filter is tuned properly**: Q and R are consistent with actual noise
   - Too-low Q: Filter diverges (overconfident in model)
   - Too-high Q: Filter noisy (underutilizes measurements)

### Consistency Check

Filter is consistent if actual errors fall within estimated uncertainty:

$$\| \hat{\mathbf{x}}_k - \mathbf{x}_{true,k} \| \lesssim \sqrt{\text{trace}(\mathbf{P}_k)}$$

This can be monitored using:
- Normalized innovation sequence: $\mathbf{y}_k^T \mathbf{S}_k^{-1} \mathbf{y}_k \sim \chi^2_2$ (when GPS available)
- Whiteness tests: Check if innovations are uncorrelated

## Tuning Parameters

### Process Noise Covariance Q

In this project:
$$\mathbf{Q} = \begin{bmatrix} q_a & 0 \\ 0 & q_r \end{bmatrix}$$

- `cfg.ekfQAcc = 0.18²`: Acceleration uncertainty
- `cfg.ekfQYawRate = (0.8°)²`: Yaw rate uncertainty

Interpretation:
- Represents model uncertainty in kinematics
- Prevents filter from being overconfident in motion model
- During outage, larger Q accelerates covariance growth

### Measurement Noise Covariance R

$$\mathbf{R} = \begin{bmatrix} \sigma_{gps}^2 & 0 \\ 0 & \sigma_{gps}^2 \end{bmatrix}$$

- `cfg.gpsSigmaXY = 2.5 m`: GPS position noise (1-sigma)

Interpretation:
- Matches actual GPS noise characteristics
- After outage, influences convergence speed
- High R → slow convergence, low R → fast convergence

### Initial Covariance P0

$$\mathbf{P}_0 = \text{diag}(4^2, 4^2, 1.5^2, (10°)^2)$$

Interpretation:
- Initial position uncertainty: 4 m
- Initial velocity uncertainty: 1.5 m/s
- Initial heading uncertainty: 10°
- Should reflect actual prior knowledge

## Linearization Effects

The EKF linearizes the nonlinear system at current state estimate:

$$\mathbf{f}(\mathbf{x}) \approx \mathbf{f}(\hat{\mathbf{x}}) + \mathbf{F} (\mathbf{x} - \hat{\mathbf{x}})$$

For small errors, this is accurate. For large errors:
- Linearization error increases
- EKF performance degrades
- Extended Kalman Filter may diverge

In this project:
- Errors during outage grow to ~10 m (navigation scale)
- Linearization error is secondary
- EKF approximation remains valid

## Computational Complexity

**Predict step**: O(n³) matrix multiplications where n=4 (state dimension)
- ~64 multiplications per step

**Update step**: O(n×m) where m=2 (measurement dimension)
- Inversion of 2×2 matrix S

**Computational load**: Negligible on modern computers
- Can run in real-time on embedded systems (50 Hz)

---

**Implementation Note**: All Jacobians are computed analytically (not numerically differentiated) for numerical stability.

---

**Next Steps**: See `limitations.md` for assumptions and future work directions.
