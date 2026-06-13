# GPS Outage Analysis

## Overview

This document analyzes how the GPS-IMU fusion system behaves when GPS measurements become unavailable and how performance recovers when GPS returns.

## The GPS Outage Problem

### Real-World Context

GPS signal denial occurs in:
- **Urban canyons**: Tall buildings block satellite signals
- **Tunnels**: Complete signal blockage
- **Dense forests**: Attenuation and multipath
- **Intentional jamming**: Military/security scenarios
- **Indoor navigation**: Buildings block signals

Modern navigation systems must operate gracefully during these outages.

### What Changes During Outage?

| Phase | GPS Available | GPS Denied |
|-------|---------------|-----------|
| **Measurement** | Position (x, y) | None |
| **EKF Correction** | Yes (Kalman gain active) | No (K = 0) |
| **EKF Prediction** | Yes | Yes (error grows) |
| **Covariance Evolution** | Decreases (corrections reduce uncertainty) | Increases (no measurements to bound error) |
| **Error Growth** | Bounded by GPS noise | Unbounded (IMU drift dominates) |

## Dead Reckoning Dynamics

### Pure IMU Integration

During the outage, if we use only IMU measurements without Kalman filtering:

$$\mathbf{x}_k = \mathbf{x}_{k-1} + \mathbf{f}(\mathbf{x}_{k-1}, \mathbf{u}_k) \Delta t$$

Where f is the kinematics and u = [a, r] are noisy IMU measurements.

### Error Sources in Dead Reckoning

1. **Acceleration Bias**
   - IMU bias affects velocity integration
   - Causes systematic velocity error: $\Delta v = b_a \cdot t_{outage}$
   - Position error grows quadratically: $\Delta x \propto b_a \cdot t_{outage}^2$

2. **Yaw Rate Bias**
   - Bias in yaw rate measurement: $b_r$
   - Causes heading drift: $\Delta \psi = b_r \cdot t_{outage}$
   - Couples to position error through velocity direction

3. **Measurement Noise**
   - High-frequency noise from IMU
   - Integrated to position
   - Effect is bounded (not growing linearly with time)

4. **Model Mismatch**
   - Kinematics assume ideal unicycle model
   - Real vehicles have tire slip, suspension effects, etc.
   - Minor effect for short outages (~20 s)

### Error Growth Rate

For acceleration bias-dominated error:

$$\text{Position Error} \approx \frac{1}{2} b_a t_{outage}^2$$

Example:
- Acceleration bias: 0.03 m/s² (realistic IMU)
- Outage duration: 20 s
- Expected position error: ~6 m

With velocity at ~4 m/s and heading changes during outage, actual error is often worse due to heading coupling.

## EKF Prediction-Only Mode

### Modified Update Step

During GPS outage, the Kalman gain becomes zero:

$$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}^T / (\mathbf{H} \mathbf{P}_k^- \mathbf{H}^T + \mathbf{R})$$

With no measurement (no z), the state and covariance become:

$$\hat{\mathbf{x}}_k = \mathbf{x}_k^- \quad \text{(prediction used as estimate)}$$
$$\mathbf{P}_k = \mathbf{P}_k^- \quad \text{(covariance not updated)}$$

The filter still propagates uncertainty (covariance growth) but cannot correct estimates.

### Advantage Over Pure Dead Reckoning

The EKF prediction step benefits from:

1. **Better initial state**: After GPS outage begins, x̂ is estimated with GPS corrections (better than pure integration)
2. **Process noise tuning**: EKF process noise Q accounts for model uncertainty
3. **Cross-state coupling**: Errors in one state (e.g., heading) don't directly corrupt other states (e.g., velocity)
4. **Covariance tracking**: Uncertainty grows in a principled way for later recovery

However, the EKF cannot correct bias errors without external measurements.

## Recovery Behavior

### Phase 1: GPS Re-acquisition (First 5–10 seconds after outage)

When GPS returns at t = 50 s:

1. **Large innovations occur**
   - GPS provides position that diverges from EKF prediction
   - Innovation: $\mathbf{y} = \mathbf{z} - \mathbf{H} \hat{\mathbf{x}}^-$ (large magnitude)

2. **Kalman gain is non-zero**
   - Covariance is large (outage drove uncertainty up)
   - Measurement noise is bounded
   - K blends prediction and measurement

3. **State and covariance converge**
   - Repeated GPS corrections drive state toward true trajectory
   - Covariance decreases as corrections continue

### Phase 2: Convergence (10–60 seconds after outage)

- GPS corrections are smaller (filter already near truth)
- Covariance decreases exponentially toward steady-state
- Filter behavior approximates normal GPS-IMU fusion

### Time to Convergence

Depends on:
- **Outage error magnitude**: Larger errors require more corrections
- **Kalman gain**: Low K (high uncertainty) → slow convergence
- **Measurement frequency**: Higher GPS rate → faster recovery
- **Process noise tuning**: Affects how quickly filter trusts GPS

Typical recovery time: **30–60 seconds** after outage ends.

## Covariance Evolution

### Before Outage

Covariance decreases asymptotically to steady-state value:

$$\text{P}_{\text{ss}} = \text{arg min}_P \; \left( F P F^T + G Q G^T - P H^T (H P H^T + R)^{-1} H P \right)$$

Steady-state uncertainty remains non-zero because:
- IMU measurements are noisy
- GPS measurements have bounded precision
- Filter cannot achieve zero uncertainty

### During Outage

No GPS measurement → no correction. Covariance grows:

$$\mathbf{P}_k = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{G} \mathbf{Q} \mathbf{G}^T$$

Growth rate depends on:
- **Process noise Q**: Larger Q → faster covariance growth
- **System dynamics F**: Nonlinear system amplifies uncertainty
- **Duration**: Quadratic growth (uncertainty spreads through prediction chain)

### After Outage

GPS correction resumes:

$$\mathbf{P}_k = (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K} \mathbf{H})^T + \mathbf{K} \mathbf{R} \mathbf{K}^T$$

Covariance decreases back to steady-state.

## IMU Bias Effects

### Slowly Varying Bias Model

In this project, IMU bias evolves as a random walk:

$$b_a(k+1) = b_a(k) + \eta_a(k)$$
$$\eta_a(k) \sim \mathcal{N}(0, q_b)$$

Where $q_b$ = `cfg.imuBiasRwAcc` is the bias random walk power spectral density.

### Why Bias is Problematic

1. **Constant offset**: Initial bias corrupts state estimation consistently
2. **Random walk**: Bias can slowly drift over time
3. **No GPS correction**: During outage, bias cannot be detected or corrected

### Practical Impact

With 0.03 m/s² acceleration bias:
- After 20 s outage: ~6 m position error
- Velocity error: 0.6 m/s persistent until GPS corrects it

The filter cannot distinguish between:
- True vehicle acceleration
- Biased IMU measurement

Only GPS can provide this information.

## Tuning for GPS-Denied Performance

### Key Parameters

1. **Process Noise (Q)**
   - Higher Q: Filter trusts IMU measurements less, relies on past estimates
   - During outage: More conservative, grows covariance faster
   - Effect: Slower divergence during outage, but also slower convergence after

2. **Measurement Noise (R)**
   - Higher R: Filter trusts GPS less
   - Effect: After recovery, convergence is slower
   - Not adjustable during outage (not adaptive)

3. **Initial Covariance (P0)**
   - High P0: Starts with high uncertainty
   - Effect: Filter takes longer to converge to GPS
   - Useful if prior estimate is unreliable

### Trade-offs

There is no universal optimal tuning because:
- **Pre-outage**: Want high Kalman gain (trust GPS, reduce noise)
- **During outage**: Want conservative Q (slow divergence)
- **Post-outage**: Want quick recovery (balance P and R)

Real systems use adaptive tuning or mode switching.

## Limitations of This Analysis

1. **Perfect initial state**: Filter initialized with ground truth
   - Real systems use noisy prior estimates
   - Recovery behavior worse if initial error is large

2. **Short outage duration**: 20 seconds is relatively short
   - Long-duration outages (hours) are more challenging
   - IMU bias effects become dominant

3. **Ideal kinematics**: No tire slip, suspension effects, or wind
   - Real vehicles add process noise
   - Affects error growth rate

4. **Fixed tuning**: Same Q, R throughout simulation
   - Adaptive filters adjust parameters based on innovation statistics
   - Can improve performance significantly

## Design Recommendations

For robust GPS-denied navigation systems:

1. **Monitor filter covariance**
   - High uncertainty indicates potential divergence
   - Use P as confidence metric for downstream applications

2. **Detect GPS loss reliably**
   - Sudden innovation changes indicate outage
   - Enable prediction-only mode explicitly

3. **Use complementary sensors**
   - Visual odometry: bounds position drift
   - Wheel encoders: improve velocity estimates
   - Magnetometer: aids heading during outage

4. **Implement adaptive filtering**
   - Adjust Q, R based on filter statistics
   - Faster recovery after outage return

5. **Plan for worst-case**
   - Design systems assuming worst-case IMU biases
   - Plan backup strategies (dead reckoning fallback)

---

**Next Steps**: See `ekf_behavior.md` for detailed EKF equations and filter mechanics.
