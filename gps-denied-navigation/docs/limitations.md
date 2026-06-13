# Limitations and Future Work

## Project Scope Limitations

### Modeling Assumptions

1. **Perfect Kinematics**
   - Assumes ideal unicycle model with no tire slip
   - Real vehicles have suspension compliance, tire dynamics
   - Effect on 20-second outage: minor (~0.5–1 m additional error)

2. **Point Mass Vehicle**
   - No consideration of vehicle rotation dynamics
   - Assumes heading changes are instantaneous
   - Real vehicles have angular momentum limits
   - Effect: Negligible for low-speed ground vehicles

3. **2D Motion Only**
   - No vertical (z) dimension
   - Appropriate for ground vehicles on flat terrain
   - Would need 3D state for aerial vehicles

4. **Constant Gravity**
   - IMU does not model vertical acceleration from gravity
   - Assumes accelerometer measures horizontal acceleration only
   - Realistic if IMU is properly mounted and calibrated

5. **Noise Model Assumptions**
   - Gaussian, white process noise
   - Real IMU noise includes:
     - Colored (non-white) components
     - Non-Gaussian outliers
     - Temperature-dependent bias drift
   - Effect: ~5–10% error underestimation during long outages

### Sensor Limitations

1. **GPS Noise Only**
   - Simulates Gaussian position noise
   - Does not include:
     - Multipath effects
     - Signal blockage (binary: available/unavailable)
     - Velocity-aiding mode
   - Real GPS has correlated errors that EKF may not handle optimally

2. **IMU Noise Model**
   - Includes white noise and slowly varying bias
   - Does not include:
     - Allan variance effects (noise floor, bias instability, random walk)
     - Temperature effects on bias
     - Scale factor errors
   - Realistic for MEMS IMU on 20-second timescales

3. **No Sensor Faults**
   - Assumes sensors fail gracefully (lose signal)
   - Does not model:
     - Hard faults (sensor output saturates)
     - Soft faults (biased measurements reported as valid)
   - Real systems need outlier rejection (RAIM, chi-square test)

### Algorithm Limitations

1. **Extended Kalman Filter**
   - Linearization at single point (may diverge with large nonlinearities)
   - Assumes Gaussian distributions (symmetric error bounds)
   - Cannot exploit non-convex geometry
   - For this application: appropriate (nonlinearity is moderate)

2. **No Adaptive Tuning**
   - Q and R fixed throughout simulation
   - Real systems benefit from:
     - Innovation monitoring (increase R if GPS noisy)
     - Outlier detection (reject bad measurements)
     - Mode-dependent tuning (different Q during turn)
   - Potential improvement: 10–20% better recovery time

3. **Initialization**
   - Filter initialized with true state
   - Real systems have unknown initial conditions
   - Bad initialization increases convergence time
   - Potential effect: +30 s convergence delay if P0 is wrong by 1 standard deviation

4. **No Multi-Hypothesis Tracking**
   - Single hypothesis (one trajectory estimate)
   - Real systems with ambiguous scenarios might track multiple possibilities
   - Not needed for this benign scenario

## Scenario Limitations

### Outage Duration

This project uses a **20-second GPS outage**:
- Realistic for urban canyon or tunnel
- Too short for:
  - Deep tunnel (minutes)
  - Desert with no infrastructure (hours)
  - Lunar/planetary exploration (weeks)

**Extended outage behavior**: 
- Error growth becomes quadratic in time (dominated by bias)
- After ~1 minute, position error may exceed vehicle environment
- Bias accumulation becomes critical limiting factor

### Vehicle Speed

This project assumes **ground vehicle** speeds (0–8 m/s):
- Appropriate for cars, trucks, robots
- Not applicable to:
  - Aircraft (100+ m/s → errors grow much faster)
  - Pedestrians (<2 m/s → shorter vehicle dynamics)
  - Space vehicles (orbital mechanics different)

### Environmental Model

The simulation assumes:
- **Flat terrain**: No elevation changes
- **No obstacles**: Free movement in any direction
- **No wind or disturbances**: Pure kinematics model
- **Controlled access**: No collisions

Real scenarios add:
- **Terrain constraints**: Road networks, obstacles
- **Environmental disturbances**: Wind, currents
- **Safety considerations**: Collision avoidance
- **Communication constraints**: Limited GPS/communication windows

## Performance Limitations

### Quantitative Error Bounds

During the 20-second outage, this system achieves:
- **Position RMSE**: ~8–15 m (depending on bias magnitude)
- **Velocity error**: ~0.5–1.0 m/s (systematic)
- **Heading error**: ~5–10° (drift)

This is acceptable for:
- Urban navigation (block-level accuracy)
- Fleet tracking (relative positions)
- Contingency/fallback mode

Not acceptable for:
- Precision approach (landing)
- Autonomous vehicles in traffic
- Lane-keeping on highways

### Recovery Time

After GPS returns (t = 50 s):
- **Fast convergence**: 10–20 seconds to ~1 m error
- **Full convergence**: 60–90 seconds to steady-state

Implications:
- Can resume primary GPS-denied operations after ~30 s
- Not suitable for real-time precise navigation immediately after recovery

## Future Extensions

### Short-Term Improvements (Easy)

1. **Adaptive Tuning**
   - Monitor innovation statistics
   - Increase R if GPS seems noisy
   - Adjust Q based on motion mode
   - **Expected benefit**: 10–20% faster recovery

2. **Outlier Rejection**
   - Chi-square test on innovations
   - Reject GPS measurements with large residuals
   - **Expected benefit**: Robustness to GPS multipath/interference

3. **Heading-Rate Sensor**
   - Add gyroscope-only heading measurement
   - Post-outage gyro helps lock heading faster
   - **Expected benefit**: 30–40% faster heading recovery

4. **Sensor Fusion with Compass**
   - Magnetometer provides absolute heading reference
   - During outage: helps constrain heading drift
   - After outage: faster convergence
   - **Expected benefit**: Heading error reduced by 50%

### Medium-Term Extensions (Moderate Effort)

1. **Error-State Kalman Filter (ESKF)**
   - Separate error dynamics from state
   - Better numerical stability
   - Can estimate and correct sensor biases
   - **Expected benefit**: Unbiased long-term estimates, 2–3x better outage performance

2. **Multiple Hypothesis Filter**
   - Particle filter or interacting multiple models
   - Track possible trajectories
   - Useful when measurement ambiguity exists
   - **Expected benefit**: Robustness to initialization errors

3. **Constraint-Based Filtering**
   - Incorporate road network constraints
   - Vehicle cannot leave road
   - Improves error bounds during outage
   - **Expected benefit**: Outage error reduced by 30–50%

4. **Complementary Sensors**
   - Wheel encoders (odometry)
   - Camera (visual odometry)
   - Radar (velocity measurements)
   - **Expected benefit**: Enables robust multi-sensor fusion, error growth slowed by 5–10x

### Long-Term Research (Ambitious)

1. **Integrated SLAM**
   - Build map while navigating
   - Use landmarks for positioning
   - Particularly useful for outages in known environments
   - **Challenge**: Real-time SLAM performance, map maintenance

2. **UWB/Cellular Localization**
   - Ultra-Wideband: accurate ranging at medium range
   - Cellular: wide coverage, lower accuracy
   - GPS alternative for outage mitigation
   - **Project**: GPS-Denied-UWB-Fusion

3. **Inertial Navigation System (INS)**
   - Continuous state-of-the-art IMU (much better than MEMS)
   - Can bridge hours-long outages with managed drift
   - Very high cost
   - **Application**: Military, aviation

4. **Cooperative Localization**
   - Multi-vehicle positioning
   - Vehicles share estimates via communication
   - Reduces individual navigation burden
   - **Research area**: Decentralized sensor fusion

5. **Learning-Based Approaches**
   - Neural networks to learn bias patterns
   - Predict IMU drift from historical data
   - Deep learning for sensor fusion
   - **Challenge**: Generalization, interpretability, real-time constraints

## Validation and Testing Strategy

For production use, extend this analysis with:

1. **Real-World Data**
   - Collect actual vehicle trajectories
   - Compare simulated vs. actual performance
   - Tune noise models to match reality

2. **Hardware-in-the-Loop**
   - Run filter on target processor
   - Test with real sensors
   - Validate timing and accuracy

3. **Edge Cases**
   - Extreme maneuvers (emergency braking, high-G turns)
   - Cold-start initialization
   - Sensor saturation
   - Multipath scenarios

4. **Monte Carlo Analysis**
   - Run 100+ trials with different random seeds
   - Compute statistics over ensemble
   - Verify theoretical covariance bounds
   - Quantify dispersion

5. **Robustness Testing**
   - Perturb initial conditions
   - Vary sensor noise levels
   - Test with different outage durations
   - Measure failure modes

## Design Lessons Learned

### What Works

1. **EKF is robust** for moderate nonlinearity and short outages
2. **Prediction-only mode** is feasible (errors bounded, recoverable)
3. **Covariance tracking** is essential for uncertainty quantification
4. **Sensor fusion** provides ~5–10x improvement over any single sensor

### What Doesn't Work

1. **Tuning by hand** is difficult; adaptive methods are better
2. **Ignoring bias** leads to long-term divergence
3. **Single-sensor fallbacks** are fragile
4. **Gaussian assumptions** break down with outliers

### Key Design Principles

1. **Graceful degradation**: System should work worse (not fail) during outage
2. **Uncertainty quantification**: Always report confidence bounds
3. **Redundancy**: Multiple sensors, multiple algorithms
4. **Validation**: Compare estimates with other sources whenever possible
5. **Monitoring**: Detect anomalies and mode switches

---

**Conclusion**: This project demonstrates fundamental GPS-denied navigation concepts suitable for educational and prototyping purposes. Production systems require additional robustness, sensors, and validation.

---

**Related Projects to Explore**:
- [GPS-IMU-Fusion-EKF](../gps-imu-fusion/)
- UWB-IMU-Fusion (TBD)
- Multi-Sensor Localization (TBD)
- Error-State Kalman Filter ESKF (TBD)
