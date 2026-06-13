function plot_results(truth, sensors, dr, ekf, err, cfg)
%PLOT_RESULTS Create engineering-oriented visualizations with GPS outage highlighting.
%
% Plots:
% 1. Trajectory comparison (ground truth, GPS meas, DR, EKF)
% 2. Position error with outage interval
% 3. Velocity error with outage interval
% 4. Heading error with outage interval
% 5. Covariance evolution (uncertainty growth/recovery)

t = truth.t;
gpsOutageStart = sensors.gpsOutageStart;
gpsOutageEnd = sensors.gpsOutageEnd;

%% Figure 1: Trajectory Comparison
fig1 = figure('Color', 'w', 'Position', [100 100 1200 600]);

plot(truth.state(:, 1), truth.state(:, 2), 'k-', 'LineWidth', 2.0, 'DisplayName', 'Ground Truth');
hold on;

plot(ekf.state(:, 1), ekf.state(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF Solution');
plot(dr.state(:, 1), dr.state(:, 2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'IMU Dead Reckoning');

% GPS measurements
gpsIdx = round(sensors.tGps / cfg.dtImu) + 1;
gpsIdx = max(1, min(numel(t), gpsIdx));
scatter(sensors.gps(:, 1), sensors.gps(:, 2), 50, 'g^', 'filled', 'DisplayName', 'GPS Measurements');

grid on;
xlabel('X Position [m]', 'FontSize', 12);
ylabel('Y Position [m]', 'FontSize', 12);
title('2D Trajectory Comparison', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
axis equal;

saveas(fig1, fullfile(cfg.resultsDir, 'fusion_trajectory.png'));
close(fig1);

%% Figure 2: Position Error with Outage Interval
fig2 = figure('Color', 'w', 'Position', [100 100 1200 500]);

% Shade outage region
ax = gca;
hold on;
xline(gpsOutageStart, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
xline(gpsOutageEnd, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
patch([gpsOutageStart, gpsOutageEnd, gpsOutageEnd, gpsOutageStart], ...
      [0, 0, max(err.ekfPos)*1.1, max(err.ekfPos)*1.1], ...
      [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'DisplayName', 'GPS Outage');

plot(t, err.drPos, 'r--', 'LineWidth', 1.5, 'DisplayName', 'IMU Dead Reckoning');
plot(t, err.ekfPos, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF Solution');

grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Position Error [m]', 'FontSize', 12);
title('Position Error Over Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
xlim([0, max(t)]);
ylim([0, max(err.ekfPos)*1.1]);

saveas(fig2, fullfile(cfg.resultsDir, 'position_error.png'));
close(fig2);

%% Figure 3: Velocity Error with Outage Interval
fig3 = figure('Color', 'w', 'Position', [100 100 1200 500]);

hold on;
xline(gpsOutageStart, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
xline(gpsOutageEnd, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
patch([gpsOutageStart, gpsOutageEnd, gpsOutageEnd, gpsOutageStart], ...
      [0, 0, max(err.ekfVel)*1.1, max(err.ekfVel)*1.1], ...
      [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

plot(t, err.drVel, 'r--', 'LineWidth', 1.5, 'DisplayName', 'IMU Dead Reckoning');
plot(t, err.ekfVel, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF Solution');

grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Velocity Error [m/s]', 'FontSize', 12);
title('Velocity Error Over Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
xlim([0, max(t)]);

saveas(fig3, fullfile(cfg.resultsDir, 'velocity_error.png'));
close(fig3);

%% Figure 4: Heading Error with Outage Interval
fig4 = figure('Color', 'w', 'Position', [100 100 1200 500]);

hold on;
xline(gpsOutageStart, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
xline(gpsOutageEnd, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
patch([gpsOutageStart, gpsOutageEnd, gpsOutageEnd, gpsOutageStart], ...
      [0, 0, max(err.ekfHdg)*1.1, max(err.ekfHdg)*1.1], ...
      [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

plot(t, rad2deg(err.drHdg), 'r--', 'LineWidth', 1.5, 'DisplayName', 'IMU Dead Reckoning');
plot(t, rad2deg(err.ekfHdg), 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF Solution');

grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Heading Error [deg]', 'FontSize', 12);
title('Heading Error Over Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
xlim([0, max(t)]);

saveas(fig4, fullfile(cfg.resultsDir, 'heading_error.png'));
close(fig4);

%% Figure 5: Covariance Evolution (Uncertainty)
fig5 = figure('Color', 'w', 'Position', [100 100 1200 600]);

% Extract position uncertainty (1-sigma) from covariance
sigmaX = squeeze(sqrt(ekf.P(1, 1, :)));
sigmaY = squeeze(sqrt(ekf.P(2, 2, :)));
sigmaV = squeeze(sqrt(ekf.P(3, 3, :)));
sigmaPsi = squeeze(sqrt(ekf.P(4, 4, :)));

subplot(2, 2, 1);
hold on;
xline(gpsOutageStart, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
xline(gpsOutageEnd, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
patch([gpsOutageStart, gpsOutageEnd, gpsOutageEnd, gpsOutageStart], ...
      [0, 0, max(sigmaX)*1.1, max(sigmaX)*1.1], ...
      [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(t, sigmaX, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('σ_x [m]');
title('X Position Uncertainty');
xlim([0, max(t)]);

subplot(2, 2, 2);
hold on;
xline(gpsOutageStart, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
xline(gpsOutageEnd, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
patch([gpsOutageStart, gpsOutageEnd, gpsOutageEnd, gpsOutageStart], ...
      [0, 0, max(sigmaY)*1.1, max(sigmaY)*1.1], ...
      [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(t, sigmaY, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('σ_y [m]');
title('Y Position Uncertainty');
xlim([0, max(t)]);

subplot(2, 2, 3);
hold on;
xline(gpsOutageStart, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
xline(gpsOutageEnd, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
patch([gpsOutageStart, gpsOutageEnd, gpsOutageEnd, gpsOutageStart], ...
      [0, 0, max(sigmaV)*1.1, max(sigmaV)*1.1], ...
      [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(t, sigmaV, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('σ_v [m/s]');
title('Velocity Uncertainty');
xlim([0, max(t)]);

subplot(2, 2, 4);
hold on;
xline(gpsOutageStart, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
xline(gpsOutageEnd, 'k--', 'LineWidth', 1.0, 'Alpha', 0.5);
patch([gpsOutageStart, gpsOutageEnd, gpsOutageEnd, gpsOutageStart], ...
      [0, 0, rad2deg(max(sigmaPsi))*1.1, rad2deg(max(sigmaPsi))*1.1], ...
      [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(t, rad2deg(sigmaPsi), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('σ_ψ [deg]');
title('Heading Uncertainty');
xlim([0, max(t)]);

sgtitle('Filter Covariance Evolution', 'FontSize', 14, 'FontWeight', 'bold');

saveas(fig5, fullfile(cfg.resultsDir, 'covariance_evolution.png'));
close(fig5);

%% Save Summary Metrics
write_summary_metrics(truth, sensors, dr, ekf, err, cfg);

end

function write_summary_metrics(truth, sensors, dr, ekf, err, cfg)
%WRITE_SUMMARY_METRICS Save performance metrics to text file.

t = truth.t;
gpsOutageStart = sensors.gpsOutageStart;
gpsOutageEnd = sensors.gpsOutageEnd;

% Compute RMS errors over different phases
idxBefore = t < gpsOutageStart;
idxOutage = t >= gpsOutageStart & t < gpsOutageEnd;
idxAfter = t >= gpsOutageEnd;

rmseDrBefore = rms(err.drPos(idxBefore));
rmseEkfBefore = rms(err.ekfPos(idxBefore));

rmseDrOutage = rms(err.drPos(idxOutage));
rmseEkfOutage = rms(err.ekfPos(idxOutage));

rmseDrAfter = rms(err.drPos(idxAfter));
rmseEkfAfter = rms(err.ekfPos(idxAfter));

fid = fopen(fullfile(cfg.resultsDir, 'summary_metrics.txt'), 'w');

fprintf(fid, '================================================================================\n');
fprintf(fid, 'GPS-DENIED NAVIGATION: PERFORMANCE SUMMARY\n');
fprintf(fid, '================================================================================\n\n');

fprintf(fid, 'Configuration:\n');
fprintf(fid, '  Total Time: %.1f s\n', max(t));
fprintf(fid, '  GPS Outage: [%.1f, %.1f) s (duration: %.1f s)\n', ...
    gpsOutageStart, gpsOutageEnd, gpsOutageEnd - gpsOutageStart);
fprintf(fid, '  GPS Sigma: %.2f m\n', cfg.gpsSigmaXY);
fprintf(fid, '  IMU Sigma (accel): %.3f m/s^2\n', cfg.imuSigmaAcc);
fprintf(fid, '  IMU Sigma (yaw): %.3f deg/s\n\n', rad2deg(cfg.imuSigmaYawRate));

fprintf(fid, 'Phase-Based Position RMSE [m]:\n');
fprintf(fid, '  Before Outage (0 - %.1f s):\n', gpsOutageStart);
fprintf(fid, '    IMU Dead Reckoning: %.3f m\n', rmseDrBefore);
fprintf(fid, '    EKF Solution:       %.3f m\n\n', rmseEkfBefore);

fprintf(fid, '  During Outage (%.1f - %.1f s):\n', gpsOutageStart, gpsOutageEnd);
fprintf(fid, '    IMU Dead Reckoning: %.3f m\n', rmseDrOutage);
fprintf(fid, '    EKF Solution:       %.3f m\n');
fprintf(fid, '    EKF Advantage:      %.1f%%\n\n', ...
    100 * (1 - rmseEkfOutage/rmseDrOutage));

fprintf(fid, '  After Outage (%.1f - %.1f s):\n', gpsOutageEnd, max(t));
fprintf(fid, '    IMU Dead Reckoning: %.3f m\n', rmseDrAfter);
fprintf(fid, '    EKF Solution:       %.3f m\n\n', rmseEkfAfter);

fprintf(fid, 'Overall Statistics:\n');
fprintf(fid, '  Max Position Error (DR):  %.3f m\n', max(err.drPos));
fprintf(fid, '  Max Position Error (EKF): %.3f m\n', max(err.ekfPos));
fprintf(fid, '  Mean Position Error (DR):  %.3f m\n', mean(err.drPos));
fprintf(fid, '  Mean Position Error (EKF): %.3f m\n\n', mean(err.ekfPos));

fprintf(fid, '================================================================================\n');
fprintf(fid, 'Key Observations:\n');
fprintf(fid, '  - EKF prediction-only mode during outage limits but does not eliminate error growth\n');
fprintf(fid, '  - Upon GPS recovery, EKF quickly re-acquires lock through Kalman correction\n');
fprintf(fid, '  - IMU bias/drift effects dominate during long outages\n');
fprintf(fid, '  - Filter covariance is valuable for uncertainty tracking and decision-making\n');
fprintf(fid, '================================================================================\n');

fclose(fid);

end
