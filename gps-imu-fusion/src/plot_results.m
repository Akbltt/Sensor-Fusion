function plot_results(truth, sensors, dr, ekf, cfg)
%PLOT_RESULTS Creates engineering-oriented figures and saves them to results/.

if ~exist(cfg.resultsDir, 'dir')
    mkdir(cfg.resultsDir);
end

err = compute_errors(truth, sensors, dr, ekf);
t = truth.t;

% Trajectory overview
f1 = figure('Color', 'w', 'Position', [100 100 1100 700]);
plot(truth.state(:, 1), truth.state(:, 2), 'k-', 'LineWidth', 2); hold on;
plot(sensors.gps(:, 1), sensors.gps(:, 2), '.', 'Color', [0.4 0.4 0.9], 'MarkerSize', 10);
plot(dr.state(:, 1), dr.state(:, 2), '-', 'Color', [0.9 0.35 0.15], 'LineWidth', 1.4);
plot(ekf.state(:, 1), ekf.state(:, 2), '-', 'Color', [0.1 0.55 0.2], 'LineWidth', 1.8);

% Uncertainty ellipses for EKF at sparse timestamps.
for k = 1:round(8 / cfg.dtImu):numel(t)
    Pxy = ekf.P(1:2, 1:2, k);
    draw_covariance_ellipse(ekf.state(k, 1:2).', Pxy, 2.0, [0.1 0.55 0.2], 0.12);
end

xlabel('X position [m]'); ylabel('Y position [m]');
title('2D Trajectory: Truth vs GPS vs IMU DR vs EKF');
grid on; axis equal;
legend('Ground Truth', 'GPS Measurements', 'IMU Dead Reckoning', 'EKF Fusion', 'Location', 'best');
saveas(f1, fullfile(cfg.resultsDir, 'trajectory_comparison.png'));

% Position error
f2 = figure('Color', 'w', 'Position', [120 120 1100 360]);
plot(t, err.gpsPos, 'Color', [0.35 0.35 0.9], 'LineWidth', 1.2); hold on;
plot(t, err.drPos, 'Color', [0.9 0.35 0.15], 'LineWidth', 1.4);
plot(t, err.ekfPos, 'Color', [0.1 0.55 0.2], 'LineWidth', 1.6);
xlabel('Time [s]'); ylabel('Position Error Norm [m]');
title('Position Error vs Time');
grid on;
legend('GPS Only (hold)', 'IMU Dead Reckoning', 'EKF Fusion', 'Location', 'best');
saveas(f2, fullfile(cfg.resultsDir, 'position_error.png'));

% Velocity and heading error
f3 = figure('Color', 'w', 'Position', [140 140 1100 700]);
subplot(2, 1, 1);
plot(t, err.drVel, 'Color', [0.9 0.35 0.15], 'LineWidth', 1.3); hold on;
plot(t, err.ekfVel, 'Color', [0.1 0.55 0.2], 'LineWidth', 1.5);
ylabel('Velocity Error [m/s]');
grid on;
legend('IMU Dead Reckoning', 'EKF Fusion', 'Location', 'best');
title('Velocity and Heading Error');

subplot(2, 1, 2);
plot(t, rad2deg(err.drHeading), 'Color', [0.9 0.35 0.15], 'LineWidth', 1.3); hold on;
plot(t, rad2deg(err.ekfHeading), 'Color', [0.1 0.55 0.2], 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Heading Error [deg]');
grid on;
legend('IMU Dead Reckoning', 'EKF Fusion', 'Location', 'best');
saveas(f3, fullfile(cfg.resultsDir, 'velocity_heading_error.png'));

% Covariance evolution
f4 = figure('Color', 'w', 'Position', [160 160 1100 500]);
plot(t, squeeze(ekf.P(1, 1, :)), 'LineWidth', 1.3); hold on;
plot(t, squeeze(ekf.P(2, 2, :)), 'LineWidth', 1.3);
plot(t, squeeze(ekf.P(3, 3, :)), 'LineWidth', 1.3);
plot(t, squeeze(ekf.P(4, 4, :)), 'LineWidth', 1.3);
xlabel('Time [s]'); ylabel('Variance');
title('EKF State Covariance Diagonal Evolution');
grid on;
legend('Pxx', 'Pyy', 'Pvv', 'Ppsi');
saveas(f4, fullfile(cfg.resultsDir, 'covariance_evolution.png'));

% Save summary metrics
metricsText = compose_metrics_text(err);
fid = fopen(fullfile(cfg.resultsDir, 'summary_metrics.txt'), 'w');
fprintf(fid, '%s\n', metricsText{:});
fclose(fid);
end

function lines = compose_metrics_text(err)
lines = {
    sprintf('GPS position RMSE [m]: %.3f', rms(err.gpsPos(~isnan(err.gpsPos))))
    sprintf('IMU DR position RMSE [m]: %.3f', rms(err.drPos))
    sprintf('EKF position RMSE [m]: %.3f', rms(err.ekfPos))
    sprintf('IMU DR velocity RMSE [m/s]: %.3f', rms(err.drVel))
    sprintf('EKF velocity RMSE [m/s]: %.3f', rms(err.ekfVel))
    sprintf('IMU DR heading RMSE [deg]: %.3f', rad2deg(rms(err.drHeading)))
    sprintf('EKF heading RMSE [deg]: %.3f', rad2deg(rms(err.ekfHeading)))
    };
end
