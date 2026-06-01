function run_experiments(cfg)
%RUN_EXPERIMENTS Runs compact what-if studies and saves summary figure.

cases = {
    struct('name', 'Baseline', 'gpsSigmaXY', cfg.gpsSigmaXY, 'useImuBias', true, 'gpsOut', [cfg.gpsOutageStart cfg.gpsOutageEnd], 'dtGps', cfg.dtGps)
    struct('name', 'High GPS Noise', 'gpsSigmaXY', 5.0, 'useImuBias', true, 'gpsOut', [cfg.gpsOutageStart cfg.gpsOutageEnd], 'dtGps', cfg.dtGps)
    struct('name', 'High IMU Drift', 'gpsSigmaXY', cfg.gpsSigmaXY, 'useImuBias', true, 'gpsOut', [cfg.gpsOutageStart cfg.gpsOutageEnd], 'dtGps', cfg.dtGps)
    struct('name', 'Slow GPS Update', 'gpsSigmaXY', cfg.gpsSigmaXY, 'useImuBias', true, 'gpsOut', [cfg.gpsOutageStart cfg.gpsOutageEnd], 'dtGps', 2.0)
    struct('name', 'Long GPS Outage', 'gpsSigmaXY', cfg.gpsSigmaXY, 'useImuBias', true, 'gpsOut', [40 100], 'dtGps', cfg.dtGps)
    };

rmseDr = zeros(numel(cases), 1);
rmseEkf = zeros(numel(cases), 1);

for i = 1:numel(cases)
    c = cases{i};
    cCfg = cfg;
    cCfg.gpsSigmaXY = c.gpsSigmaXY;
    cCfg.ekfR = diag([cCfg.gpsSigmaXY^2, cCfg.gpsSigmaXY^2]);
    cCfg.useImuBias = c.useImuBias;
    cCfg.dtGps = c.dtGps;
    cCfg.gpsOutageStart = c.gpsOut(1);
    cCfg.gpsOutageEnd = c.gpsOut(2);

    if strcmp(c.name, 'High IMU Drift')
        cCfg.imuBiasRwAcc = 0.006;
        cCfg.imuBiasRwYaw = deg2rad(0.03);
    end

    truth = generate_ground_truth(cCfg);
    sensors = simulate_sensors(truth, cCfg);
    dr = imu_dead_reckoning(sensors, truth, cCfg);
    ekf = ekf_fusion(sensors, truth, cCfg);
    err = compute_errors(truth, sensors, dr, ekf);

    rmseDr(i) = rms(err.drPos);
    rmseEkf(i) = rms(err.ekfPos);
end

fig = figure('Color', 'w', 'Position', [180 180 1100 500]);
b = bar([rmseDr rmseEkf], 'grouped');
b(1).FaceColor = [0.9 0.35 0.15];
b(2).FaceColor = [0.1 0.55 0.2];
set(gca, 'XTick', 1:numel(cases), 'XTickLabel', cellfun(@(x) x.name, cases, 'UniformOutput', false));
ylabel('Position RMSE [m]');
title('Experiment Comparison: IMU DR vs EKF Position Error');
grid on;
legend({'IMU Dead Reckoning', 'EKF Fusion'}, 'Location', 'northwest');
xtickangle(20);

saveas(fig, fullfile(cfg.resultsDir, 'experiment_rmse_comparison.png'));
close(fig);

fid = fopen(fullfile(cfg.resultsDir, 'experiment_summary.csv'), 'w');
fprintf(fid, 'Case,IMU_DR_RMSE_m,EKF_RMSE_m\n');
for i = 1:numel(cases)
    fprintf(fid, '%s,%.5f,%.5f\n', cases{i}.name, rmseDr(i), rmseEkf(i));
end
fclose(fid);
end
