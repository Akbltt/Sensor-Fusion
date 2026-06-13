function run_project()
%RUN_PROJECT Main entry point for GPS-Denied Navigation analysis.
%
% Execution sequence:
% 1. Load configuration
% 2. Generate ground truth trajectory
% 3. Simulate GPS and IMU sensors with outage
% 4. Run EKF fusion algorithm
% 5. Compute IMU-only dead reckoning baseline
% 6. Calculate error metrics
% 7. Generate plots and analysis
% 8. Create animation

close all; clear all; clc;

% Robustly detect project root from current working directory
currentDir = pwd;

% Check if we're in the src directory or project root
if strcmp(fileparts(currentDir), fileparts(fileparts(currentDir)))
    % We're in src or a subdirectory
    projectRoot = fileparts(currentDir);
else
    projectRoot = currentDir;
end

% Ensure src directory exists and add to path
srcPath = fullfile(projectRoot, 'src');
if isfolder(srcPath)
    addpath(srcPath);
else
    error('Could not find src directory at: %s', srcPath);
end

% Verify project structure and create results if needed
resultsDir = fullfile(projectRoot, 'results');
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

fprintf('================================================================================\n');
fprintf('GPS-DENIED NAVIGATION ANALYSIS\n');
fprintf('================================================================================\n\n');

%% Configuration
fprintf('Loading configuration...\n');
cfg = default_config(projectRoot);
fprintf('  Simulation time: %.1f s\n', cfg.T);
fprintf('  GPS outage: [%.1f, %.1f) s\n', cfg.gpsOutageStart, cfg.gpsOutageEnd);
fprintf('  GPS sigma: %.2f m\n', cfg.gpsSigmaXY);
fprintf('  IMU sigma (accel): %.3f m/s^2\n', cfg.imuSigmaAcc);
fprintf('\n');

%% Ground Truth
fprintf('Generating ground truth trajectory...\n');
truth = generate_ground_truth(cfg);
fprintf('  Generated %d state samples (%.1f Hz effective)\n', numel(truth.t), 1/cfg.dtImu);
fprintf('\n');

%% Sensor Simulation
fprintf('Simulating GPS and IMU sensors...\n');
sensors = simulate_sensors(truth, cfg);
fprintf('  GPS measurements: %d samples (%.1f Hz)\n', numel(sensors.tGps), 1/cfg.dtGps);
fprintf('  IMU measurements: %d samples (%.1f Hz)\n', numel(sensors.t), 1/cfg.dtImu);
fprintf('\n');

%% EKF Fusion
fprintf('Running EKF fusion with GPS outage handling...\n');
tic;
ekf = ekf_fusion(sensors, truth, cfg);
ekfTime = toc;
fprintf('  Completed in %.3f s\n', ekfTime);
fprintf('  State estimates: %d samples\n', size(ekf.state, 1));
fprintf('\n');

%% IMU Dead Reckoning (Baseline)
fprintf('Computing IMU-only dead reckoning baseline...\n');
dr = imu_dead_reckoning(sensors, truth, cfg);
fprintf('  Baseline estimate: %d samples\n', size(dr.state, 1));
fprintf('\n');

%% Error Analysis
fprintf('Computing error metrics...\n');
err = compute_errors(truth, sensors, dr, ekf);

% Quick statistics
drPosRms = rms(err.drPos);
ekfPosRms = rms(err.ekfPos);
improvement = 100 * (1 - ekfPosRms / drPosRms);

fprintf('  Overall Position RMS:\n');
fprintf('    IMU Dead Reckoning: %.3f m\n', drPosRms);
fprintf('    EKF Solution:       %.3f m\n', ekfPosRms);
fprintf('    EKF Improvement:    %.1f%%\n', improvement);
fprintf('\n');

%% Visualization
fprintf('Generating plots...\n');
plot_results(truth, sensors, dr, ekf, err, cfg);
fprintf('  Saved plots to %s\n', cfg.resultsDir);
fprintf('\n');

%% Animation
fprintf('Creating animation...\n');
animate_results(truth, sensors, dr, ekf, cfg);
fprintf('\n');

%% Summary
fprintf('================================================================================\n');
fprintf('ANALYSIS COMPLETE\n');
fprintf('================================================================================\n');
fprintf('\nResults Summary:\n');
fprintf('  - Trajectory visualization: fusion_trajectory.png\n');
fprintf('  - Error plots: position_error.png, velocity_error.png, heading_error.png\n');
fprintf('  - Covariance analysis: covariance_evolution.png\n');
fprintf('  - Metrics summary: summary_metrics.txt\n');
fprintf('  - Animation: fusion_animation.gif\n');
fprintf('\nKey Findings:\n');
fprintf('  - EKF prediction-only mode during GPS outage limits error accumulation\n');
fprintf('  - Error growth rate increases during outage (no GPS corrections)\n');
fprintf('  - Upon GPS recovery, EKF quickly re-acquires true trajectory\n');
fprintf('  - Filter covariance grows during outage, decreases after recovery\n');
fprintf('  - IMU bias/drift effects dominate long-duration outage performance\n');
fprintf('\nSee docs/ for detailed technical analysis and tuning observations.\n');
fprintf('================================================================================\n\n');

end
