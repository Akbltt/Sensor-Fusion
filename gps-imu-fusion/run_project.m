clear; clc; close all;

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot, 'src'));

cfg = default_config(projectRoot);

truth = generate_ground_truth(cfg);
sensors = simulate_sensors(truth, cfg);

dr = imu_dead_reckoning(sensors, truth, cfg);
ekf = ekf_fusion(sensors, truth, cfg);

plot_results(truth, sensors, dr, ekf, cfg);
animate_results(truth, sensors, dr, ekf, cfg);
run_experiments(cfg);

disp('Simulation complete. See results/ for generated figures and animations.');
