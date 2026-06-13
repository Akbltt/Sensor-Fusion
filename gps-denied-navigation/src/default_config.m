function cfg = default_config(projectRoot)
%DEFAULT_CONFIG Central configuration for GPS-denied navigation simulation.
%
% This configuration defines:
% - Simulation time and sensor rates
% - Sensor noise models
% - EKF tuning parameters
% - GPS outage scenario
% - Visualization settings

cfg.projectRoot = projectRoot;
cfg.resultsDir = fullfile(projectRoot, 'results');

%% Time and Sampling
cfg.T = 140;               % [s] total simulation time
cfg.dtImu = 0.02;          % [s] IMU update period (50 Hz)
cfg.dtGps = 1.0;           % [s] GPS update period (1 Hz)

%% Initial Conditions
cfg.x0 = [0; 0; 4; deg2rad(5)];  % [x, y, v, psi]

%% GPS Sensor Model
cfg.gpsSigmaXY = 2.5;      % [m] position measurement noise (1-sigma)

%% IMU Sensor Model
cfg.imuSigmaAcc = 0.12;    % [m/s^2] acceleration noise (1-sigma)
cfg.imuSigmaYawRate = deg2rad(0.6); % [rad/s] yaw rate noise (1-sigma)

% Optional slowly varying IMU bias (realistic for longer experiments)
cfg.useImuBias = true;
cfg.imuBiasAcc0 = 0.03;    % [m/s^2] initial acceleration bias
cfg.imuBiasYawRate0 = deg2rad(0.08); % [rad/s] initial yaw rate bias
cfg.imuBiasRwAcc = 0.002;  % [m/s^2/sqrt(s)] acceleration bias random walk
cfg.imuBiasRwYaw = deg2rad(0.01);    % [rad/s/sqrt(s)] yaw rate bias random walk

%% EKF Tuning
cfg.ekfP0 = diag([4^2, 4^2, 1.5^2, deg2rad(10)^2]);  % initial covariance
cfg.ekfQAcc = 0.18^2;                 % process noise - acceleration
cfg.ekfQYawRate = deg2rad(0.8)^2;     % process noise - yaw rate
cfg.ekfR = diag([cfg.gpsSigmaXY^2, cfg.gpsSigmaXY^2]); % measurement noise

%% GPS Outage Scenario (Primary Focus)
cfg.gpsOutageStart = 30;   % [s] outage begins
cfg.gpsOutageEnd = 50;     % [s] outage ends
% Interpretation: GPS available [0, 30), unavailable [30, 50), available [50, T]

%% Reproducibility
cfg.rngSeed = 9;

%% Visualization and Export
cfg.animDownsample = 2;    % render every N time steps
cfg.animMaxFrames = 450;   % hard cap to keep GIF export time reasonable
cfg.writeMp4 = false;      % export to MP4 (requires video codec)
cfg.gifSpeedMultiplier = 3.2; % >1 accelerates playback
cfg.gifName = 'fusion_animation.gif';
cfg.mp4Name = 'fusion_animation.mp4';

end
