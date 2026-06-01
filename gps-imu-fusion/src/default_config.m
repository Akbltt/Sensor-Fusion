function cfg = default_config(projectRoot)
%DEFAULT_CONFIG Central configuration for simulation, sensors, and EKF.

cfg.projectRoot = projectRoot;
cfg.resultsDir = fullfile(projectRoot, 'results');

% Time settings
cfg.T = 140;               % [s] total simulation time
cfg.dtImu = 0.02;          % [s] IMU period (50 Hz)
cfg.dtGps = 1.0;           % [s] GPS period (1 Hz)

% Initial true state [x; y; v; psi]
cfg.x0 = [0; 0; 4; deg2rad(5)];

% GPS model
cfg.gpsSigmaXY = 2.5;      % [m]

% IMU model
cfg.imuSigmaAcc = 0.12;    % [m/s^2]
cfg.imuSigmaYawRate = deg2rad(0.6); % [rad/s]

% Optional slowly varying IMU bias/drift
cfg.useImuBias = true;
cfg.imuBiasAcc0 = 0.03;    % [m/s^2]
cfg.imuBiasYawRate0 = deg2rad(0.08); % [rad/s]
cfg.imuBiasRwAcc = 0.002;  % [m/s^2/sqrt(s)]
cfg.imuBiasRwYaw = deg2rad(0.01);    % [rad/s/sqrt(s)]

% EKF tuning
cfg.ekfP0 = diag([4^2, 4^2, 1.5^2, deg2rad(10)^2]);
cfg.ekfQAcc = 0.18^2;
cfg.ekfQYawRate = deg2rad(0.8)^2;
cfg.ekfR = diag([cfg.gpsSigmaXY^2, cfg.gpsSigmaXY^2]);

% GPS outage scenario in baseline
cfg.gpsOutageStart = 65;
cfg.gpsOutageEnd = 85;

% Reproducibility
cfg.rngSeed = 9;

% Animation settings
cfg.animDownsample = 2;
cfg.writeMp4 = false;
cfg.gifSpeedMultiplier = 3.2; % >1 makes playback faster
cfg.gifName = 'fusion_animation.gif';
cfg.mp4Name = 'fusion_animation.mp4';
end
