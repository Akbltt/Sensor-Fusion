function sensors = simulate_sensors(truth, cfg)
%SIMULATE_SENSORS Generates GPS and IMU measurements with realism.
%
% Models:
% - GPS: asynchronous position measurements at cfg.dtGps rate
% - IMU: continuous acceleration and yaw rate with noise/bias
%
% GPS availability can be controlled with GPS outage interval.

rng(cfg.rngSeed);

N = numel(truth.t);
t = truth.t;

%% IMU Simulation (high rate, 50 Hz)
aMeas = zeros(N, 1);
rMeas = zeros(N, 1);

if cfg.useImuBias
    % Slowly varying bias (random walk)
    biasAcc = cfg.imuBiasAcc0 + cumsum(randn(N, 1)) * cfg.imuBiasRwAcc * sqrt(cfg.dtImu);
    biasYaw = cfg.imuBiasYawRate0 + cumsum(randn(N, 1)) * cfg.imuBiasRwYaw * sqrt(cfg.dtImu);
else
    biasAcc = cfg.imuBiasAcc0 * ones(N, 1);
    biasYaw = cfg.imuBiasYawRate0 * ones(N, 1);
end

aMeas = truth.aTrue + biasAcc + cfg.imuSigmaAcc * randn(N, 1);
rMeas = truth.rTrue + biasYaw + cfg.imuSigmaYawRate * randn(N, 1);

%% GPS Simulation (low rate, 1 Hz)
% Measurements are taken at discrete times cfg.dtGps apart
nGps = floor(cfg.T / cfg.dtGps) + 1;
tGps = (0:cfg.dtGps:(nGps-1)*cfg.dtGps)';

% Interpolate true position at GPS times
tIndex = round(tGps / cfg.dtImu) + 1;
tIndex = max(1, min(N, tIndex));

xTrue = truth.state(tIndex, 1);
yTrue = truth.state(tIndex, 2);

% Add GPS noise
gps = zeros(nGps, 2);
gps(:, 1) = xTrue + cfg.gpsSigmaXY * randn(nGps, 1);
gps(:, 2) = yTrue + cfg.gpsSigmaXY * randn(nGps, 1);

%% GPS Availability (Outage Interval)
isGpsAvailable = true(N, 1);
isGpsAvailable(t >= cfg.gpsOutageStart & t < cfg.gpsOutageEnd) = false;

%% Package Output
sensors.t = t;
sensors.tGps = tGps;
sensors.aMeas = aMeas;
sensors.rMeas = rMeas;
sensors.gps = gps;
sensors.isGpsAvailable = isGpsAvailable;
sensors.gpsOutageStart = cfg.gpsOutageStart;
sensors.gpsOutageEnd = cfg.gpsOutageEnd;

end
