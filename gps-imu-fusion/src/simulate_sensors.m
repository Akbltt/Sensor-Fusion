function sensors = simulate_sensors(truth, cfg)
%SIMULATE_SENSORS Generates noisy GPS and IMU measurements.

rng(cfg.rngSeed + 1);

t = truth.t;
N = numel(t);

aMeas = truth.aTrue;
rMeas = truth.rTrue;

biasAcc = zeros(N, 1);
biasYaw = zeros(N, 1);
biasAcc(1) = cfg.imuBiasAcc0 * double(cfg.useImuBias);
biasYaw(1) = cfg.imuBiasYawRate0 * double(cfg.useImuBias);

for k = 2:N
    if cfg.useImuBias
        biasAcc(k) = biasAcc(k - 1) + cfg.imuBiasRwAcc * sqrt(cfg.dtImu) * randn;
        biasYaw(k) = biasYaw(k - 1) + cfg.imuBiasRwYaw * sqrt(cfg.dtImu) * randn;
    end
end

aMeas = aMeas + biasAcc + cfg.imuSigmaAcc * randn(N, 1);
rMeas = rMeas + biasYaw + cfg.imuSigmaYawRate * randn(N, 1);

isGpsTime = abs(mod(t, cfg.dtGps)) < 1e-9;
isOutage = t >= cfg.gpsOutageStart & t <= cfg.gpsOutageEnd;
isGpsAvailable = isGpsTime & ~isOutage;

gps = nan(N, 2);
gps(isGpsAvailable, :) = truth.state(isGpsAvailable, 1:2) + ...
    cfg.gpsSigmaXY * randn(sum(isGpsAvailable), 2);

% Build a simple GPS-only localization by holding the latest measurement.
gpsOnly = nan(N, 2);
last = [nan, nan];
for k = 1:N
    if isGpsAvailable(k)
        last = gps(k, :);
    end
    gpsOnly(k, :) = last;
end

sensors.t = t;
sensors.aMeas = aMeas;
sensors.rMeas = rMeas;
sensors.gps = gps;
sensors.gpsOnly = gpsOnly;
sensors.isGpsAvailable = isGpsAvailable;
sensors.biasAcc = biasAcc;
sensors.biasYaw = biasYaw;
end
