function dr = imu_dead_reckoning(sensors, truth, cfg)
%IMU_DEAD_RECKONING Baseline: pure IMU integration without GPS corrections.
%
% This represents what happens without the EKF correction:
% errors accumulate indefinitely during GPS outage and never recover.
% Useful for understanding EKF benefit.

N = numel(sensors.t);
xDr = zeros(N, 4);
xDr(1, :) = truth.state(1, :);

for k = 1:N-1
    a = sensors.aMeas(k);
    r = sensors.rMeas(k);
    xDr(k + 1, :) = propagate_state(xDr(k, :).', a, r, cfg.dtImu).';
end

dr.state = xDr;
dr.gpsOutageStart = sensors.gpsOutageStart;
dr.gpsOutageEnd = sensors.gpsOutageEnd;

end
