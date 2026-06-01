function dr = imu_dead_reckoning(sensors, truth, cfg)
%IMU_DEAD_RECKONING Integrates IMU-only motion estimate.

N = numel(sensors.t);
x = zeros(N, 4);
x(1, :) = truth.state(1, :);

for k = 1:N-1
    x(k + 1, :) = propagate_state(x(k, :).', sensors.aMeas(k), sensors.rMeas(k), cfg.dtImu).';
end

dr.state = x;
end
