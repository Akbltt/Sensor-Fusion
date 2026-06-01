function truth = generate_ground_truth(cfg)
%GENERATE_GROUND_TRUTH Creates a mixed straight/turning trajectory.

rng(cfg.rngSeed);

t = (0:cfg.dtImu:cfg.T)';
N = numel(t);

x = zeros(N, 4); % [x y v psi]
x(1, :) = cfg.x0.';

aTrue = zeros(N, 1);
rTrue = zeros(N, 1);

for k = 1:N
    tk = t(k);

    % Longitudinal profile with acceleration and deceleration segments.
    if tk < 20
        a = 0.08;
    elseif tk < 35
        a = 0.00;
    elseif tk < 55
        a = -0.06;
    elseif tk < 75
        a = 0.05;
    elseif tk < 95
        a = 0.00;
    elseif tk < 115
        a = -0.03;
    else
        a = 0.02;
    end

    % Yaw-rate profile to create curved segments.
    if tk < 25
        r = 0.00;
    elseif tk < 45
        r = deg2rad(2.6);
    elseif tk < 65
        r = 0.00;
    elseif tk < 90
        r = deg2rad(-2.0);
    elseif tk < 115
        r = 0.00;
    else
        r = deg2rad(1.6);
    end

    aTrue(k) = a;
    rTrue(k) = r;

    if k < N
        x(k + 1, :) = propagate_state(x(k, :).', a, r, cfg.dtImu).';
    end
end

truth.t = t;
truth.state = x;
truth.aTrue = aTrue;
truth.rTrue = rTrue;
end
