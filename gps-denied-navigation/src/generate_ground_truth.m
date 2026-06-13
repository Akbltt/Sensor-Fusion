function truth = generate_ground_truth(cfg)
%GENERATE_GROUND_TRUTH Creates ground truth trajectory with mixed maneuvers.
%
% Generates a realistic 2D vehicle trajectory with:
% - Straight segments at various velocities
% - Curved turns with yaw rate changes
% - Acceleration and deceleration phases
%
% Trajectory is timed to be interesting during the GPS outage interval.

rng(cfg.rngSeed);

t = (0:cfg.dtImu:cfg.T)';
N = numel(t);

x = zeros(N, 4); % [x y v psi]
x(1, :) = cfg.x0.';

aTrue = zeros(N, 1);
rTrue = zeros(N, 1);

for k = 1:N
    tk = t(k);

    % Acceleration/deceleration profile
    if tk < 20
        a = 0.08;      % accelerate
    elseif tk < 35
        a = 0.00;      % constant velocity
    elseif tk < 55
        a = -0.06;     % decelerate (includes outage period)
    elseif tk < 75
        a = 0.05;      % accelerate
    elseif tk < 95
        a = 0.00;      % constant
    elseif tk < 115
        a = -0.03;     % gentle deceleration
    else
        a = 0.02;      % gentle acceleration to end
    end

    % Yaw rate profile (creates turns)
    if tk < 25
        r = 0.00;                % straight
    elseif tk < 45
        r = deg2rad(2.6);        % left turn (overlaps with end of outage)
    elseif tk < 65
        r = 0.00;                % straight (after outage)
    elseif tk < 90
        r = deg2rad(-2.0);       % right turn
    elseif tk < 115
        r = 0.00;                % straight
    else
        r = deg2rad(1.6);        % gentle left turn
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
