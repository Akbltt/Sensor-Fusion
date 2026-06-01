function ekf = ekf_fusion(sensors, truth, cfg)
%EKF_FUSION EKF with IMU prediction and GPS position updates.

N = numel(sensors.t);
xHat = zeros(N, 4);
PHist = zeros(4, 4, N);

xHat(1, :) = truth.state(1, :);
P = cfg.ekfP0;
PHist(:, :, 1) = P;

Qin = diag([cfg.ekfQAcc, cfg.ekfQYawRate]);
H = [1 0 0 0; 0 1 0 0];
R = cfg.ekfR;

for k = 1:N-1
    xk = xHat(k, :).';
    a = sensors.aMeas(k);
    r = sensors.rMeas(k);

    % Predict
    xPred = propagate_state(xk, a, r, cfg.dtImu);
    [F, G] = state_jacobian(xk, a, cfg.dtImu);
    PPred = F * P * F.' + G * Qin * G.';

    % Correct only when GPS is available.
    if sensors.isGpsAvailable(k + 1)
        z = sensors.gps(k + 1, :).';
        y = z - H * xPred;
        S = H * PPred * H.' + R;
        K = PPred * H.' / S;

        xUpd = xPred + K * y;
        xUpd(4) = wrap_to_pi(xUpd(4));

        I = eye(4);
        P = (I - K * H) * PPred * (I - K * H).' + K * R * K.';
        xHat(k + 1, :) = xUpd.';
    else
        xHat(k + 1, :) = xPred.';
        P = PPred;
    end

    PHist(:, :, k + 1) = P;
end

ekf.state = xHat;
ekf.P = PHist;
end
