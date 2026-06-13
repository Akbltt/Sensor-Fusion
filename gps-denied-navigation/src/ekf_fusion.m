function ekf = ekf_fusion(sensors, truth, cfg)
%EKF_FUSION Extended Kalman Filter with GPS outage handling.
%
% During GPS availability: EKF prediction + GPS position correction
% During GPS outage: EKF prediction only (dead reckoning with uncertainty tracking)
%
% Demonstrates how filter covariance grows during outage and recovers when
% GPS becomes available.

N = numel(sensors.t);
xHat = zeros(N, 4);
PHist = zeros(4, 4, N);

% Initialize filter with ground truth (ideally, use prior estimate)
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

    % Predict: propagate state and covariance
    xPred = propagate_state(xk, a, r, cfg.dtImu);
    [F, G] = state_jacobian(xk, a, cfg.dtImu);
    PPred = F * P * F.' + G * Qin * G.';

    % Correct: GPS measurement update only if available
    if sensors.isGpsAvailable(k + 1)
        % Map IMU time index to GPS measurement index
        tk = sensors.t(k + 1);
        gpsIdx = round(tk / cfg.dtGps) + 1;
        gpsIdx = max(1, min(size(sensors.gps, 1), gpsIdx));
        
        % Only apply GPS update if measurement exists
        if gpsIdx <= size(sensors.gps, 1) && gpsIdx >= 1
            z = sensors.gps(gpsIdx, :).';
            
            % Innovation and innovation covariance
            y = z - H * xPred;
            S = H * PPred * H.' + R;
            
            % Kalman gain
            K = PPred * H.' / S;

            % State update
            xUpd = xPred + K * y;
            xUpd(4) = wrap_to_pi(xUpd(4));

            % Covariance update (Joseph form for numerical stability)
            I = eye(4);
            P = (I - K * H) * PPred * (I - K * H).' + K * R * K.';
            
            xHat(k + 1, :) = xUpd.';
        else
            % No GPS measurement at this index, prediction only
            xHat(k + 1, :) = xPred.';
            P = PPred;
        end
    else
        % During GPS outage: prediction only, no correction
        % Covariance grows as uncertainty accumulates
        xHat(k + 1, :) = xPred.';
        P = PPred;
    end

    PHist(:, :, k + 1) = P;
end

ekf.state = xHat;
ekf.P = PHist;
ekf.gpsOutageStart = sensors.gpsOutageStart;
ekf.gpsOutageEnd = sensors.gpsOutageEnd;

end
