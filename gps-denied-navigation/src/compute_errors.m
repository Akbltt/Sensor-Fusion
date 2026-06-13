function err = compute_errors(truth, sensors, dr, ekf)
%COMPUTE_ERRORS Compute error metrics for dead reckoning and EKF.
%
% Returns position, velocity, and heading errors for performance analysis.

N = numel(truth.t);

% Position errors
drPos = sqrt(sum((dr.state(:, 1:2) - truth.state(:, 1:2)).^2, 2));
ekfPos = sqrt(sum((ekf.state(:, 1:2) - truth.state(:, 1:2)).^2, 2));

% Velocity errors (scalar)
drVel = abs(dr.state(:, 3) - truth.state(:, 3));
ekfVel = abs(ekf.state(:, 3) - truth.state(:, 3));

% Heading errors (wrap to [-pi, pi])
drHdg = abs(wrap_to_pi(dr.state(:, 4) - truth.state(:, 4)));
ekfHdg = abs(wrap_to_pi(ekf.state(:, 4) - truth.state(:, 4)));

err.t = truth.t;
err.drPos = drPos;
err.ekfPos = ekfPos;
err.drVel = drVel;
err.ekfVel = ekfVel;
err.drHdg = drHdg;
err.ekfHdg = ekfHdg;

err.gpsOutageStart = sensors.gpsOutageStart;
err.gpsOutageEnd = sensors.gpsOutageEnd;

end
