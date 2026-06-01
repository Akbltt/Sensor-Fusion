function err = compute_errors(truth, sensors, dr, ekf)
%COMPUTE_ERRORS Computes position, velocity, and heading error signals.

N = size(truth.state, 1);

truthPos = truth.state(:, 1:2);
truthV = truth.state(:, 3);
truthPsi = truth.state(:, 4);

err.gpsPos = nan(N, 1);
validGps = ~any(isnan(sensors.gpsOnly), 2);
err.gpsPos(validGps) = vecnorm(sensors.gpsOnly(validGps, :) - truthPos(validGps, :), 2, 2);

err.drPos = vecnorm(dr.state(:, 1:2) - truthPos, 2, 2);
err.ekfPos = vecnorm(ekf.state(:, 1:2) - truthPos, 2, 2);

err.drVel = dr.state(:, 3) - truthV;
err.ekfVel = ekf.state(:, 3) - truthV;

err.drHeading = arrayfun(@(a, b) wrap_to_pi(a - b), dr.state(:, 4), truthPsi);
err.ekfHeading = arrayfun(@(a, b) wrap_to_pi(a - b), ekf.state(:, 4), truthPsi);
end
