function [F, G] = state_jacobian(x, a, dt)
%STATE_JACOBIAN Jacobians for EKF prediction.

v = x(3);
psi = x(4);

F = eye(4);
F(1, 3) = cos(psi) * dt;
F(1, 4) = -v * sin(psi) * dt - 0.5 * a * sin(psi) * dt^2;
F(2, 3) = sin(psi) * dt;
F(2, 4) = v * cos(psi) * dt + 0.5 * a * cos(psi) * dt^2;

G = zeros(4, 2); % [accNoise; yawRateNoise]
G(1, 1) = 0.5 * cos(psi) * dt^2;
G(2, 1) = 0.5 * sin(psi) * dt^2;
G(3, 1) = dt;
G(4, 2) = dt;
end
