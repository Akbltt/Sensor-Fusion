function [F, G] = state_jacobian(x, a, dt)
%STATE_JACOBIAN Compute linearization matrices for EKF prediction step.
%
% F: state transition Jacobian (dxNext/dx)
% G: process noise Jacobian (dxNext/dw), where w = [a_noise; r_noise]'

px = x(1);
py = x(2);
v = x(3);
psi = x(4);

% State Jacobian
F = [
    1,  0,  cos(psi)*dt,  -v*sin(psi)*dt - 0.5*a*sin(psi)*dt^2;
    0,  1,  sin(psi)*dt,   v*cos(psi)*dt + 0.5*a*cos(psi)*dt^2;
    0,  0,  1,             0;
    0,  0,  0,             1
];

% Process Noise Jacobian
G = [
    0.5*cos(psi)*dt^2,  0;
    0.5*sin(psi)*dt^2,  0;
    dt,                  0;
    0,                   dt
];

end
