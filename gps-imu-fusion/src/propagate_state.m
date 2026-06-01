function xNext = propagate_state(x, a, r, dt)
%PROPAGATE_STATE Nonlinear unicycle-like kinematics with acceleration input.
% State x = [x; y; v; psi], inputs are longitudinal acceleration and yaw rate.

px = x(1);
py = x(2);
v = x(3);
psi = x(4);

px = px + v * cos(psi) * dt + 0.5 * a * cos(psi) * dt^2;
py = py + v * sin(psi) * dt + 0.5 * a * sin(psi) * dt^2;
v = v + a * dt;
psi = wrap_to_pi(psi + r * dt);

xNext = [px; py; v; psi];
end
