function ang = wrap_to_pi(ang)
%WRAP_TO_PI Wrap angle in radians to [-pi, pi].
ang = mod(ang + pi, 2 * pi) - pi;
end
