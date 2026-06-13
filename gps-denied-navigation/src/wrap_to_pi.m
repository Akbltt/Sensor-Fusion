function ang = wrap_to_pi(ang)
%WRAP_TO_PI Normalize angle to [-pi, pi].

ang = mod(ang + pi, 2*pi) - pi;

end
