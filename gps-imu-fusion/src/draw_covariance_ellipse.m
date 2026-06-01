function draw_covariance_ellipse(mu, Pxy, nsig, color, alphaVal)
%DRAW_COVARIANCE_ELLIPSE Draws a 2D covariance ellipse.

if any(~isfinite(Pxy), 'all') || min(eig((Pxy + Pxy.') / 2)) <= 0
    return;
end

[V, D] = eig((Pxy + Pxy.') / 2);
theta = linspace(0, 2 * pi, 50);
unitCircle = [cos(theta); sin(theta)];
shape = nsig * V * sqrt(D) * unitCircle;
xy = shape + mu(:);

patch(xy(1, :), xy(2, :), color, 'FaceAlpha', alphaVal, 'EdgeColor', color, 'LineWidth', 0.6);
end
