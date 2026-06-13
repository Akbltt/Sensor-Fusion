function animate_results(truth, sensors, dr, ekf, cfg)
%ANIMATE_RESULTS Create animated visualization of navigation during GPS outage.
%
% Generates GIF showing:
% - Vehicle trajectory evolution
% - Ground truth path
% - EKF estimated path
% - IMU dead reckoning path
% - GPS measurements
% - GPS outage interval (highlighted)

t = truth.t;
gpsOutageStart = sensors.gpsOutageStart;
gpsOutageEnd = sensors.gpsOutageEnd;

% Downsampling for animation speed with a hard cap on frame count.
maxFrames = 450;
if isfield(cfg, 'animMaxFrames')
    maxFrames = cfg.animMaxFrames;
end
adaptiveDownsample = max(cfg.animDownsample, ceil(numel(t) / maxFrames));
idxAnim = 1:adaptiveDownsample:numel(t);
nFrames = numel(idxAnim);

% Prepare figure
fig = figure('Color', 'w', 'Position', [100 100 1000 800], 'Visible', 'off');
ax = axes('Parent', fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'X Position [m]', 'FontSize', 11);
ylabel(ax, 'Y Position [m]', 'FontSize', 11);

xMin = min(truth.state(:, 1)) - 10;
xMax = max(truth.state(:, 1)) + 10;
yMin = min(truth.state(:, 2)) - 10;
yMax = max(truth.state(:, 2)) + 10;
xlim(ax, [xMin, xMax]);
ylim(ax, [yMin, yMax]);

% Create graphics once, then update data each frame.
hTruth = plot(ax, NaN, NaN, 'k-', 'LineWidth', 2.0, 'DisplayName', 'Ground Truth');
hEkf = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF Solution');
hDr = plot(ax, NaN, NaN, 'r--', 'LineWidth', 1.5, 'DisplayName', 'IMU Dead Reckoning');
hGps = scatter(ax, NaN, NaN, 40, 'g^', 'filled', 'DisplayName', 'GPS Measurements');

hTruthNow = scatter(ax, NaN, NaN, 120, 'ko', 'filled', 'LineWidth', 1.5, 'DisplayName', 'Truth (current)');
hEkfNow = scatter(ax, NaN, NaN, 90, 'bs', 'filled', 'DisplayName', 'EKF (current)');
hDrNow = scatter(ax, NaN, NaN, 90, 'rs', 'filled', 'DisplayName', 'DR (current)');

legend(ax, 'Location', 'best', 'FontSize', 10);

gifFile = fullfile(cfg.resultsDir, cfg.gifName);
if isfile(gifFile)
    delete(gifFile);
end

fprintf('  Animation frames: %d (downsample=%d)\n', nFrames, adaptiveDownsample);

for frame = 1:nFrames
    k = idxAnim(frame);
    tk = t(k);

    % Update trajectories up to current time.
    set(hTruth, 'XData', truth.state(1:k, 1), 'YData', truth.state(1:k, 2));
    set(hEkf, 'XData', ekf.state(1:k, 1), 'YData', ekf.state(1:k, 2));
    set(hDr, 'XData', dr.state(1:k, 1), 'YData', dr.state(1:k, 2));

    % Update GPS measurements up to current time.
    gpsIdxUpTo = sensors.tGps <= tk;
    set(hGps, 'XData', sensors.gps(gpsIdxUpTo, 1), 'YData', sensors.gps(gpsIdxUpTo, 2));

    % Update current position markers.
    set(hTruthNow, 'XData', truth.state(k, 1), 'YData', truth.state(k, 2));
    set(hEkfNow, 'XData', ekf.state(k, 1), 'YData', ekf.state(k, 2));
    set(hDrNow, 'XData', dr.state(k, 1), 'YData', dr.state(k, 2));

    % Title with time and phase indicator
    if tk < gpsOutageStart
        phase = 'GPS Available';
        titleColor = [0.2, 0.7, 0.2];
    elseif tk < gpsOutageEnd
        phase = 'GPS OUTAGE';
        titleColor = [0.9, 0.2, 0.2];
    else
        phase = 'GPS Recovery';
        titleColor = [0.2, 0.2, 0.7];
    end
    
    title(ax, sprintf('Time: %.1f s [%s]', tk, phase), 'FontSize', 14, 'FontWeight', 'bold', 'Color', titleColor);
    
    % Capture frame
    drawnow limitrate;
    frame_img = getframe(fig);
    img = frame2im(frame_img);
    [img_index, cmap] = rgb2ind(img, 256);
    
    % Write to GIF
    if frame == 1
        imwrite(img_index, cmap, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
    else
        imwrite(img_index, cmap, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
    
    % Progress indicator
    if mod(frame, max(1, floor(nFrames / 10))) == 0
        fprintf('  Animation progress: %d/%d frames\n', frame, nFrames);
    end
end

close(fig);
fprintf('Animation saved: %s\n', gifFile);

end
