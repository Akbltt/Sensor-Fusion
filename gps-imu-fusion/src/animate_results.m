function animate_results(truth, sensors, dr, ekf, cfg)
%ANIMATE_RESULTS Creates trajectory animation and exports GIF and optional MP4.

if ~exist(cfg.resultsDir, 'dir')
    mkdir(cfg.resultsDir);
end

gifPath = fullfile(cfg.resultsDir, cfg.gifName);
mp4Path = fullfile(cfg.resultsDir, cfg.mp4Name);
gifDelay = max(0.01, (cfg.dtImu * cfg.animDownsample) / cfg.gifSpeedMultiplier);

fig = figure('Color', 'w', 'Position', [80 80 900 700]);
ax = axes(fig); hold(ax, 'on');
plot(ax, truth.state(:, 1), truth.state(:, 2), 'k--', 'LineWidth', 1.2);

gpsSc = plot(ax, nan, nan, '.', 'Color', [0.4 0.4 0.9], 'MarkerSize', 12);
drLine = plot(ax, nan, nan, '-', 'Color', [0.9 0.35 0.15], 'LineWidth', 1.5);
ekfLine = plot(ax, nan, nan, '-', 'Color', [0.1 0.55 0.2], 'LineWidth', 1.8);
trueCar = plot(ax, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

xlabel(ax, 'X position [m]'); ylabel(ax, 'Y position [m]');
title(ax, 'GPS-IMU Fusion EKF Animation');
grid(ax, 'on'); axis(ax, 'equal');
legend(ax, {'Ground Truth (path)', 'GPS Measurements', 'IMU DR', 'EKF Fusion', 'True Vehicle'}, 'Location', 'best');

pad = 10;
xMin = min(truth.state(:, 1)) - pad; xMax = max(truth.state(:, 1)) + pad;
yMin = min(truth.state(:, 2)) - pad; yMax = max(truth.state(:, 2)) + pad;
xlim(ax, [xMin xMax]); ylim(ax, [yMin yMax]);

doMp4 = cfg.writeMp4;
if doMp4
    try
        vw = VideoWriter(mp4Path, 'MPEG-4');
        vw.FrameRate = round(1 / (cfg.dtImu * cfg.animDownsample));
        open(vw);
    catch
        doMp4 = false;
    end
end

frameCount = 0;
for k = 1:cfg.animDownsample:numel(truth.t)
    frameCount = frameCount + 1;

    set(drLine, 'XData', dr.state(1:k, 1), 'YData', dr.state(1:k, 2));
    set(ekfLine, 'XData', ekf.state(1:k, 1), 'YData', ekf.state(1:k, 2));
    set(trueCar, 'XData', truth.state(k, 1), 'YData', truth.state(k, 2));

    gpsValid = ~isnan(sensors.gps(1:k, 1));
    set(gpsSc, 'XData', sensors.gps(gpsValid, 1), 'YData', sensors.gps(gpsValid, 2));

    title(ax, sprintf('GPS-IMU Fusion EKF Animation | t = %.1f s', truth.t(k)));
    drawnow;

    fr = getframe(fig);
    [imind, cm] = rgb2ind(frame2im(fr), 256);
    if frameCount == 1
        imwrite(imind, cm, gifPath, 'gif', 'LoopCount', inf, 'DelayTime', gifDelay);
    else
        imwrite(imind, cm, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
    end

    if doMp4
        writeVideo(vw, fr);
    end
end

if doMp4
    close(vw);
end

close(fig);
end
