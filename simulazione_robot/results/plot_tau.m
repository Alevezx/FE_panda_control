clear
close all
% plot_data.m
% Plots an nx7 data matrix against a time vector,
% adds vertical lines at specified times, and configures legend/axes.

% --- Replace these with your data ---
t = readmatrix("../traj/t.txt");                       % nx1 time vector
data = readmatrix("torques_with_friction_out.txt");                      % nx7 data matrix
% -------------------------------------------

% Line colours and labels for the 7 data channels
colors = lines(7);
channelNames = arrayfun(@(k) sprintf('Joint %d', k), 1:7, 'UniformOutput', false);

% Vertical-line properties
vlineTimes  = [3, 4, 7, 9, 11];
vlineColor  = [0.4 0.4 0.4];   % dark grey
vlineStyle  = '--';
vlineWidth  = 1.2;

figure;
hold on;

% --- Plot the 7 data channels ---
hData = gobjects(1, 7);
for k = 1:7
    hData(k) = plot(t, data(:, k), ...
        'Color',     colors(k, :), ...
        'LineWidth', 1.5, ...
        'DisplayName', channelNames{k});
end

% --- Draw vertical lines ---
hVline = gobjects(1, numel(vlineTimes));
for i = 1:numel(vlineTimes)
    hVline(i) = xline(vlineTimes(i), ...
        vlineStyle, ...
        'Color',     vlineColor, ...
        'LineWidth', vlineWidth, ...
        'DisplayName', sprintf('t = %g', vlineTimes(i)));
end

% --- Axes formatting ---
axis([0 11 -50 50]);
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Virtual robot torques with friction');
grid on;

% --- Legend: data channels + vertical lines ---
legend([hData, hVline], 'Location', 'bestoutside');

hold off;