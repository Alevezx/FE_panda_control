clear
close all
% plot_data.m
% Plots an nx7 data matrix against a time vector,
% adds vertical lines at specified times, and configures legend/axes.

% --- Replace these with your data ---
t = readmatrix("../traj/t.txt");                       % nx1 time vector
data = readmatrix("torques_out_friction.txt");                      % nx7 data matrix
% -------------------------------------------

% Line colours and labels for the 7 data channels
colors = lines(7);
channelNames = arrayfun(@(k) sprintf('Giunto %d', k), 1:7, 'UniformOutput', false);

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

% --- Axes formatting ---
axis([0 11 -50 50]);
xlabel('Tempo [s]');
ylabel('Coppia [Nm]');
title('Coppie ai giunti simulate con compensazione inerziale');
grid on;

% --- Legend: data channels + vertical lines ---
legend(hData, 'Location', 'bestoutside');

hold off;