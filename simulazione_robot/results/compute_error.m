%% Load data
t_exp   = load('t_BASE_exp.txt');          % 330x1
tau_exp = readmatrix('tau_BASE_exp.txt');  % 330x7, tab-separated

t_sim   = load('../traj/t.txt');                   % 551x1
tau_sim = readmatrix('torques_out.txt');   % 551x7, comma-separated
tau_Gaz = readmatrix('torques_with_friction_out_Gaz.txt');
tau_Scalera = readmatrix('torques_with_friction_out_Scalera.txt');

%% Interpolate simulated torques onto the experimental time grid
tau_sim_interp = zeros(length(t_exp), 7);
for j = 1:7
    tau_sim_interp(:, j) = interp1(t_sim, tau_sim(:, j), t_exp, 'linear', 'extrap');
    tau_Gaz_interp(:, j) = interp1(t_sim, tau_Gaz(:, j), t_exp, 'linear', 'extrap');
    tau_Scalera_interp(:, j) = interp1(t_sim, tau_Scalera(:, j), t_exp, 'linear', 'extrap');
end

%% Compute error
error_sim = tau_exp - tau_sim_interp;          % 330x7

rmse = sqrt(mean(error_sim.^2, 1));            % RMSE per joint
mae  = mean(abs(error_sim), 1);                % Mean Absolute Error per joint
max_err = max(abs(error_sim), [], 1);          % Max absolute error per joint
mean_err = mean(error_sim, 1);                 % Mean signed error (bias/offset) per joint

% Relative RMSE (%) - normalize by the RMS of the real signal, not instantaneous value
rel_rmse = rmse ./ sqrt(mean(tau_exp.^2, 1)) * 100;

% Relative mean error (%) - normalize by mean absolute real torque
rel_mean_err = mean_err ./ mean(abs(tau_exp), 1) * 100;

fprintf('\nSenza attrito:\nGiunto |   RMS   | Err max | Err medio | RMS rel | Err medio rel \n');
for j = 1:7
    fprintf('   %d   | %7.4f | %7.4f |  %7.4f  |  %6.2f |   %6.2f\n', ...
        j, rmse(j), max_err(j), mean_err(j), rel_rmse(j), rel_mean_err(j));
end

error_Gaz = tau_exp - tau_Gaz_interp;          % 330x7

rmse = sqrt(mean(error_Gaz.^2, 1));            % RMSE per joint
mae  = mean(abs(error_Gaz), 1);                % Mean Absolute Error per joint
max_err = max(abs(error_Gaz), [], 1);          % Max absolute error per joint
mean_err = mean(error_Gaz, 1);                 % Mean signed error (bias/offset) per joint
% Relative RMSE (%) - normalize by the RMS of the real signal, not instantaneous value
rel_rmse = rmse ./ sqrt(mean(tau_exp.^2, 1)) * 100;

% Relative mean error (%) - normalize by mean absolute real torque
rel_mean_err = mean_err ./ mean(abs(tau_exp), 1) * 100;

fprintf('\nCon attrito secondo Gaz:\nGiunto |   RMS   | Err max | Err medio | RMS rel | Err medio rel \n');
for j = 1:7
    fprintf('   %d   | %7.4f | %7.4f |  %7.4f  |  %6.2f |   %6.2f\n', ...
        j, rmse(j), max_err(j), mean_err(j), rel_rmse(j), rel_mean_err(j));
end

error_Scalera = tau_exp - tau_Scalera_interp;          % 330x7

rmse = sqrt(mean(error_Scalera.^2, 1));            % RMSE per joint
mae  = mean(abs(error_Scalera), 1);                % Mean Absolute Error per joint
max_err = max(abs(error_Scalera), [], 1);          % Max absolute error per joint
mean_err = mean(error_Scalera, 1);                 % Mean signed error (bias/offset) per joint
% Relative RMSE (%) - normalize by the RMS of the real signal, not instantaneous value
rel_rmse = rmse ./ sqrt(mean(tau_exp.^2, 1)) * 100;

% Relative mean error (%) - normalize by mean absolute real torque
rel_mean_err = mean_err ./ mean(abs(tau_exp), 1) * 100;

fprintf('\nCon attrito secondo Scalera:\nGiunto |   RMS   | Err max | Err medio | RMS rel | Err medio rel \n');
for j = 1:7
    fprintf('   %d   | %7.4f | %7.4f |  %7.4f  |  %6.2f |   %6.2f\n', ...
        j, rmse(j), max_err(j), mean_err(j), rel_rmse(j), rel_mean_err(j));
end
%% Plot error per joint
figure;
for j = 1:7
    subplot(4, 2, j);
    plot(t_exp, error_sim(:, j), 'b');
    hold on;
    plot(t_exp, error_Gaz(:, j), 'r');
    plot(t_exp, error_Scalera(:, j), 'g');
    title(sprintf('Errore giunto %d', j));
    xlabel('Tempo [s]'); ylabel('[Nm]');
    grid on;
end
sgtitle('Errori di coppia');
lgd = legend({'Senza attrito', 'Attrito secondo Gaz', 'Attrito secondo Scalera'});
lgd.Position = [0.75 0.11 0.1 0.1];

%% Overlay comparison plot per joint
figure;
for j = 1:7
    subplot(4, 2, j);
    plot(t_exp, tau_exp(:, j), 'y', 'LineWidth', 1.2); hold on;
    plot(t_exp, tau_sim_interp(:, j), 'b', 'LineWidth', 1.2);
    plot(t_exp, tau_Gaz_interp(:, j), 'r', 'LineWidth', 1.2);
    plot(t_exp, tau_Scalera_interp(:, j), 'g', 'LineWidth', 1.2);
    title(sprintf('Giunto %d', j));
    xlabel('Tempo [s]'); ylabel('[Nm]');
    grid on;
end
sgtitle('Confronto tra coppie reali e simulate');
lgd = legend('Reale', 'Senza attrito', 'Attrito secondo Gaz', 'Attrito secondo Scalera');
lgd.Position = [0.75 0.11 0.1 0.1];