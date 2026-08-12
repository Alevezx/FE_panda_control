% Load data
t_exp      = load('t_BASE_exp.txt');
tau_exp    = load('tau_BASE_exp.txt');  % adjust filenames
t_sim      = load("../traj/t.txt");  % if you used the same t vector for simulation
tau_sim    = load('torques_with_friction_out.txt');

% They may have different number of rows — interpolate sim onto exp time grid
tau_sim_interp = zeros(length(t_exp), 7);
t_sim_vec = linspace(t_sim(1), t_sim(end), size(tau_sim, 1));

for j = 1:7
    tau_sim_interp(:, j) = interp1(t_sim_vec, tau_sim(:, j), t_exp, 'linear');
end

% Compute residual and mean offset per joint
residual = tau_exp - tau_sim_interp;
offset   = mean(residual, 1);  % 1x7 vector

fprintf('Offset per joint:\n');
for j = 1:7
    fprintf('  Joint %d: %.4f Nm\n', j, offset(j));
end

% Optional: plot residuals to check if offset is truly constant
figure;
for j = 1:7
    subplot(4, 2, j);
    plot(t_exp, residual(:, j)); hold on;
    yline(offset(j), 'r--', sprintf('mean = %.3f', offset(j)));
    title(sprintf('Joint %d residual', j));
    xlabel('Time [s]'); ylabel('Nm');
    grid on;
end
sgtitle('Residual (real - sim) per joint');