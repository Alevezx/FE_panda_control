q_f = 1;
q_cmd = [0];
q_d = 0;
t = 0;

while(t < 5)
    t = t+ .001;
    s = t/move_duration;
    s = min(s, 1);
    alpha = s*s*(3-2*s);
    q_cmd = [q_cmd; q_d + alpha*(q_f - q_d)];
    % q_d = q_cmd(end);

end