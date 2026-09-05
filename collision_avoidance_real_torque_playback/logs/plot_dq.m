filename = 'dq_robot.xml';

doc = xmlread(filename);
keypoints = doc.getElementsByTagName('keypoint');

N = keypoints.getLength;

time = zeros(N,1);
q = zeros(N,7);    % one column for each point id

for i = 0:N-1
    kp = keypoints.item(i);

    % Time
    time(i+1) = str2double(char(kp.getAttribute('time')));

    % Seven point values
    pts = kp.getElementsByTagName('point');
    for j = 0:6
        q(i+1,j+1) = str2double(char(pts.item(j).getTextContent));
    end
end

% Plot all seven trajectories
figure
t = 0:.001:(N-1)/1000;
plot(t', q, 'LineWidth', 1.5)

grid on
xlabel('Time (s)')
ylabel('Joint speed')
title('Robot joint speed')

legend('Joint 0','Joint 1','Joint 2','Joint 3', ...
    'Joint 4','Joint 5','Joint 6', ...
    'Location','best')

hold on;
% vertical lines
% vlineTimes  = [3, 4, 7, 9, 11];
% vlineColor  = [0.4 0.4 0.4];   % dark grey
% vlineStyle  = '--';
% vlineWidth  = 1.2;
% 
% hVline = gobjects(1, numel(vlineTimes));
% for i = 1:numel(vlineTimes)
%     hVline(i) = xline(vlineTimes(i), ...
%         vlineStyle, ...
%         'Color',     vlineColor, ...
%         'LineWidth', vlineWidth, ...
%         'DisplayName', sprintf('t = %g', vlineTimes(i)));
% end