S = readstruct('dist_segment.xml');

time = [S.keypoint.timeAttribute];
stop_duration = [S.keypoint.stop_duration];

figure
plot(time, stop_duration,'o')
hold on
plot(time, stop_duration)
grid on
xlabel('Time (s)')
ylabel('Stop duration (s)')
% axis([0 12 0 0.5])