S = readstruct('dist_segment.xml');

time = [S.keypoint.timeAttribute];
stop_duration = [S.keypoint.stop_duration];

figure
plot(time, stop_duration)
grid on
xlabel('Time (s)')
ylabel('Stop duration (s)')