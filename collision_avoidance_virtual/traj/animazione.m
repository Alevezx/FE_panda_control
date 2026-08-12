clear
close all

q = load("q.txt");
figure
rbt = importrobot("panda.urdf");
rbt.DataFormat = 'row';

r = rateControl(10);

% Animazione
show(rbt, q(1,:), PreservePlot=false);
hold on
pos = tform2trvec(getTransform(rbt, q(1,:), "panda_link8"));
pl = plot3(pos(1), pos(2), pos(3));
ginput;

% mostra l'animazione
for k = 2:size(q,1)
    show(rbt, q(k,:), PreservePlot=false);
    pos = tform2trvec(getTransform(rbt, q(k,:), "panda_link8"));
    pl.XData(k) = pos(1);
    pl.YData(k) = pos(2);
    pl.ZData(k) = pos(3);
    waitfor(r);
end