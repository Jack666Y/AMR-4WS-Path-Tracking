function pathXY = makePath_Scurve()
    x1 = linspace(0, 4, 140);  y1 = zeros(size(x1));
    x2 = linspace(4, 12, 360); x2 = x2(2:end);
    y2 = 1.2*tanh(1.2*(x2-6)) - 1.2*tanh(1.2*(x2-10));
    x3 = linspace(12, 20, 360); x3 = x3(2:end);
    y3 = y2(end) - (1.2*tanh(1.2*(x3-14)) - 1.2*tanh(1.2*(x3-18)));
    x4 = linspace(20, 24, 160); x4 = x4(2:end);
    y4 = y3(end)*ones(size(x4));

    x = [x1 x2 x3 x4]';
    y = [y1 y2 y3 y4]';

    R = [cosd(10) -sind(10); sind(10) cosd(10)];
    XY = (R*[x'; y'])';
    XY(:,1) = XY(:,1) + 2;
    XY(:,2) = XY(:,2) + 2;

    pathXY = resampleByArcLength(XY, 0.04);
end
