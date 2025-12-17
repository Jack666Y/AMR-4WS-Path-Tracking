function XY = makePath_OvalTrack()
    % Closed oval-like loop with mild waviness
    a = 6.8; b = 3.8;
    amp = 0.8; nW = 2;

    t = linspace(0, 2*pi, 900)';

    x = a*cos(t);
    y = b*sin(t);

    x = x + amp*cos(nW*t).*cos(t);
    y = y + amp*cos(nW*t).*sin(t);

    th = deg2rad(10);
    R  = [cos(th) -sin(th); sin(th) cos(th)];
    XY = (R*[x'; y'])';
    XY(:,1) = XY(:,1) + 10.0;
    XY(:,2) = XY(:,2) + 7.0;

    XY(end+1,:) = XY(1,:);
end
