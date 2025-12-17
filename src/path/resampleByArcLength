function XYr = resampleByArcLength(XY, ds)
    dXY = diff(XY,1,1);
    segL = sqrt(sum(dXY.^2,2));
    keep = [true; segL > 1e-12];   
    XY = XY(keep,:);

    d  = sqrt(sum(diff(XY,1,1).^2,2));
    s  = [0; cumsum(d)];

    [s_u, ia] = unique(s, 'stable');
    XY_u = XY(ia,:);

    s2 = (0:ds:s_u(end))';
    x2 = interp1(s_u, XY_u(:,1), s2, 'linear');
    y2 = interp1(s_u, XY_u(:,2), s2, 'linear');
    XYr = [x2 y2];
end
