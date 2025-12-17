function [x, y, th] = interpRefByS(Path, sQuery)
    % Interpolate position and tangent by arc-length s
    sQ = wrapS(sQuery, Path.L);

    x = interp1(Path.s2, Path.XY2(:,1), sQ, 'linear');
    y = interp1(Path.s2, Path.XY2(:,2), sQ, 'linear');

    ds = 0.06;
    xb = interp1(Path.s2, Path.XY2(:,1), wrapS(sQ+ds, Path.L), 'linear');
    yb = interp1(Path.s2, Path.XY2(:,2), wrapS(sQ+ds, Path.L), 'linear');
    th = atan2(yb-y, xb-x);
end
