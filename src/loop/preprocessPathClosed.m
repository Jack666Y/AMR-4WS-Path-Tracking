function Path = preprocessPathClosed(pathXY)
    % Build arc-length map for closed loop interpolation
    if norm(pathXY(end,:) - pathXY(1,:)) > 1e-9
        pathXY(end+1,:) = pathXY(1,:);
    end

    d = sqrt(sum(diff(pathXY,1,1).^2,2));
    keep = [true; d > 1e-12];
    pathXY = pathXY(keep,:);

    d  = sqrt(sum(diff(pathXY,1,1).^2,2));
    s  = [0; cumsum(d)];
    L  = s(end);

    Path.XY  = pathXY;
    Path.s   = s;
    Path.L   = L;
    Path.XY2 = [pathXY; pathXY(2:end,:)];
    Path.s2  = [s; s(2:end)+L];
end
