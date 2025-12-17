function obs = generateObstacles(n, bounds, protectedPts, rMin, rMax)
    % Random circular obstacles with simple spacing constraints
    xmin=bounds(1); xmax=bounds(2); ymin=bounds(3); ymax=bounds(4);
    obs = zeros(0,3);
    tries = 0;

    while size(obs,1) < n && tries < 8000
        tries = tries + 1;
        r  = rMin + (rMax-rMin)*rand();
        cx = xmin + r + (xmax-xmin-2*r)*rand();
        cy = ymin + r + (ymax-ymin-2*r)*rand();

        ok = true;
        for k = 1:size(protectedPts,1)
            if hypot(cx-protectedPts(k,1), cy-protectedPts(k,2)) < (r + 1.5)
                ok = false; break;
            end
        end
        if ~ok, continue; end

        for j = 1:size(obs,1)
            if hypot(cx-obs(j,1), cy-obs(j,2)) < (r + obs(j,3) + 0.3)
                ok = false; break;
            end
        end
        if ~ok, continue; end

        obs(end+1,:) = [cx cy r]; %#ok<AGROW>
    end
end
