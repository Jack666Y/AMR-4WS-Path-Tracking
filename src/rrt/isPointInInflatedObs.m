function inside = isPointInInflatedObs(p, obs, margin)
    inside = false;
    for i = 1:size(obs,1)
        cx = obs(i,1); cy = obs(i,2); r = obs(i,3) + margin;
        if (p(1)-cx)^2 + (p(2)-cy)^2 <= r^2
            inside = true; return;
        end
    end
end
