function ok = isSegmentCollisionFree(p0, p1, obs, margin, ds)
    % Discretized segment collision check with inflated obstacles
    v = p1 - p0;
    L = norm(v);
    if L < 1e-12
        ok = ~isPointInInflatedObs(p0, obs, margin);
        return;
    end
    n = max(2, ceil(L/ds));
    ok = true;
    for k = 0:n
        tt = k/n;
        p = p0 + tt*v;
        if isPointInInflatedObs(p, obs, margin)
            ok = false; return;
        end
    end
end
