function [xref, yref, thref, sLast] = getRefOnPathForwardClosed(pos, Path, Ld, sLast)
    % Loop-safe reference using progress sLast (prevents index jumping)
    pos = pos(:).';
    L = Path.L;

    backWin = 1.0;
    fwdWin  = 3.0;

    s0 = wrapS(sLast, L);
    sA = s0 - backWin;
    sB = s0 + fwdWin;

    idx = windowIndices(Path.s, sA, sB, L);
    dif = Path.XY(idx,:) - pos;
    [~, kk] = min(sum(dif.^2,2));
    iNear = idx(kk);

    sNear = Path.s(iNear);
    sNear = unwrapNear(sNear, s0, L);
    sLast = sNear;

    sRef = wrapS(sNear + Ld, L);
    [xref, yref, thref] = interpRefByS(Path, sRef);
end
