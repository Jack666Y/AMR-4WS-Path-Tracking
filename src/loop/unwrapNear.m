function sU = unwrapNear(sVal, sRef, L)
    candidates = [sVal-L, sVal, sVal+L];
    [~,k] = min(abs(candidates - sRef));
    sU = candidates(k);
end
