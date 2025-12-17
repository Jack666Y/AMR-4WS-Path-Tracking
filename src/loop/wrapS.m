function sW = wrapS(s, L)
    sW = mod(s, L);
    if sW < 0, sW = sW + L; end
end
