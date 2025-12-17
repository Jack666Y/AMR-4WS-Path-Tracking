function idx = windowIndices(s, sA, sB, L)
    sA = wrapS(sA, L); sB = wrapS(sB, L);
    if sA <= sB
        idx = find(s >= sA & s <= sB);
    else
        idx = [find(s >= sA); find(s <= sB)];
    end
    if isempty(idx)
        idx = (1:numel(s))';
    end
end
