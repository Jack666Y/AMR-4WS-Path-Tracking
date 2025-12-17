function s0 = projectToPathS(pos, Path)
    % Initial progress estimate using nearest vertex
    pos = pos(:).';
    dif = Path.XY - pos;
    [~, idx] = min(sum(dif.^2,2));
    s0 = Path.s(idx);
end
