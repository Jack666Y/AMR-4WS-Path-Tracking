function [xref, yref, thref, lastIdx] = getRefOnPathForward(pos, pathXY, Ld, lastIdx)
    N = size(pathXY,1);

    searchStart = max(1, lastIdx - 10);
    seg = pathXY(searchStart:N,:) - pos;
    d2  = sum(seg.^2, 2);
    [~, k] = min(d2);
    idx = searchStart + k - 1;

    idx = max(idx, lastIdx);
    lastIdx = idx;

    j = idx;
    while j < N && hypot(pathXY(j,1)-pos(1), pathXY(j,2)-pos(2)) < Ld
        j = j + 1;
    end
    j = min(j, N);

    xref = pathXY(j,1);
    yref = pathXY(j,2);

    if j < N
        dp = pathXY(j+1,:) - pathXY(j,:);
    else
        dp = pathXY(j,:) - pathXY(max(j-1,1),:);
    end
    thref = atan2(dp(2), dp(1));
end
