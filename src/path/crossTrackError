function [idxNear, e_ct, th_path] = crossTrackError(pos, pathXY)
    dif = pathXY - pos;
    [~, idxNear] = min(sum(dif.^2,2));

    N = size(pathXY,1);
    i0 = max(1, min(N-1, idxNear));
    dp = pathXY(i0+1,:) - pathXY(i0,:);
    th_path = atan2(dp(2), dp(1));

    v = [cos(th_path), sin(th_path)];
    n = [-v(2), v(1)];
    e_ct = n * (pos(:) - pathXY(i0,:).');
end
