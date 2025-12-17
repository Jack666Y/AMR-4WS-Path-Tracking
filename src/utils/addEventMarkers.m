function addEventMarkers(tvec, evtMark)
    idx = find(evtMark);
    for ii = 1:numel(idx)
        xline(tvec(idx(ii)), '--', 'LineWidth', 1.0);
    end
end
