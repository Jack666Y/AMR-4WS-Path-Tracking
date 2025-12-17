function path = backtrackPath(nodes, parent, goalIdx)
    % Follow parent pointers from goal back to start
    idx = goalIdx;
    path = nodes(idx,:);
    while parent(idx) ~= 0
        idx = parent(idx);
        path = [nodes(idx,:); path]; %#ok<AGROW>
    end
end
