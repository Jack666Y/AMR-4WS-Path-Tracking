function [idx, qNear] = nearestNode(nodes, q)
    dif = nodes - q;
    [~, idx] = min(sum(dif.^2,2));
    qNear = nodes(idx,:);
end
