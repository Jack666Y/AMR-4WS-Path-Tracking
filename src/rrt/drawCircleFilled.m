function h = drawCircleFilled(cx, cy, r)
    t = linspace(0,2*pi,100);
    h = fill(cx + r*cos(t), cy + r*sin(t), 'k', 'EdgeColor','k');
end
