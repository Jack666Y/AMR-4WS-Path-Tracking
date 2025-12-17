function h = drawCircleOutline(cx, cy, r, colorChar)
    t = linspace(0,2*pi,140);
    h = plot(cx + r*cos(t), cy + r*sin(t), colorChar, 'LineWidth', 1.5);
end
