function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
