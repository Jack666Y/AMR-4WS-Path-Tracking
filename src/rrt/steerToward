function qNew = steerToward(qNear, qRand, step)
    v = qRand - qNear;
    d = norm(v);
    if d < 1e-9
        qNew = qNear;
    else
        qNew = qNear + (step/d)*v;
    end
end
