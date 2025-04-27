function [u] = zeroInput(t)
    if t >= 0
        u = 0;
    else
        u = t;
    end
end