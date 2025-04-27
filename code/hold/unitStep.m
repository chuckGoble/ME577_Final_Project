function [u] = unitStep(t)
    if t > 0
        u = 1;
    else
        u = 0;
    end
end