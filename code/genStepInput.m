function [u] = genStepInput(tVec, Amp, stepTime)
    
    u = NaN(size(tVec));

    for ii = 1:1:length(u)
        if tVec(ii) > stepTime
            u(ii) = Amp;
        else
            u(ii) = 0;
        end
    end
end