function [u] = genImpulseInput(tVec, Amp, impulseTime)
    u = NaN(size(tVec));

    for ii = 1:1:length(u)
        if tVec(ii) == impulseTime
            u(ii) = Amp;
        else
            u(ii) = 0;
        end
    end
end