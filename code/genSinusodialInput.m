function [u] = genSinusodialInput(tVec, timeShift, amp, freq)
    
    omega = freq .* 2 .* pi;

    u = NaN(size(tVec));

    for ii = 1:1:length(u)
        if tVec(ii) > timeShift
            u(ii) = amp.*sin(omega.*(tVec(ii)-timeShift));
        else
            u(ii) = 0;
        end
    end
end