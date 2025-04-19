function [y] = moveOutput(y, shift)
    for ii = 1:1:size(y, 1)
        y(ii, :) = y(ii, :) + shift;
    end
end