% Calculates the least square solution.
%	M - Input matrix
%   vn - Normalized least square vector solution from M
%   vend - Unnormalized least square vector solution from M

function [vn, vend] = lsvect(M)
    [~, ~, V] = svd(M);
    vend = V(:,end);
    vn = vend / vend(end);
end