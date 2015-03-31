% Calculates the epipole related to the fundamental matrix.
% @params:
%     - F: the fundmental matrix
%     - e: the calculated epipole

function e = epipole(F)
    [~, ~, V] = svd(F);
    v3 = V(:,end);
    e = v3 / v3(3);
end