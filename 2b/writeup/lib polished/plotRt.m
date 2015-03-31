% Plots trajectory given rotation matrices and translation vectors
%   Rs - 3x3xN rotation matrices between each pair of successive frames
%   ts - 3xN translation vectors between each pair of successive frames
function plotRt(Rs, ts)
    N = size(Rs, 3);
    
    Ts = zeros(4, 4, N);
    for i=1:N
        Ts(:, :, i) = inv([Rs(:, :, i), ts(:, i); 0 0 0 1]);
    end
    plotT(Ts);
end