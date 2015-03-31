% Estimates the fundamental matrix using least-square solution.
% @params:
%     - pts1: 2xN points from camera 1 image
%     - pts2: 2xN points from camera 2 image
%     - normalization_constant: the larger dimension of a camera image
%     - F: the estimated fundamental matrix

function F = eightpoint_norm(pts1, pts2, normalization_constant)
    % Size of points
    N = size(pts1, 2);
    % Least square matrix
    A = zeros(N, 9); % N-by-9
    
    % Normalize the input coordinates
    T = [1/normalization_constant, 0, 0; ...
         0, 1/normalization_constant, 0; ...
         0, 0, 1];
    pts1n = T(1:2, 1:2) * pts1;
    pts2n = T(1:2, 1:2) * pts2;

    % Construct the least square matrix
    for i=1:N
        x  = pts1n(1, i); y  = pts1n(2, i);
        xp = pts2n(1, i); yp = pts2n(2, i);
        A(i, :) = [xp*x, xp*y, xp, yp*x, yp*y, yp, x, y, 1];
    end
    
    % Linear solution
    [~, ~, V] = svd(A);
    % Get the singular vector corresponding to the smallest singular value
    f = V(:, end);
    
    % Construct the fundamental matrix to 3x3
    F = reshape(f, 3, 3)';
    
    % Enforce singularity constraint
    [U, S, V] = svd(F);
    S(3, 3) = 0;
    F = U * S * V';
    for i=1:size(pts1, 2)
      [pts2n(:,i); 1]'*F*[pts1n(:,i); 1];
    end

    % Denormalize
    F = T' * F * T;
    F = F / F(end);
end