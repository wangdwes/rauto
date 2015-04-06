% Triangulates stereo points
%   M1 - 3x4 camera matrix for p1
%   p1 - 2xN points
%   M2 - 3x4 camera matrix for p2
%   p2 - 2xN points
%   P - 3xN triangulated points
function P = triangulate(M1, p1, M2, p2)
    % Size of points
    N = size(p1, 2);
    % Least square matrix
    A = zeros(2, 4); % N-by-4
    % Result 3D points
    P = zeros(3, N);

    for i=1:N
        x = p1(1, i); y = p1(2, i);
        xp = p2(1, i); yp = p2(2, i);
        A(1, :) = y*M1(3, :) - M1(2, :);
        A(2, :) = M1(1, :) - x*M1(3, :);
        A(3, :) = yp*M2(3, :) - M2(2, :);
        A(4, :) = M2(1, :) - xp*M2(3, :);
        [~, ~, V] = svd(A);
        p = V(:, end);
        p = p / p(end);
        P(:, i) = p(1:3);
    end
    
end