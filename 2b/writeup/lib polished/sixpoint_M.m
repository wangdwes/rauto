% Computes the least square camera matrix estimate using six point correspondences.
%   x - 2x6 2D image points
% 	X - 3x6 3D world points
% 	M - the estimated camera matrix (3x4)

function M = sixpoint_M(p, P)
    % Size of points
    minN = 6;
    N = size(p, 2);
    % Least square matrix
    A = zeros(N, 12); % N-by-12
    
    % Check if N point pairs exist
    if (size(p, 2) < minN || size(P, 2) < minN)
        fprintf('ERROR: Need at least %d points!\n', minN);
        M = 0;
        return
    end
    
    % Make homogeneous
    Ph = [P; ones(1, N)];

    for i=1:N
        xp = p(1, i); yp = p(2, i);
        Pi = Ph(:, i);
        z = zeros(1, 4);
        A(2*i-1, :) = [Pi', z, -xp*Pi'];
        A(2*i, :) = [z, Pi', -yp*Pi'];
    end
    
    % Linear solution
    [~, ~, V] = svd(A);
    % Get the singular vector corresponding to the smallest singular value
    m = V(:, end);
    
    % Construct the camera matrix to 3x4
    M = reshape(m, 4, 3)';
end