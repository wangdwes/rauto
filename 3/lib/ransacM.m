% Computes the least square camera matrix estimate using RANSAC.
%   x - Nx6 2D image points
%   X - Nx6 3D world points
%   M - Estimated camera matrix (3x4)
%   Inliers - Inlier point indices within a set threshold

function [M, inliers] = ransacM(p, P)
    % Constants
    nRANSAC = 300; % Number of RANSAC iterations 15000?
    nPoints = 6; % Number of points for each ransac
    thres = 0.5;
    N = size(p, 2); % Size of points
    
    %fprintf('Running RANSAC %d times with threshold %f\n', nRANSAC, thres);
    
    % Variables
    inliers = zeros(1, N); % Store inliers
    nInliers = 0; % Number of inliers
    
    % RANSAC iterations
    for iRANSAC=1:nRANSAC
        % Randomly select 6 points and compute M using six point algorithm
        randind = randperm(N, nPoints);
        tmpM = sixpoint_M(p(:, randind), P(:, randind));
        
        % Record results
        tmpInliers = zeros(1, N);
        tmpN = 0;
        
        for i=1:N
            Pi = P(:, i);
            pi = p(:, i);
            reproj = tmpM * [Pi; 1]; % Reprojection
            reproj = reproj * (pi(1) / reproj(1)); % Scale to match point on first element
            repError = norm(pi - reproj(1:2, :)); % Find norm/error in inhomogeneous coordinates
            if (repError < thres)
                tmpN = tmpN + 1;
                tmpInliers(tmpN) = i;
            end
        end

        % Replace if more inliers
        if (tmpN > nInliers)
            M = tmpM;
            inliers = tmpInliers;
            nInliers = tmpN;
        end
    end
    
    inliers = inliers(:, 1:nInliers);
    
    M = M / M(3,3);
%     M = M .* (~(abs(M) < 0.1));
    
%     errors = zeros(1, N);
%     for i=1:N
%         Pi = P(:, i);
%         pi = p(:, i);
%         reproj = M * [Pi; 1]; % Reprojection
%         reproj = reproj * (pi(1) / reproj(1)); % Scale to match point on first element
%         errors(1, i) = norm(pi - reproj(1:2, :)); % Find norm/error in inhomogeneous coordinates
%     end
%     inliers
%     length(inliers)
%     errors'
%     hist(errors);

end