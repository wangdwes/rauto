% Computes the transformation between 3D points using RANSAC.
%   X - 3xN 3D points (N >= 3)
%   Y - 3xN 3D points (N >= 3)
%   T - Homogeneous transformation s.t. Xh = T*Yh (h = homogeneous coordinates)
%   Inliers - Inlier point indices within a set threshold

function [T, inliers] = ransac3D(X, Y)
    % Constants
    nRANSAC = 3000; % Number of RANSAC iterations 15000?
    nPoints = 3; % Number of points for each ransac
    thres = 0.2;%0.4;
    N = size(X, 2); % Size of points
    
    %fprintf('Running RANSAC %d times with threshold %f\n', nRANSAC, thres);
    
    % Variables
    inliers = zeros(1, N); % Store inliers
    nInliers = 0; % Number of inliers
    
    % RANSAC iterations
    for iRANSAC=1:nRANSAC
        % Randomly select 3 points and compute T
        randind = randperm(N, nPoints);
        tmpT = estimateRigidTransform(X(:, randind), Y(:, randind));
        
        % Record results
        tmpInliers = zeros(1, N);
        tmpN = 0;
        
        for i=1:N
            Xi = X(:, i);
            Yi = Y(:, i);
            reproj = tmpT * [Yi; 1]; % Reprojection
            repError = norm(Xi - reproj(1:3, :)); % Find norm/error in inhomogeneous coordinates
            if (repError < thres)
                tmpN = tmpN + 1;
                tmpInliers(tmpN) = i;
            end
        end

        % Replace if more inliers
        if (tmpN > nInliers)
            T = tmpT;
            inliers = tmpInliers;
            nInliers = tmpN;
        end
    end
    
    inliers = inliers(:, 1:nInliers);
    
%     errors = zeros(1, N);
%     for i=1:N
%         Xi = X(:, i);
%         Yi = Y(:, i);
%         reproj = T * [Yi; 1]; % Reprojection
%         errors(1, i) = norm(Xi - reproj(1:3, :)); % Find norm/error in inhomogeneous coordinates
%     end
%     inliers
%     length(inliers)
%     errors'
%     hist(errors)
    
    %T = estimateRigidTransform(X(:, inliers), Y(:, inliers));

end