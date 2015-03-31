function [F, inliers] = ransacF(pts1, pts2, normalization_constant)
    % Constants
    nRANSAC = 3000; % Number of RANSAC iterations 15000?
    nPoints = 7; % Number of points for each ransac
    thRANSAC = 0.005; % Threshold of x'T * F * x for a point x to be inlier
    N = size(pts1, 2); % Size of points
    
    %fprintf('Running RANSAC %d times with threshold %f\n', nRANSAC, thRANSAC);
    
    % Variables
    inliers = zeros(1, N); % Store inliers
    nInliers = 0; % Number of inliers
    
    % RANSAC iterations
    for iRANSAC=1:nRANSAC
        % Randomly select 7 points and compute F using 7-pt algorithm
        randind = randperm(N, nPoints);
        tmpF_cell = sevenpoint_norm(pts1(:, randind), pts2(:, randind), normalization_constant);
        
        % Go through each F
        for iF=1:length(tmpF_cell)
            tmpF = tmpF_cell{iF};
            tmpInliers = zeros(1, N);
            tmpN = 0;
    
            for i=1:N
                x  = [pts1(:, i); 1];
                xp = [pts2(:, i); 1];
                error = abs(xp' * tmpF * x / norm(tmpF * x));
                if (error < thRANSAC)
                    tmpN = tmpN + 1;
                    tmpInliers(tmpN) = i;
                end
            end
        
            % Replace if more inliers
            if (tmpN > nInliers)
                F = tmpF;
                inliers = tmpInliers;
                nInliers = tmpN;
            end

        end
    end
    
    F = F / F(end);
    inliers = inliers(:, 1:nInliers);

end