
function [bestF, bestError, inliers] = ransacF(matches, locs1, locs2, nIter, tol)

  % this must be initialized for obvious reason. 
  % by the way, 200 and 1 are reasonable values for nIter and tol. 
  inliers = zeros(size(matches, 1), 1); bestF = eye(3); bestError = 0;

  for iter = 1: nIter

    % randomly sample eight point pairs (correspondences) each time,
    % and compute the corresponding fundamental matrix.  
    matchsel = matches(randperm(size(matches, 1), 8), :);
    f = fundamat(locs1(matchsel(:, 1), :)', locs2(matchsel(:, 2), :)'); 

    % compute the distance and see if we have come cross a better solution.
    % note that this statement can be re-written to be slightly more efficient.
    dists = diag(locs2(matches(:, 2), :)' * f * locs1(matches(:, 1), :));
    if (sum(inliers) < sum(dists < tol))
      bestF = H2to1; bestError = sum(dists); inliers = dists < tol; end 

  end 

end
