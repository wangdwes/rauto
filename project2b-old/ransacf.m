
function [bestF, inliers] = ransacf(matches, locs1, locs2, nIter, tol)

  % this must be initialized for obvious reason. 
  % by the way, 200 and 1 are reasonable values for nIter and tol. 
  inliers = zeros(size(matches, 1), 1); bestF = eye(3);

  for iter = 1: nIter

    % randomly sample eight point pairs (correspondences) each time,
    % and compute the corresponding fundamental matrix.  
    matchsel = matches(randperm(size(matches, 1), 8), :);
    f = fundamat(locs1(matchsel(:, 1), :), locs2(matchsel(:, 2), :)); 

    % compute the distance and see if we have come cross a better solution.
    % note that this statement can be re-written to be slightly more efficient.
    locs1match = [locs1(matches(:, 1), :), ones(size(matches, 1), 1)]; 
    locs2match = [locs2(matches(:, 2), :), ones(size(matches, 1), 1)]; 
    normfac1 = locs1match * f'; normfac2 = locs2match * f;  
    dists = abs(sum(locs2match * f .* locs1match, 2) .^ 2 ./ ...
      (normfac1(:, 1) .^ 2 + normfac1(:, 2) .^ 2 + ...
       normfac2(:, 1) .^ 2 + normfac2(:, 2) .^ 2)); 

    % if there are more inliers, we'll take it.  
    if (sum(inliers) < sum(dists < tol)), inliers = dists < tol; end

  end 

  % c.f.: http://en.wikipedia.org/wiki/RANSAC 
  % the model may be improved by reestimating it using all members of the consensus set.
  ilmatches = matches(inliers, :); 
  bestF = fundamat(locs1(ilmatches(:, 1), :), locs2(ilmatches(:, 2), :));

end
