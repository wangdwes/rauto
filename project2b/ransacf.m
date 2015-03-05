
function [bestf, inliers] = ransacf(matches, locs1, locs2, maxiter, tol)

  % note that bestf cannot be initialized to eye. 
  matchcount = size(matches, 1); inliers = zeros(matchcount, 1); bestf = diag([1, 1, 0]);
  locs1match = locs1(matches(:, 1), :); locs2match = locs2(matches(:, 2), :); 

  for index = 1: maxiter,

    % randomly sample eight point pairs (correspondences) each time,
    % and compute the corresponding fundamential matrix. 
    matchsel = matches(randperm(matchcount, 8), :);
    f = fundamat(locs1(matchsel(:, 1), :), locs2(matchsel(:, 2), :));

    % compute the sampson distances as our error metrics. 
    normfac1 = locs1match * f'; normfac2 = locs2match * f;
    normfac1 = [normfac1(:, 1) ./ normfac1(:, 3), normfac1(:, 2) ./ normfac1(:, 3)]; 
    normfac2 = [normfac2(:, 1) ./ normfac2(:, 3), normfac2(:, 2) ./ normfac2(:, 3)]; 
    dists = abs(sum(locs2match * f .* locs1match, 2) .^ 2 ./ ...
      (normfac1(:, 1) .^ 2 + normfac1(:, 2) .^ 2 + ...
       normfac2(:, 1) .^ 2 + normfac2(:, 2) .^ 2)); 

    % if there're more inliers and previously stored, take it. 
    if (sum(inliers) < sum(dists < tol)), inliers = dists < tol; end

  end 
  
  % the model may be improved by re-restimating it
  % using all members of the consensus set.
  ilmatches = matches(inliers, :);
  bestf = fundamat(locs1(ilmatches(:, 1), :), locs2(ilmatches(:, 2), :)); 

end
