
function [bestfm, inliers] = ransacfm(locs1, locs2, maxiter, tol)

  % in the literature, the gold standard method, aka the maximum likelihood
  % estimate method is most recommended along parameterization and sampson distance. 
  % but in our use case, the simplest sampson distance metric should suffice. 
  matchcount = size(locs1, 1); niter = 0;
  inliers = false(matchcount, 1); bestfm = diag([1 1 0]);

  while niter < maxiter, % maxiter can be altered inside the while-loop.  
    % randomly select eight point correspondences each time, and compute
    % the fundamental matrix with the normalized 8-point algorithm.
    selected = randperm(matchcount, 8);
    fm = fundamat(locs1(selected, :), locs2(selected, :)); 

    [locs3d, cameramat1, cameramat2] = triangulate(locs1, locs2, fm);

    err1 = locs3d * cameramat1'; err1 = bsxfun(@rdivide, err1, err1(:, 3));
    err1 = err1 - locs1; err1 = err1(:, 1) .^ 2 + err1(:, 2) .^ 2; 

    err2 = locs3d * cameramat2'; err2 = bsxfun(@rdivide, err2, err2(:, 3)); 
    err2 = err2 - locs2; err2 = err2(:, 1) .^ 2 + err2(:, 2) .^ 2; 

    dists = err1 + err2; 

    % compute the sampson distances (first-order geometric error) as our error metrics.
%    leftmulti = locs1 * fm'; rightmulti = locs2 * fm;
%    dists = sum(rightmulti .* locs1, 2) .^ 2 ./ ( ...
%      leftmulti(:, 1) .^ 2 + rightmulti(:, 1) .^ 2 + ...
%      leftmulti(:, 2) .^ 2 + rightmulti(:, 2) .^ 2);

    niter = niter + 1;
    % if there're more inliers than previously stored, take it. 
    if sum(inliers) < sum(dists < tol ^ 2),
      inliers = dists < tol;
      % a routine that adaptively changes the maximum number of iterations
      % based on our desired confidence level and ratio of inliers.
      ratio = sum(inliers) / matchcount;
      if ratio > 1 - eps, maxiter = 1; % no outliers? we'll stop here. 
      else maxiter = min(maxiter, ceil(log(0.01) / (-ratio ^ 8))); end;
    end
  end

  % the model may be improved by re-estimating it
  % using all members of the consensus set, i quote.  
  assert(sum(inliers) >= 8, 'The number of inliers is less than eight.'); 
  bestfm = fundamat(locs1(inliers, :), locs2(inliers, :)); 

end

 
