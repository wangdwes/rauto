
function fm = fundamat(locs1, locs2) 
 
  % we're assuming that locs1 and locs2 are normalized homogeneous coordinates here. 
  % this may not achieve an opitmal result but it avoids repetitive computation 
  % of similar data points in the ransac routine. 
  coeff = horzcat( ... 
    locs2(:, 1) .* locs1(:, 1), locs2(:, 1) .* locs1(:, 2), locs2(:, 1) .* locs1(:, 3), ...
    locs2(:, 2) .* locs1(:, 1), locs2(:, 2) .* locs1(:, 2), locs2(:, 2) .* locs1(:, 3), ...
    locs2(:, 3) .* locs1(:, 1), locs2(:, 3) .* locs1(:, 2), locs2(:, 3) .* locs1(:, 3)); 

  % use svd to find the eigenvector corresponding to the minimum eigenvalue.
  % then enforce rank-2 constraint by applying another singular value decomposition.  
  [~, ~, rightevi] = svd(coeff, 0);
  [fmtilde] = reshape(rightevi(:, end), 3, 3)';

  [leftevi, eeva, rightevi] = svd(fmtilde); eeva(end) = 0;  
  [fm] = leftevi * eeva * rightevi';

end
  
