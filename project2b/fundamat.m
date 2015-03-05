
function fm = fundamat(locs1, locs2) 
 
  % we're assuming that locs1 and locs2 are normalized homogeneous coordinates here. 
  % this may not achieve an opitmal result but it avoids repetitive computation 
  % of similar data points in the ransac routine. 
  coeff = horzcat( ... 
    locs2(:, 1) .* locs1(:, 1), locs2(:, 1) .* locs1(:, 2), locs2(:, 1) .* locs1(:, 3), ...
    locs2(:, 2) .* locs1(:, 1), locs2(:, 2) .* locs1(:, 2), locs2(:, 2) .* locs1(:, 3), ...
    locs2(:, 3) .* locs1(:, 1), locs2(:, 3) .* locs1(:, 2), locs2(:, 3) .* locs1(:, 3)); 

  % use eig whenever possible to save computational time.  
  [eigenvecs, eigenvals] = eig(coeff' * coeff); 
  [~, index] = min(diag(eigenvals));
  [fmtilde] = reshape(eigenvecs(:, index), 3, 3)'; 

  % enforce rank-2 constraint. here we have no option but utilizing svd.  
  [leftevi, eeva, rightevi] = svd(fmtilde); eeva(3, 3) = 0;
  [fm] = leftevi * eeva * rightevi'; 

end
  
