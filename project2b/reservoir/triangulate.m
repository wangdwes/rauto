
function locs3d = triangulate(locs1, cameramat1, locs2, cameramat2)

  % compute the coefficients for the four linear equations
  % contributed by a single point correspondence.
  coeff = @(index) {sparse(vertcat( ...
    locs1(index, 2) * cameramat1(3, :) - cameramat1(2, :), ...
    cameramat1(1, :) - locs1(index, 1) * cameramat1(3, :), ...
    locs2(index, 2) * cameramat2(3, :) - cameramat2(2, :), ...
    cameramat2(1, :) - locs2(index, 1) * cameramat2(3, :)))}; 

  % concatencate the coefficients for each point correspondence to
  % form a large, sparse, block diagonal matrix, such that triangulation 
  % for all pairs can be solved with one invocation to some function. 
  coeffcol = arrayfun(coeff, 1: size(locs1, 1)); 
  coeffblk = blkdiag(coeffcol{:}); 
  
  % invoke eigs instead of eig for large and sparse matrix, as recommended.
  [eigenvec, ~] = eigs(coeffblk' * coeffblk, 1, 'lm'); 
  [locs3d] = reshape(eigenvec, 4, []);
  [locs3d] = bsxfun(@rdivide, locs3d(1: 3, :), locs3d(4, :));

end
