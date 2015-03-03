
function locs = triangulate(locs1, cameramat1, locs2, cameramat2)

  % compute the coefficients for the four linear equations contributed by 
  % a single point correspondence.
  coeff = @(index) {vertcat( ...
    locs1(index, 2) * cameramat1(3, :) - cameramat1(2, :), ...
    cameramat1(1, :) - locs1(index, 1) * cameramat1(3, :), ...
    locs2(index, 2) * cameramat2(3, :) - cameramat2(2, :), ...
    cameramat2(1, :) - locs2(index, 1) * cameramat2(3, :))}; 
    
  % concatencate the coefficients for each point correspondence to form a block diagonal matrix
  % such that triangulation for all points can be solved with one svd.
  coeffcol = arrayfun(coeff, 1: size(locs1, 1));
  coeffblk = blkdiag(coeffcol{:}); 
  
  % apply singular value decompisition and take the eigenvector that corresponds 
  % to the minimum eigenvalue, then reshape and restore it to regular coorindates.  
  [~, ~, eigenvec] = svd(coeffblk);
  [locs] = reshape(eigenvec(:, end), 4, []);
  [locs] = bsxfun(@rdivide, locs(1: 3, :), locs(4, :)); 

end
  

