
% normalize two/three-dimensional homogeneous coordinates. 
% they're returned along with the homogeneous transformation matrix.  
function [normcoords, transmat] = normalize(coords)

  % compute the necessary translation and scaling,
  % and formulate the homogeneous transformation matrix.
  centers = mean(coords, 1); ctrcoords = bsxfun(@minus, coords, centers);
  scale = sqrt(size(coords, 2) - 1) / mean(sqrt(sum(ctrcoords(:, 1: end - 1) .^ 2, 2))); 

  % formulate the homogeneous transformation matrix and normalize the coordinates. 
  transmat = diag([scale * ones(1, size(coords, 2) - 1), 1]);
  transmat(1: end - 1, end) = -scale * centers(1: end - 1); 
  normcoords = coords * transmat'; 

end

