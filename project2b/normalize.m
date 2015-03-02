
% convert two-dimensional coordinates to normalized three-dimensional coordinates.
% they're returned along with the homogeneous transformation matrix.  
function [normcoords, transmat] = normalize(coords) 

  % compute the necessary translation and scaling, 
  % and formulate the homogeneous transformation matrix.  
  centers = mean(coords, 1); ctrcoords = bsxfun(@minus, coords, centers);
  scale = sqrt(2) / mean(sqrt(ctrcoords(:, 1) .^ 2 + ctrcoords(:, 2) .^ 2));
  transmat = [scale, 0, -scale * centers(1); 0, scale, -scale * centers(2); 0, 0, 1];

  % homogenize the coordinates and apply the transformation, 
  normcoords = [coords, ones(size(coords, 2), 1)] * transmat';

end
