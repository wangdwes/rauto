
function [r, t] = camtrans(locs1, locs2, f, intriparams)

  w = [0, -1, 0; 1, 0, 0; 0, 0, 1]; 
  % obtain the essential matrix and its decomposition, 
  % and assemble all four possible solution for later assessment.  
  essmat = intriparams' * f *  intriparams; [u, s, v] = svd(essmat);
  r1 = u * w * v'; r2 = u * w' * v'; t1 = u(:, end); t2 = -u(:, end);

  % note that f is determined only up a scale, negative or positive. 
  % there is nothing much we can do on that part, but the rotation matrix
  % is actually characterized as orthogonal with determinant 1.  
  r1 = r1 * sign(det(r1)); r2 = r2 * sign(det(r2)); 

  % so there are four possible solutions (candidates) that are compatible with f. 
  % but there is only one where the points reside in front of both cameras. 
  % c.f. http://en.wikipedia.org/wiki/Essential_matrix for a method that determines which. 
  rcandi = cat(3, r1, r1, r2, r2); tcandi = cat(3, t1, t2, t1, t2); loc1h = [locs1(3, :), 1];
  tnom = rcandi(1, :, :) - locs2(3, 1) * rcandi(3, :, :);

  % here we compute the z's of the 3d point with respective to the frames of each camera.
  % should they both be positive, we know that the correponding pair constitutes a solution.  
  z = sum(tnom .* permute(tcandi, [2 1 3]), 2) ./ sum(bsxfun(@times, tnom, loc1h), 2); 
  zprime = sum(permute(bsxfun(@times, z, loc1h') - tcandi, [2 1 3]) .* rcandi(3, :, :), 2); 
  selected = squeeze((z > 0) & (zprime > 0)); r = rcandi(:, :, selected); t = tcandi(:, :, selected);

end



