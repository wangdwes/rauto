
function [r, t] = camtrans(locs1, locs2, f, intriparams)

  w = [0, -1, 0; 1, 0, 0; 0, 0, 1];
  z = [0, 1, 0; -1, 0, 0; 0, 0, 0];
  % obtain the essential matrix and its decomposition.
  % if the determinant or r is not positive one, take the inverse of the essential matrix.
  % c.f. http://en.wikipedia.org/wiki/Essential_matrix 
  essmat = intriparams' * f * intriparams; [u, s, v] = svd(essmat);
  if det(u * w' * v') < 0, [u, s, v] = svd(-essmat); end

  % assemble all four possible solution for later assessment.
  % note that taking the third column of v seemingly leads to incorrect results. reason unknown.  
  r1 = u * w * v'; r2 = u * w' * v';
  t1 = vee(u * w * s * u'); t2 = vee(u * w' * s * u');

  % so there are four possible solutions (candidates) that are compatible with f. 
  % but there is only one where the points reside in front of both cameras. 
  % c.f. http://en.wikipedia.org/wiki/Essential_matrix for a method that determines which. 
  rcandi = cat(3, r1, r1, r2, r2); tcandi = cat(3, t1, t2, t1, t2); loc1h = [locs1(1, :), 1];
  tnom = rcandi(1, :, :) - locs2(1, 1) * rcandi(3, :, :);

  % here we compute the z's of the 3d point with respective to the frames of each camera.
  % should they both be positive, we know that the correponding pair constitutes a solution.  
  z = sum(tnom .* permute(tcandi, [2 1 3]), 2) ./ sum(bsxfun(@times, tnom, loc1h), 2); 
  zprime = sum(permute(bsxfun(@times, z, loc1h') - tcandi, [2 1 3]) .* rcandi(3, :, :), 2); 
  selected = ((z > 0) & (zprime > 0)); r = rcandi(:, :, selected); t = tcandi(:, :, selected);
  
  % if this happens we're doomed. 
  assert(sum(selected) == 1, 'There does not exist an unique camera transformation.'); 

end

