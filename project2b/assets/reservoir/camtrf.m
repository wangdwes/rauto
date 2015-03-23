
function [rot, tr] = camtrf(locs1, locs2, fm, intriparams)

  w = [0, -1, 0; 1, 0, 0; 0, 0, 1];
  % the literature proposes a sr decomposition for an essential matrix with two identical
  % and one zero eigenvalues, but our matrix computed from the fundamental matrix doens't
  % necessarily observe this constraint. therefore, we try to enforce it before proceeding.
  [u, s, v] = svd(intriparams' * fm * intriparams); em = u * diag([1 1 0]) * v';
  [u, s, v] = svd(em); if det(u*w'*v') < 0, [u, s, v] = svd(-em); end; 
  [rot1, rot2, tr1] = deal(u*w*v', u*w'*v', u(:, end)'); tr2 = -tr1;

  % here we have four possible combinations of rotation matrix and translation vector.
  % instead of invoking the expensive triangulation routine, we're employing the 
  % method by h.c. louguet-higgins, a computer algorithm for reconstructing a scene... however,
  % note that different transformation convention, and the 'calibrated camera' requirement.  
  rotcandi = cat(3, rot1, rot1, rot2, rot2);
  trcandi = -cat(3, tr1*rot1, tr2*rot1, tr1*rot2, tr2*rot2);  
  
  % account for the 'calibrated camera' requirement by converting the coordinates.  
  locs2k = locs2(1, :) * inv(intriparams)';
  locs1k = locs1(1, :) * inv(intriparams)';

  % then we'll be able to follow the proposed routine to compute the z coordinate values 
  % of one three-dimensional point with respect to the frames of each camera. should they
  % both be positive, the corresponding pair is our solution.  
  tnom = rotcandi(1, :, :) - locs2k(1, 1) * rotcandi(3, :, :);
  z = sum(tnom .* trcandi, 2) ./ sum(bsxfun(@times, tnom, locs1k(1, :)), 2);
  zprime = sum((bsxfun(@times, z, locs1k(1, :)) - trcandi) .* rotcandi(3, :, :), 2);

  % obtain the correct combination of translation and rotation.  
  selected = squeeze(z > 0 & zprime > 0); 
  rot = rotcandi(:, :, selected);
  tr = squeeze(trcandi(:, :, selected))';

  % if this happens we're doomed.
  assert(sum(selected) == 1, 'There are %d valid camera transformations.', sum(selected));

end

