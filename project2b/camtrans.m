
function [rot, tr] = camtrf(locs1, locs2, fm, intriparams)
  
  w = [0, -1, 0; 1, 0, 0; 0, 0, 1];
  z = [0, 1, 0; -1, 0, 0; 0, 0, 0];
  % obtain the essential matrix from the fundamental matrix. 
  % note that this may not comply with the internal constraint. 

  % obtain the essential matrix from the fundamental matrix. note that this
  % may not comply with its internal constraint and we may have trouble in a later part.  
  em = intriparams' * fm * intriparams;

  % apply singular value decomposition to obtain the candidates for rotation matrices
  % translations. this routine actually assumes that the internal constraint is observed.  
  [u, s, v] = svd(em); if det(u * w' * v') < 0, [u, s, v] = svd(-em); end;
  [rot1, rot2, tr1] = deal(u * w' * v', u * w * v', vee(u * w * s * u')'); tr2 = -tr1;

  % at this point we have four combinations of rotation matrices and translation vectors. 
  % we would have to triangulate one of the point correspondence to figure out which 
  % pair leaves the 3d-point in front of the camera. 
  rotcandi = cat(3, rot1, rot1, rot2, rot2);
  trcandi = cat(3, tr1, tr2, tr1, tr2);

  % here we're computing the z coordinates of some three-dimensional points with respect
  % to the frames of each camera. should they both be positive, the corresponding pair is correct. 
  tnom = rotcandi(1, :, :) - locs2(1, 1) * rotcandi(3, :, :);
  z = sum(tnom .* trcandi, 2) ./ sum(bsxfun(@times, tnom, locs1(1, :)), 2);
  zprime = sum((bsxfun(@times, z, locs1(1, :)) - trcandi) .* rotcandi(3, :, :), 2);

  % obtain the correct combination of translation and rotation.  
  selected = squeeze(z > 0 & zprime > 0); 
  rot = rotcandi(:, :, selected);
  tr = squeeze(trcandi(:, :, selected))';

  % if this happens we're doomed.
  assert(sum(selected) == 1, 'There are %d valid camera transformations.', sum(selected));

end

