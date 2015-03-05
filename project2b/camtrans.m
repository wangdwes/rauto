
function [rot, tr] = camtrans(locs1, locs2, fm, intriparams)

  w = [0, -1, 0; 1, 0, 0; 0, 0, 1];
  % obtain the essential matrix and its decomposition. note that if the determinant 
  % of its resulting rotation matrix is negative, use its inverse for svd.
  essmat = intriparams' * fm * intriparams; [leftevi, eeva, rightevi] = svd(essmat); 
  if det(leftevi * w' * rightevi') < 0,  [leftevi, eeva, rightevi] = svd(-essmat); end; 

  % assemble all four possible solutions for later assessment. 
  % note that taking the third column of v produces a different result, but still correct.
  rot1 = leftevi * w * rightevi'; rot2 = leftevi * w' * rightevi'; 
  tr1 = vee(rightevi * w * eeva * rightevi')'; tr2 = -tr1; 

  % i apologize for producing bad-looking code... but when performance is
  % the critical factor, we have to make some sacrifice here. 
  rotcandi = cat(3, rot1, rot1, rot2, rot2); trcandi = cat(3, tr1, tr2, tr1, tr2);
  tnom = rotcandi(1, :, :) - locs2(1, 1) * rotcandi(3, :, :);

  % here we're computing the z coordinates of some three-dimensional points with respect
  % to the frames of each camera. should they both be positive, the corresponding pair is correct. 
  z = sum(tnom .* trcandi, 2) ./ sum(bsxfun(@times, tnom, locs1(1, :)), 2);
  zprime = sum((bsxfun(@times, z, locs1(1, :)) - trcandi) .* rotcandi(3, :, :), 2);
  selected = squeeze(z > 0 & zprime > 0); 

  % obtain the correct combination of translation and rotation.  
  rot = rotcandi(:, :, selected);
  tr = squeeze(trcandi(:, :, selected))';

  % if this happens we're doomed.
  assert(sum(selected) == 1, 'There are %d valid camera transformations', sum(selected));

end









