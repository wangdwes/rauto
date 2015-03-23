
function fm = fundamat(locs1, locs2)

  % assume that locs1 and locs2 are un-normalized homogeneous coordinates.
  % note that normalization is performed each time to achieve slightly
  % better performance at the cost of an infinitesimal overhead. 
  [locs1, transmat1] = normalize(locs1);
  [locs2, transmat2] = normalize(locs2);

  coeff = horzcat( ... % assemble the system of linear equations (8-point algorithm)... 
    locs2(:, 1) .* locs1(:, 1), locs2(:, 1) .* locs1(:, 2), locs2(:, 1) .* locs1(:, 3), ...
    locs2(:, 2) .* locs1(:, 1), locs2(:, 2) .* locs1(:, 2), locs2(:, 2) .* locs1(:, 3), ...
    locs2(:, 3) .* locs1(:, 1), locs2(:, 3) .* locs1(:, 2), locs2(:, 3) .* locs1(:, 3)); 

  % use eig instead of svd to obtain the eigenvector corresponding to the minimum eigenvalue. 
  % yes that does save some computational time for an obsessive compulsory disorder patient. 
  [eigenvecs, eigenvals] = eig(coeff' * coeff);
  [~, index] = min(diag(eigenvals)); fmtilde = reshape(eigenvecs(:, index), 3, 3)';

  % then, enforce the internal rank-2 constraint, and un-normalize the coordinates.
  [leftevi, eeva, rightevi] = svd(fmtilde); eeva(end) = 0; 
  [fm] = transmat2' * leftevi * eeva * rightevi' * transmat1;  

end

