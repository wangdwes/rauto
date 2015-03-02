
function mat = fundamat(locs1, locs2) 

% normalize the coordinates to eliminate dc bias
% and guarantee an average distance to the centroid of sqrt2. 
[locs1, transmat1] = normalize(locs1);
[locs2, transmat2] = normalize(locs2);

% each point contributes one linear equation to this system. 
% note that this seems to be slightly faster than the original kronecker product.  
coeff = horzcat(bsxfun(@times, locs1(:, 1), locs2), bsxfun(@times, locs1(:, 2), locs2), locs2);

% utilize svd to find the elements of the fundamental matrix, 
% then use it again to enforce the rank-2 constraint.  
[~, ~, v] = svd(coeff, 0); 
[u, s, v] = svd(reshape(v(:, end), 3, 3)', 0);
[mat] = transmat2' * (u * diag([s(1, 1), s(2, 2), 0]) * v') * transmat1; 
