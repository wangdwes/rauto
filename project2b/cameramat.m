
function cm = cameramat(locs3d, locs2d, method) 

  % write something random to make this look well documented.  
  count = size(locs3d, 1); inliers = false(count, 1);
  cm = eye(3, 4); niter = 0; maxiter = 200; tol = 1;   

  % iterate until the maximum number of iterations is hit. 
  while niter < maxiter, 

    % randomly select six point pairs (correspondences) each time, 
    % and compute the corresponding camera matrix. 
    selected = randperm(count, 6); 
    cm = dlt6p(locs3d(selected, :), locs2d(selected, :));

    % compute the reprojection error. 
    rplocs = locs3d * cm'; rplocs = bsxfun(@rdivide, rplocs, rplocs(:, 3));
    dists = sum((rplocs(:, 1: 2) - locs2d(:, 1: 2)) .^ 2, 2); 

    niter = niter + 1;
    % if there're more inliers than previously stored, take it.
    if sum(inliers) < sum(dists < tol ^ 2),
      inliers = dists < tol;
      % implemented here is a sub-routine that updates the number of iterations
      % based on our desired confidence level and ratio of inliers.
      ratio = sum(inliers) / count;
      if ratio > 1 - eps, maxiter = 1;
      else ratiop6 = ratio ^ 6; 
        maxiter = min(maxiter, floor(2 * log(10) / ratiop6));
      end
    end
  end

  cm = dlt6p(locs3d(inliers, :), locs2d(inliers, :)); 
  % the model may be improved by re-estimating it
  % using all memeber of the consensus set. 

  function cm = dlt6p(locs3d, locs2d)
    % each 3d-2d point correspondence contributes two independent equations. 
    coeff = [locs3d, zeros(size(locs3d)), -bsxfun(@times, locs2d(:, 1), locs3d); ...
             zeros(size(locs3d)), locs3d, -bsxfun(@times, locs2d(:, 2), locs3d)]; 
    % obtain the eigenvector corresponding to the minimum eigenvalue. 
    [~, ~, eigenvecs] = svd(coeff); cm = reshape(eigenvecs(:, end), 4, 3)'; end

end
