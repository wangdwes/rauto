
function [cm, inliers] = cameramat(locs3d, locs2d, method) 

  % normalize the points as suggested in hartley, zisserman.  
  [locs3d, transmat3d] = normalize(locs3d);
  [locs2d, transmat2d] = normalize(locs2d); 

  % write something random to make this look well documented.  
  count = size(locs3d, 1); inliers = false(count, 1);
  cm = eye(3, 4); niter = 0; maxiter = 200; tol = 1e-4;

  % iterate until the maximum number of iterations is hit. 
  while niter < maxiter, 

    % randomly select six point pairs (correspondences) each time, 
    % and compute the corresponding camera matrix. 
    selected = randperm(count, 6); 
    cm = dlt6p(locs3d(selected, :), locs2d(selected, :));

    dists = sum(reprojerr(locs3d, locs2d, cm) .^ 2, 2); niter = niter + 1;
    % if there're more inliers than previously stored, take it.
    if sum(inliers) < sum(dists < tol),
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

  locs3d = locs3d(inliers, :); locs2d = locs2d(inliers, :); cm = dlt6p(locs3d, locs2d); 
  % i do apologize for having to write the same thing twice.
  % here we're invoking the levenberg-marquardt routine to optimize the solution.  
  options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'Display', 'none'); 
  cm = lsqnonlin(@(vec) reprojerr(locs3d, locs2d, reshape(vec, 4, 3)'), reshape(cm', [], 1), [], [], options); 
  cm = inv(transmat2d) * reshape(cm, 4, 3)' * transmat3d; 

  function cm = dlt6p(locs3d, locs2d)
    % each 3d-2d point correspondence contributes two independent equations. 
    coeff = [locs3d, zeros(size(locs3d)), -bsxfun(@times, locs2d(:, 1), locs3d); ...
             zeros(size(locs3d)), locs3d, -bsxfun(@times, locs2d(:, 2), locs3d)]; 
    % obtain the eigenvector corresponding to the minimum eigenvalue. 
    % important: normalize the matrix or our dists computation will be meaningless.  
    [~, ~, eigenvecs] = svd(coeff); cm = reshape(eigenvecs(:, end), 4, 3)'; end

  function err = reprojerr(locs3d, locs2d, cm)
    % computing the reprojection error 
    rplocs = locs3d * cm'; rplocs = bsxfun(@rdivide, rplocs, rplocs(:, 3)); 
    err = rplocs(:, 1: 2) - locs2d(:, 1: 2); end

end
