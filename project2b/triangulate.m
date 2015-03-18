
function locs3d = triangulate(locs1, locs2, cameramat1, cameramat2, method)

  % adopted from fundfromcameras - fundamental matrix from camera matrices. 
  % http://www.csse.uwa.edu.au/~pk/research/matlabfns/Projective/fundfromcameras.m
  fm = hat(cameramat2 * null(cameramat1)) * cameramat2 * pinv(cameramat1);
 
  % implemented here is the 'optimal triangulation method', introduced as algorithm 12.1 in the literature.
  % note that instead of producing the actual three-dimensional points, this step only optimizes the
  % two-dimensional points such that they better meet the epipolar line constraint.  
  if nargin == 5 & strcmp(method, 'optimal')  

    % iterating througha all points is definitely the fastest method known to me.  
    [transmat1, transmat2] = deal(eye(3));
    for index = 1: size(locs1, 1)

      % defining transformation matrices that would bring the points to the origin.  
      % replace the old fundamental matrix with one corresponding to translated coordinates.  
      transmat1(1: 2, end) = -locs1(index, 1: 2)';
      transmat2(1: 2, end) = -locs2(index, 1: 2)';
      fmt = inv(transmat2)' * fm * inv(transmat1); 

      % compute the right and left epipoles by applying svd on the new fundamental matrix,
      % and normalize them such that the sum of the squares of the first two elements is one.  
      [leftevi, ~, rightevi] = svd(fmt); 
      [epipole1] = leftevi(:, end) / sqrt(sumsqr(leftevi(1: 2, end))); 
      [epipole2] = rightevi(:, end) / sqrt(sumsqr(rightevi(1: 2, end)));

      % form two rotation matrices and replace the fundamental matrix again.  
      rotmat1 = [epipole1(1), epipole1(2), 0; -epipole1(2), epipole1(1), 0; 0, 0, 1];
      rotmat2 = [epipole2(1), epipole2(2), 0; -epipole2(2), epipole2(1), 0; 0, 0, 1];
      fmt = rotmat2 * fmt * rotmat1';

      % form the 6-order polynomial given by equation 12.7 and find its six roots.
      % again allow me to apologize to have included such a gigantic equation in here.  
      [f, fp, a, b, c, d] = deal(epipole1(3), epipole2(3), fmt(2, 2), fmt(2, 3), fmt(3, 2), fmt(3, 3));
      [ts] = [real(roots(vertcat( ... 
        -(a*d-b*c)*f^4*a*c, ...
        -(a*d-b*c)*f^4*(b*c+a*d)+(a^2+fp^2*c^2)^2, ...
        -(a*d-b*c)*(f^4*b*d+2*a*c*f^2)+2*(a^2+fp^2*c^2)*(2*a*b+2*c*d*fp^2), ...
        -(a*d-b*c)*(2*f^2*(b*c+a*d))+2*(a^2+fp^2*c^2)*(b^2+fp^2*d^2)+(2*a*b+2*c*d*fp^2)^2, ...
        -(a*d-b*c)*(a*c+2*b*d*f^2)+2*(2*a*b+2*c*d*fp^2)*(b^2+fp^2*d^2), ...
        -(a*d-b*c)*(b*c+a*d)+(b^2+fp^2*d^2)^2, ...
        -(a*d-b*c)*b*d))); Inf]; % cost function to be evaluated at infinity as well. 

      % evaluate the cost function at the real parts of the roots and select the smallest. 
      % the closest points on the epipolar lines to the origin are evaluated. 
      tmin = min(ts.^2./(1+f^2*ts.^2)+(c*ts+d).^2./((a*ts+b).^2+fp^2*(c*ts+d).^2));  
      xhat1 = vertcat(tmin^2*f, tmin, tmin^2*f^2+1);
      xhat2 = vertcat(fp*(c*tmin+d)^2, -(a*tmin+b)*(c*tmin+d), fp^2*(c*tmin+d)^2+(a*tmin+b)^2);

      % transfer back to the original coordinates.  
      locs1(index, :) = (inv(transmat1) * rotmat1' * xhat1)';
      locs2(index, :) = (inv(transmat2) * rotmat2' * xhat2)';

    end
  end

  % alternatively, if the user requests some level of approximation but wishes to avoid 
  % too much computational cost, the sampson approximation (first-order geometric correction) 
  % may be used. see section 12.4 in the literature for more details.  
  if nargin == 5 & strcmp(method, 'sampson')

    % compute the coefficients. see page 315 for the mathematical derivation. 
    leftmulti = locs1 * fm'; rightmulti = locs2 * fm; 
    coeff = sum(rightmulti .* locs1, 2) ./ ...
      (leftmulti(:, 1) .^ 2 + rightmulti(:, 1) .^ 2 + ...
       leftmulti(:, 2) .^ 2 + rightmulti(:, 2) .^ 2); 
  
    % apply the first-order correction to the original coordinates.
    % note that this only works well when the correction is expected to be small.
    locs1(:, 1) = locs1(:, 1) - rightmulti(:, 1) .* coeff; 
    locs1(:, 2) = locs1(:, 2) - rightmulti(:, 2) .* coeff; % alternatively, use bsxfun.
    locs2(:, 1) = locs2(:, 1) - leftmulti(:, 1) .* coeff;
    locs2(:, 2) = locs2(:, 2) - leftmulti(:, 2) .* coeff; 
 
  end

  locs3d = zeros(size(locs1, 1), 4); 
  % iterate through all point correspondences, which is the fastest knowm method, 
  % as opposed to concatencate a block diagonal matrix and svd once.
  for index = 1: size(locs1, 1) 

    % form the system of linear equations.
    coeff = vertcat( ...
      locs1(index, 1) * cameramat1(3, :) - cameramat1(1, :), ... 
      locs1(index, 2) * cameramat1(3, :) - cameramat1(2, :), ... 
      locs2(index, 1) * cameramat2(3, :) - cameramat2(1, :), ... 
      locs2(index, 2) * cameramat2(3, :) - cameramat2(2, :)); 

    % solve for the location of the three-dimensional point.  
    [~, ~, eigenvecs] = svd(coeff);
    [locs3d(index, :)] = eigenvecs(:, end)' / eigenvecs(4, end); 

  end
end

function mat = hat(vec)
  % convert a vector to its corresponding skew-symmetric matrix.
  mat = [0, -vec(3), vec(2); vec(3), 0, -vec(1); -vec(2), vec(1), 0];
end
