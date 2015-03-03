
function locs = reconstruct(locs1, locs2, f)

  % decompose the fundamental matrix to get the epipoles, 
  % which helps us construct the camera matrices. 
  [leftevec, ~, rightevec] = svd(f);
  [cameramat1] = horzcat(eye(3), ones(3, 1)); 
  [cameramat2] = horzcat(hat(rightevec(:, end)) * f, leftevec(:, end)); 

  % invoke the triangulation routine to figure out the location of 3-space points.  
  locs = triangulate(locs1, cameramat1, locs2, cameramat2); 

end
  

