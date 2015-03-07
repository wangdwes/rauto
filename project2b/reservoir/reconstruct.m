
function locs3d = reconstruct(locs1, locs2, f) 

  % decompose the fundamental matrix to get the epipoles,   
  % which help us construct the camera matrices.
  [leftevi, ~, rightevi] = svd(f);
  [cameramat1] = horzcat(eye(3), ones(3, 1));
  [cameramat2] = horzcat(hat(rightevi(:, end)) * f, leftevi(:, end));

  % invoke the triangulation routine to find out the three-dimensional points.
  locs3d = triangulate(locs1, cameramat1, locs2, cameramat2);

end
