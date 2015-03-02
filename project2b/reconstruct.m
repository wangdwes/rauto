
function locs = reconstruct(im1, im2)

  % find the surf featuress. 
  points1 = detectSURFFeatures(im1);
  points2 = detectSURFFeatures(im2);
  % extract the features.
  [f1, vpts1] = extractFeatures(im1, points1);
  [f2, vpts2] = extractFeatures(im2, points2);
  % retrieve the locations of matched points.
  matches = matchFeatures(f1, f2);
  locs1 = double(vpts1.Location); 
  locs2 = double(vpts2.Location);

  % run the ransac routine to get the best fundamental matrix, 
  % and choose camera matrices according to slide 6, 4.0 reconstruction. 
  [bestF, ~, inliers] = ransacF(matches, locs1, locs2, 200, 1);
  [leftevec, ~, rightevec] = svd(bestF); ilmatches = matches(inliers, :);
  [cameramat1] = horzcat(eye(3), ones(3, 1)); 
  [cameramat2] = horzcat(hat(rightevec(:, end)) * bestF, leftevec(:, end)); 

  % invoke the triangulation routine to figure out the location of 3-space points.  
  locs = triangulate(locs1(ilmatches(:, 1)), cameramat1, locs2(ilmatches(:, 2)), cameramat2); 

end
  

