
function [locs1, locs2, bestF] = findmatch(im1, im2)

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
  % then find out the correspondences that produce this matrix.  
  [bestF, inliers] = ransacf(matches, locs1, locs2, 300, 1e-3);
  ilmatches = matches(inliers, :);
  locs1 = locs1(ilmatches(:, 1), :);
  locs2 = locs2(ilmatches(:, 2), :);

end
  

