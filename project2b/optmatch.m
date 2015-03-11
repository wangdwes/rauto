
function [locs1, locs2, bestfm] = optmatch(f1, vpts1, f2, vpts2)
  
  % find the optimal fundamental matrix that minimizes the sampson distance between
  % two images, assuming that they're connected with pure rotation and translation. 
  [matches] = matchFeatures(f1, f2, 'Method', 'NearestNeighborSymmetric');
  [locs1, locs2] = deal(vpts1(matches(:, 1), :), vpts2(matches(:, 2), :));

  % invoke the ransac routine to find the best model fit, and get rid of the outliers. 
  [bestfm, inliers] = ransacfm(locs1, locs2, 200, 1);
%  [bestfm, inliers] = estimateFundamentalMatrix(locs1(:, 1: 2), locs2(:, 1: 2));
  [locs1, locs2] = deal(locs1(inliers, :), locs2(inliers, :));

end
