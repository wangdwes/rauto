
function [locs1, locs2, bestf] = findmatches(feats1, vpts1, feats2, vpts2)

  % find the matched features and retrieve their locations.
  [matches] = matchFeatures(feats1, feats2); 
  [locs1, transmat1] = normalize(vpts1.Location(matches(:, 1), :));
  [locs2, transmat2] = normalize(vpts2.Location(matches(:, 2), :));

  % invoke the ransac routine to find the fundamental matrix.  
  [bestf, inliers] = ransacf(locs1, locs2, 500, 1e-3); sum(inliers)
  [locs1, locs2] = deal(locs1(inliers, :) * inv(transmat1)', ...
                        locs2(inliers, :) * inv(transmat2)');

  % de-normalize the fundamental matrix.  
  bestf = transmat2' * bestf * transmat1; 

end
