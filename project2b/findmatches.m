
function [locs1, locs2, bestf] = findmatches(feats1, vpts1, feats2, vpts2)

  % find the matched features and retrieve their locations.
  [matches] = matchFeatures(feats1, feats2);
  [locs1, transmat1] = normalize(vpts1.Location);
  [locs2, transmat2] = normalize(vpts2.Location);

  % invoke the ransac routine to find the fundamental matrix.  
  [bestf, inliers] = ransacf(matches, locs1, locs2, 200, 1e-3);
  ilmatches = matches(inliers, :);
  locs1 = locs1(ilmatches(:, 1), :);
  locs2 = locs2(ilmatches(:, 2), :);
 
  % de-normalize this.  
  bestf = transmat2' * bestf * transmat1; 

end
