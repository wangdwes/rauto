
[im1, feats1, vpts1] = imload('left001.jpg');
[im2, feats2, vpts2] = imload('left080.jpg');

[matches] = matchFeatures(feats1, feats2);

locs1 = vpts1(matches(:, 1), :); locs1b = locs1; 
locs2 = vpts2(matches(:, 2), :); locs2b = locs2; 

f1 = estimateFundamentalMatrix(locs1, locs2, 'Method', 'RANSAC')

[locs1, transmat1] = normalize(double(locs1.Location));
[locs2, transmat2] = normalize(double(locs2.Location));

[f, inliers] = ransacf(locs1, locs2, 500, 0.1);
f2 = transmat2' * f * transmat1

f3 = transmat2' * fundamat(locs1, locs2) * transmat1 

