
[im1, feats1, vpts1] = impreproc('left001.jpg');
[im2, feats2, vpts2] = impreproc('right001.jpg');

[matches] = matchFeatures(feats1, feats2);

locs1 = vpts1(matches(:, 1), :);  
locs2 = vpts2(matches(:, 2), :);

[f, inliers] = ransacfm(locs1, locs2, 500, 0.01);

tic; triangulate(locs1, locs2, f, 'optimal'); toc;
tic; triangulate(locs1, locs2, f); toc; 
