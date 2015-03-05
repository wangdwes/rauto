
load('sensor/hand_carry.mat');
load('assets/useful.mat');

% allocate some storage for our trajectory.  
point = zeros(3, 1); points = zeros(3, length(left_image_names)); 

% iterate through all successive images and find their 
% incremental transformation. 
[im1, feats1, vpts1] = imload(left_image_names{1});
for index = 2: length(left_image_names);

  % it's hard to implement rolling array here, so we use the hard way. 
  if mod(index, 2) == 0, 
    [im2, feats2, vpts2] = imload(left_image_names{index});
    [locs1, locs2, fm] = findmatches(feats1, vpts1, feats2, vpts2);
  else
    [im1, feats1, vpts1] = imload(left_image_names{index});
    [locs2, locs1, fm] = findmatches(feats1, vpts1, feats2, vpts2);
  end

  [rot, tr] = camtrans(locs1, locs2, fm, intriparams); tr = [0.1, 0, 0]';
  points(:, index) = point; point = rot * (point - tr); index

end

