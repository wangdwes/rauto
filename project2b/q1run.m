
load('sensor/hand_carry.mat');
load('assets/useful.mat');

% allocate some storage for our trajectory.  
point = zeros(3, 1); [points, thetas] = deal(zeros(3, length(left_image_names))); 

rrot = eye(3);  

% iterate through all successive images and find their 
% incremental transformation. 
[im1, feats1, vpts1] = imload(left_image_names{1});
for index = 2: length(left_image_names)

  % it's hard to implement rolling array here, so we use the hard way. 
  if mod(index, 2) == 0, 
    [im2, feats2, vpts2] = imload(left_image_names{index});
    [locs1, locs2, fm] = findmatches(feats1, vpts1, feats2, vpts2);  
  else
    [im1, feats1, vpts1] = imload(left_image_names{index});
    [locs2, locs1, fm] = findmatches(feats2, vpts2, feats1, vpts1);
  end

  [rot, tr] = camtrans(locs1, locs2, fm, intriparams);
  points(:, index) = point; point = rot * (point - tr); index

  rrot = rrot * rot; 

  thetas(1, index) = atan2(rrot(3, 2), rrot(3, 3));
  thetas(2, index) = atan2(-rrot(3, 1), sqrt(rrot(3, 2)^2+rrot(3,3)^2));
  thetas(3, index) = atan2(rrot(2, 1), rrot(1, 1));

end

