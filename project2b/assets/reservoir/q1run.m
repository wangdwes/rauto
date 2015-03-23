
load('sensor_data/hand_carry.mat');
load('assets/useful.mat');

% allocate some storage for our trajectory.  
point = zeros(3, 1); [points, thetas] = deal(zeros(3, length(left_image_names))); 

rrot = eye(3);  

% iterate through all successive images and find their 
% incremental transformation. 
[im1, feats1, vpts1] = imload(left_image_names{1});
for index = 2: length(left_image_names)

%  [im1, feats1, vpts1] = imload(left_image_names{index - 1});
%  [im2, feats2, vpts2] = imload(left_image_names{index});

  % it's hard to implement rolling array here, so we use the hard way. 
  if mod(index, 2) == 0, 
    [im2, feats2, vpts2] = imload(left_image_names{index});
    [locs1, locs2, fm] = findmatches(feats1, vpts1, feats2, vpts2);  
  else
    [im1, feats1, vpts1] = imload(left_image_names{index});
    [locs1, locs2, fm] = findmatches(feats2, vpts2, feats1, vpts1);
  end

  locs1r = bsxfun(@rdivide, locs1(:, 1: 3), locs1(:, 3));
  locs2r = bsxfun(@rdivide, locs2(:, 1: 3), locs2(:, 3));

%    locs1r = [locs1, ones(size(locs1, 1), 1)];
%    locs2r = [locs2, ones(size(locs2, 1), 1)];

%  if mod(index, 2) == 0, 
%    showMatchedFeatures(im1, im2, locs1r(:, 1: 2), locs2r(:, 1: 2));
%  else
%    showMatchedFeatures(im1, im2, locs1r(:, 1: 2), locs2r(:, 1: 2));
%  end

%  title(num2str(index)); 
%  waitforbuttonpress; clf

  [rot, tr] = camtrf(locs1r, locs2r, fm, intriparams);
  points(:, index) = point; point = rot * (point - tr); index

  rrot = rrot * rot; det(rrot) 

%  thetas(:, index) = rotationMatrix2eulerAngles(rrot);

  thetas(1, index) = atan2(rrot(3, 2), rrot(3, 3));
  thetas(2, index) = atan2(-rrot(3, 1), sqrt(rrot(3, 2)^2+rrot(3,3)^2));
  thetas(3, index) = atan2(rrot(2, 1), rrot(1, 1));

end

thetas(1, 311:end) = thetas(1, 311:end)-2*pi;

