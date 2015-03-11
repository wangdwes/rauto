
% use fullfile instead of slashes to ensure cross-platform compatibility.  
load(fullfile('sensor', 'hand_carry.mat'));
load(fullfile('assets', 'useful.mat')); 

[point] = zeros(3, 1); count = length(left_image_names);
[points, thetas] = deal(zeros(3, count)); cumrot = eye(3);
[im1, f1, vpts1] = impreproc(left_image_names{1});
% iterate through all successive images and find their incremental transformation. 
for index = 2: count,

  % it's hard to implement a rolling array here so we just do it the hard way. 
  if mod(index, 2) == 0, 
    [im2, f2, vpts2] = impreproc(left_image_names{index});
    [locs1, locs2, fm] = optmatch(f1, vpts1, f2, vpts2);
  else
    [im1, f1, vpts1] = impreproc(left_image_names{index});
    [locs1, locs2, fm] = optmatch(f2, vpts2, f1, vpts1);
  end


%  if mod(index, 2) == 0,
%    showMatchedFeatures(im1, im2, locs1(:, 1: 2), locs2(:, 1: 2)); 
%    waitforbuttonpress;
%  else
%    showMatchedFeatures(im2, im1, locs1(:, 1: 2), locs2(:, 1: 2)); 
%    waitforbuttonpress;
%  end


  % find the rotation and translation between successive frames.
  % and store a trajectory of the camera origins.  
  [rot, tr] = camtrf(locs1, locs2, fm, intriparams); index
  points(:, index) = point; point = rot * (point - tr); cumrot = cumrot * rot;

  % store some euler angles as well for easier plotting.
  thetas(1, index) = atan2(cumrot(3, 2), cumrot(3, 3));
  thetas(2, index) = atan2(-cumrot(3, 1), sqrt(cumrot(3, 2).^2+cumrot(3,3)^2));
  thetas(3, index) = atan2(cumrot(2, 1), cumrot(1, 1));

end

   
