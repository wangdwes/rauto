
% use fullfile instead of slashes to ensure cross-platform compatibility.  
load(fullfile('sensor', 'hand_carry.mat'));
load(fullfile('assets', 'useful.mat')); translation = [-0.1621, 0, 0]';

[point] = zeros(3, 1); count = length(left_image_names);
[points, thetas] = deal(zeros(3, count)); cumrot = eye(3);

% iterate through all successive images and find their incremental transformation. 
for index = 1: count - 1, index, 

  % extract features and find correspondences for stereo image pairs.
  % and invoke our triangulate method to find our their three-dimensional points.  
  [iml, featl, vptl] = impreproc(left_image_names{index});
  [imr, featr, vptr] = impreproc(right_image_names{index}); 
  [matches] = matchFeatures(featl, featr); 
  [locs3d] = triangulateb(vptl(matches(:, 1), :), vptr(matches(:, 2), :), ...
                         intriparams * eye(3, 4), intriparams * horzcat(eye(3), translation), '');  

  [locs3d] = triangulate(intriparams * eye(3, 4), ...
                         vptl(matches(:, 1), 1: 2)', ...
                         intriparams * horzcat(eye(3), translation), ...
                         vptr(matches(:, 2), 1: 2)'); 

  locs3d = horzcat(locs3d', ones(size(locs3d, 2), 1)); 

  % this is apparently not efficient - but for proof of concept it should be fine.  
  [ims, feats, vpts] = impreproc(left_image_names{index + 1}); 
  [matchest] = matchFeatures(featl, feats);
  [~, ia, ib] = intersect(matches(:, 1), matchest(:, 1)); 
  [~, locb] = ismember(matches(ia, 1), matches(:, 1)); 

  % compute the camera matrix for later decomposition into r and t.  
  cm = cameramat(locs3d(locb, :), vpts(matchest(ib, 2), :)); 
  extriparams = inv(intriparams) * cm; 
  extriparams = extriparams / det(extriparams(:, 1: 3)) ^ (1 / 3);  

  % store some euler angles as well for easier plotting.
  points(:, index) = point; 
  point = extriparams(:, 1: 3) * (point - extriparams(:, 4)); 

  % thetas(1, index) = atan2(cumrot(3, 2), cumrot(3, 3));
  % thetas(2, index) = atan2(-cumrot(3, 1), sqrt(cumrot(3, 2).^2+cumrot(3,3)^2));
  % thetas(3, index) = atan2(cumrot(2, 1), cumrot(1, 1));

end

   
