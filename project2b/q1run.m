
% use fullfile instead of slashes to ensure cross-platform compatibility.  
load(fullfile('sensor', 'hand_carry.mat'));
load(fullfile('assets', 'useful.mat')); translation = [-0.1621, 0, 0]';

[point] = zeros(3, 1); count = length(left_image_names);
[points] = deal(zeros(3, count)); h = waitbar(0, 'Please wait...'); 

% iterate through all successive images and find their incremental transformation. 
for index = 1: count - 1, waitbar(index / count, h); 

  % extract features and find correspondences for stereo image pairs.
  % and invoke our triangulate method to find our their three-dimensional points.  
  [iml, featl, vptl] = impreproc(left_image_names{index});
  [imr, featr, vptr] = impreproc(right_image_names{index}); 
  [matches] = matchFeatures(featl, featr); 
  [locs3d] = triangulate(vptl(matches(:, 1), :), vptr(matches(:, 2), :), ...
                         intriparams * eye(3, 4), intriparams * horzcat(eye(3), translation));

  % this is apparently not efficient - but for proof of concept it should be fine.  
  [ims, feats, vpts] = impreproc(left_image_names{index + 1}); 
  [matchest] = matchFeatures(featl, feats);
  [~, ia, ib] = intersect(matches(:, 1), matchest(:, 1));
  [~, locb] = ismember(matches(ia, 1), matches(:, 1)); 

  % compute the camera matrix for later decomposition into r and t.  
  cm = cameramat(locs3d(locb, :), vpts(matchest(ib, 2), :)); 
  [rot, tr] = camat2rt(cm / cm(3, 3)); 

  % store some euler angles as well for easier plotting.
  points(:, index) = point; point = rot * point - tr;

end
