
% use fullfile instead of slashes to ensure cross-platform compatibility.  
load(fullfile('sensor_data', 'hand_carry.mat'));
load(fullfile('assets', 'useful.mat')); translation = [-0.1621, 0, 0]';

[point] = zeros(3, 1); count = length(left_image_names);
[points] = deal(zeros(3, count)); h = waitbar(0, 'Please wait...'); 

% iterate through all successive images and find their incremental transformation. 
for index = 1: count - 1, waitbar(index / (count - 1), h); 

  % extract features and find correspondences for stereo image pairs.
  % and invoke our triangulate method to find our their three-dimensional points.  
  [iml, featl, vptl] = impreproc(left_image_names{index});
  [imr, featr, vptr] = impreproc(right_image_names{index}); 
  [matches] = matchFeatures(featl, featr);
  [locs3d] = triangulate(vptl(matches(:, 1), :) * inv(intriparams)', ...
                         vptr(matches(:, 2), :) * inv(intriparams)', ... 
                         eye(3, 4), horzcat(eye(3), translation)); 

  % this is apparently not efficient - but for proof of concept it should be fine.  
  [ims, feats, vpts] = impreproc(left_image_names{index + 1}); 
  [matchest] = matchFeatures(featl, feats); 
  [~, ia, ib] = intersect(matches(:, 1), matchest(:, 1));
  [~, locb] = ismember(matches(ia, 1), matches(:, 1)); 

  % compute the camera matrix for later decomposition into r and t.  
  extriparams = cameramat(locs3d(locb, :), vpts(matchest(ib, 2), :) * inv(intriparams)'); 
  extriparams = extriparams / nthroot(det(extriparams(:, 1: 3)), 3); 

  % ...
  rot = extriparams(:, 1: 3); tr = extriparams(:, 4); 
  points(:, index) = point; point = rot * point + tr; 

end
close(h); 
