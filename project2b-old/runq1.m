
% load the data and the parameters.  
load('sensor/hand_carry.mat');
load('assets/useful.mat');

point = zeros(3, 1);
points = zeros(3, length(left_image_names)); 

for index = 1: length(left_image_names) - 1   

  [locs1, locs2, f] = findmatch(frames{index}, frames{index});
  [r, t] = camtrans(locs1, locs2, f, intriparams);

  points(:, index) = point; index
  point = r * (point - t);

end

