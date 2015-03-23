
function [im, feats, vpts] = impreproc(name)

  % load the image and detect the keypoints.
  im = im2double(rgb2gray(imread(fullfile('sensor_data', name))));
  points = detectSURFFeatures(im); 

  % obtain the valid points and the features, 
  % then homogenize the coordinates. 
  [feats, vpts] = extractFeatures(im, points);
  [vpts] = horzcat(double(vpts.Location), ones(vpts.Count, 1));

end 
