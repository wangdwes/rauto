
function [im, feats, vpts] = imload(fname)

  % load the image and find out some interesting points.  
  im = im2double(rgb2gray(imread(fullfile('sensor', fname))));
  points = detectSURFFeatures(im);

  % find out the valid points and the features. 
  [feats, vpts] = extractFeatures(im, points);

end

