
% load the data and the parameters.  
load('sensor/hand_carry.mat');
load('assets/useful.mat');

im1 = im2double(rgb2gray(imread(fullfile('sensor', left_image_names{1}))));
im2 = im2double(rgb2gray(imread(fullfile('sensor', left_image_names{2}))));

[locs1, locs2, f] = findmatch(im1, im2);
[r, t] = camtrans(locs1, locs2, f, intriparams)
