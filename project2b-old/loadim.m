
% load the data and the parameters.  
load('sensor/hand_carry.mat');
frames = cellfun(@(name) {im2double(rgb2gray(imread(fullfile('sensor', name))))}, left_image_names); 
