clear; clc; close all;

%% Variables
% Calibration images directory
calDir = 'camera_calibration';
% Square size in millimeters
squareSize = 152.3;
% Calibration image indices (max = 3473)
calInd = [72 2796 2535 3220 3400];

%% Calibration
% Convert indices to names
imageNames = arrayfun(@(i) fullfile(calDir, sprintf('rawright%04d.jpg', i)), calInd, 'UniformOutput', false);
imdisp([imageNames], 'Size', [1 5]);

% Estimate the camera parameters
fprintf('Estimating camera parameters... ');
params = camcalib(squareSize, imageNames);
fprintf('done!\n');
