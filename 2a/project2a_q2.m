clear; clc; close all;
addpath('lib2a');

%% Variables
% Calibration image directory
imgDir = 'camera_calibration';
matDir = 'mat';
% Square size in millimeters
squareSize = 152.3;
% Calibration image indices (max = 3473)
calInd = [72 2796 2835 3220 3400];
% Correspondence image points
ptsInd = 1;
% Whether to use saved results
useResults = 1;
% Whether to save results
saveResults = 0;

%% Load saved results
if useResults
    load(fullfile(matDir, 'params.mat'));
end
    
%% Calibration
% Convert indices to names
imageNamesL = arrayfun(@(i) fullfile(imgDir, sprintf('rawleft%04d.jpg', i)), calInd, 'UniformOutput', false);
imageNamesR = arrayfun(@(i) fullfile(imgDir, sprintf('rawright%04d.jpg', i)), calInd, 'UniformOutput', false);
%imdisp([imageNamesL, imageNamesR], 'Size', [2 5]);

% Estimate the left, right, and stereo (combined) camera parameters
fprintf('Estimating camera parameters... ');
if ~useResults
    paramsS = camcalib(squareSize, imageNamesL, imageNamesR);
end
params1 = paramsS.CameraParameters1;
params2 = paramsS.CameraParameters2;
fprintf('done!\n');

%% Undistort image
img1d = imread(fullfile(imgDir, sprintf('rawleft%04d.jpg', ptsInd)));
img2d = imread(fullfile(imgDir, sprintf('rawright%04d.jpg', ptsInd)));
img1 = undistortImage(img1d, params1);
img2 = undistortImage(img2d, params2);

%% Calculate F and E
fprintf('Estimating F and E... ');
load(fullfile(matDir, sprintf('pts%04d_15.mat', ptsInd)));
K1 = params1.IntrinsicMatrix'; % Remember to tranpose K since MATLAB returns row-order
K2 = params2.IntrinsicMatrix';
if ~useResults
    %F = eightpoint_norm(pts1, pts2, max(size(img1)));
    [F, inliers] = ransacF(pts1, pts2, max(size(img1)));
    E = K2' * F * K1;
    E = E / E(3,3);
end
fprintf('done!\n');

%% Calculate R and t
fprintf('Calculating R and t... ');
if ~useResults
    [R, t] = getRt(-E, K1, K2, pts1(:,1), pts2(:,1));
    M1 = K1 * [eye(3,3) [0 0 0]'];
    M2 = K2 * [R t];
end
fprintf('done!\n');

%% Verify x'^T*F*x=0
errF = cell2mat(cellfun(@(p1, p2) [p2; 1]' * F * [p1; 1], num2cell(pts1, 1), num2cell(pts2, 1), 'UniformOutput', false));

%% Ground truths
gtF = paramsS.FundamentalMatrix;
gtF = gtF / gtF(3,3);
gtE = paramsS.EssentialMatrix;
gtE = gtE / gtE(3,3);

load(fullfile(matDir, 'cameraParameter.mat'));
yuhanF = stereoParams.FundamentalMatrix;
yuhanF = yuhanF / yuhanF(3,3);
yuhanE = stereoParams.EssentialMatrix;
yuhanE = yuhanE / yuhanE(3,3);

[gtR, gtt] = getRt(gtE, K1, K2, pts1(:, 1), pts2(:, 2));
[yuhanR, yuhant] = getRt(yuhanE, K1, K2, pts1(:, 1), pts2(:, 2));

%% Save Results if flag on
if saveResults
    save(fullfile(matDir, 'params.mat'), 'paramsS', 'E', 'F', 'R', 't');
end
