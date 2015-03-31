clear; clc; close all;
addpath('lib');

%% Constants
% Sensor image directory
imgDir = 'sensor_data';
interval = 1;
% Image pair index max (634)
ptsIndMax = floor(634/interval) - 1;
b = 0.1621;
f = 164.255034407511;

%% Initialize calibration results
K = [f, 0.0, 214.523999214172;
     0.0, f, 119.433252334595;
     0.0, 0.0, 1.0];
R = eye(3,3);
% x = right; y = down, z = forward
t = [-b, 0.0, 0.0]';
ML = K * eye(3, 4);
MR = K * [R t];

%% Result containers
Ms = zeros(3, 4, ptsIndMax);
cs = zeros(3, ptsIndMax);
Ks = zeros(3, 3, ptsIndMax);
Rs = zeros(3, 3, ptsIndMax);
ts = zeros(3, ptsIndMax);
ZYXs = zeros(3, ptsIndMax);
ZYXsdeg = zeros(3, ptsIndMax);

% Go through each image pair
fprintf('Starting...\n');
for ptsInd=1:ptsIndMax
    fprintf('ptsInd: %d\n', ptsInd*interval);
    %% Load image pair
    imgL = imread(fullfile(imgDir, sprintf('left%03d.jpg', ptsInd*interval)));
    imgR = imread(fullfile(imgDir, sprintf('right%03d.jpg', ptsInd*interval)));
    imgL1 = imread(fullfile(imgDir, sprintf('left%03d.jpg', (ptsInd+1)*interval)));

    %% Extract features across image pair (part a)
    IL = rgb2gray(imgL);
    IR = rgb2gray(imgR);
    IL1 = rgb2gray(imgL1);

    pointsL = detectHarrisFeatures(IL);
    pointsR = detectHarrisFeatures(IR);
    pointsL1 = detectHarrisFeatures(IL1);

    [featuresL, valid_pointsL] = extractFeatures(IL, pointsL);
    [featuresR, valid_pointsR] = extractFeatures(IR, pointsR);
    [featuresL1, valid_pointsL1] = extractFeatures(IL1, pointsL1);

    %% Match features across image pair (part b)
    indexPairsLR = matchFeatures(featuresL, featuresR);
    indexPairsLR = indexPairsLR(:, :);

    matchedPointsL = valid_pointsL(indexPairsLR(:, 1), :);
    matchedPointsR = valid_pointsR(indexPairsLR(:, 2), :);

    pL = matchedPointsL.Location';
    pR = matchedPointsR.Location';
    
%     figure;
%     showMatchedFeatures(IL, IR, matchedPointsL, matchedPointsR, 'montage');

    %% Triangulate (part c)
    % Disparity depth matching?
%     Praw = triangulate2(ML, pL, MR, pR);
%     x_xp = pL(1, :) - pR(1, :);
%     Z = b*f ./ x_xp;
%     P = Praw;
%     P(3, :) = Praw(3, :) .* (Z ./ Praw(3, :));
    P = triangulate2(ML, pL, MR, pR);
    
%     figure;
%     grid on;
%     scatter(P(1, :), P(2, :));
%     scatter3(P(1, :), P(2, :), P(3, :));
%     xlabel('x/right (m)');
%     ylabel('y/down (m)');
%     zlabel('z/forward (m)');
%     xlim([-5, 5])
%     ylim([-5, 1])
%     zlim([0, 8])

    %% Match features across successive left image (part d)
    % Filter to get the set of points we know 3D points of
    featuresL0 = binaryFeatures(featuresL.Features(indexPairsLR(:, 1), :));
    valid_pointsL0 = cornerPoints(valid_pointsL.Location(indexPairsLR(:, 1), :));

    indexPairsL01 = matchFeatures(featuresL0, featuresL1);

    matchedPointsL0 = valid_pointsL0(indexPairsL01(:, 1), :);
    matchedPointsL1 = valid_pointsL1(indexPairsL01(:, 2), :);

    pL0 = matchedPointsL0.Location';
    pL1 = matchedPointsL1.Location';
    
%     figure();
%     showMatchedFeatures(IL, IR, matchedPointsL, matchedPointsR, 'montage');

    % Get the corresponding 3D points
    PL = P(:, indexPairsL01(:, 1));
    
%     figure;
%     scatter(PL(1, :), PL(2, :));
%     scatter3(PL(1, :), PL(2, :), PL(3, :));
%     xlabel('x/right');
%     ylabel('y/down');
%     zlabel('z/forward');
%     xlim([-5, 5])
%     ylim([-5, 5])
%     zlim([0, 20])

    %% Compute the motion to the next frame
    % Camera matrix of the next frame
    [ML1, inliers] = ransacM(pL1, PL);
    fprintf('ransac total: %d, inliers: %d\n', size(pL1, 2), length(inliers));
    
%     figure;
%     showMatchedFeatures(IL, IR, matchedPointsL(inliers, :), matchedPointsR(inliers, :), 'montage');
    
    % Camera center
    cL1 = lsvect(ML1);
    % Intrinsic K and rotation R using googled rq script
    [KL1, RL1] = rq(ML1(:, 1:3));
    % Force diagonals of K to be positive (same as ground truth)
    RL1 = bsxfun(@times, RL1, sign(diag(KL1))); % Force rows to have the same signs
    KL1 = bsxfun(@times, KL1, sign(diag(KL1))'); % Force columns to have the same signs    
    
    % Record results
    Ms(:, :, ptsInd) = ML1;
    cs(:, ptsInd) = cL1(1:3);
    Ks(:, :, ptsInd) = KL1;
    Rs(:, :, ptsInd) = RL1;
    t = -RL1 * cL1(1:3);
    ts(:, ptsInd) = t;
    ZYXs(:, ptsInd) = RToZYX(RL1);
    deg = RToZYX(RL1) / pi*180;
    ZYXsdeg(:, ptsInd) = deg;
end

fprintf('done!\n');

