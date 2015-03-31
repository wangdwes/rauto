clear; close all;
addpath('lib');

%% Constants
% Sensor image directory
imgDir = 'sensor_data';
interval = 3;
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
Ts = zeros(4, 4, ptsIndMax);
Rs = zeros(3, 3, ptsIndMax);
ts = zeros(3, ptsIndMax);
ZYXs = zeros(3, ptsIndMax);
ZYXsdeg = zeros(3, ptsIndMax);
n=280;
% Go through each image pair
fprintf('Starting...\n');
for ptsInd=1:ptsIndMax
%     fprintf('ptsInd: %d\n', ptsInd*interval);
    %% Load image pair
    imgL1 = imread(fullfile(imgDir, sprintf('left%03d.jpg', ptsInd*interval)));
    imgR1 = imread(fullfile(imgDir, sprintf('right%03d.jpg', ptsInd*interval)));
    imgL2 = imread(fullfile(imgDir, sprintf('left%03d.jpg', (ptsInd+1)*interval)));
    imgR2 = imread(fullfile(imgDir, sprintf('right%03d.jpg', (ptsInd+1)*interval)));
%     imdisp([imgL, imgR], 'Size', [2, 2]);

    %% Extract features across image pair (part a)
    IL1 = rgb2gray(imgL1);
    IR1 = rgb2gray(imgR1);
    IL2 = rgb2gray(imgL2);
    IR2 = rgb2gray(imgR2);

    pointsL1 = detectHarrisFeatures(IL1);
    pointsR1 = detectHarrisFeatures(IR1);
    pointsL2 = detectHarrisFeatures(IL2);
    pointsR2 = detectHarrisFeatures(IR2);

    [featuresL1, valid_pointsL1] = extractFeatures(IL1, pointsL1);
    [featuresR1, valid_pointsR1] = extractFeatures(IR1, pointsR1);
    [featuresL2, valid_pointsL2] = extractFeatures(IL2, pointsL2);
    [featuresR2, valid_pointsR2] = extractFeatures(IR2, pointsR2);

    %% Match features across image pair (part b)
    indexPairsLR1 = matchFeatures(featuresL1, featuresR1);
    indexPairsLR2 = matchFeatures(featuresL2, featuresR2);
    
    matchedPointsL1 = valid_pointsL1(indexPairsLR1(:, 1), :);
    matchedPointsR1 = valid_pointsR1(indexPairsLR1(:, 2), :);
    matchedPointsL2 = valid_pointsL2(indexPairsLR2(:, 1), :);
    matchedPointsR2 = valid_pointsR2(indexPairsLR2(:, 2), :);
    
    pL1 = matchedPointsL1.Location';
    pR1 = matchedPointsR1.Location';
    pL2 = matchedPointsL2.Location';
    pR2 = matchedPointsR2.Location';

%     figure;
%     showMatchedFeatures(IL1,IR1,matchedPointsL2,matchedPointsR2,'montage');

    %% Triangulate (part c)
    Praw = triangulate2(ML, pL1, MR, pR1);
    x_xp = pL1(1,:) - pR1(1,:);
    Z = b*f ./ x_xp;
    P1 = Praw;
    P1(3,:) = Praw(3,:) .* (Z ./ Praw(3,:));
%     P1 = triangulate2(ML, pL1, MR, pR1);
    
    Praw = triangulate2(ML, pL2, MR, pR2);
    x_xp = pL2(1,:) - pR2(1,:);
    Z = b*f ./ x_xp;
    P2 = Praw;
    P2(3,:) = Praw(3,:) .* (Z ./ Praw(3,:));
%     P2 = triangulate2(ML, pL2, MR, pR2);
    
%     PP = P2
%     figure(2);
%     scatter(PP(1,:), PP(2,:));
%     scatter3(PP(1,:), PP(2,:), PP(3,:));
%     xlabel('x/right');
%     ylabel('y/down');
%     zlabel('z/forward');
%     xlim([-5, 5])
%     ylim([-5, 5])
%     zlim([0, 20])

    %% Match features across successive left image (part d)
    % Filter to get the set of points we know 3D points of
    featuresL1f = binaryFeatures(featuresL1.Features(indexPairsLR1(:, 1), :));
    featuresL2f = binaryFeatures(featuresL2.Features(indexPairsLR2(:, 1), :));
    
    indexPairsL12 = matchFeatures(featuresL1f, featuresL2f);
    
    matchedPointsL1f = matchedPointsL1(indexPairsL12(:, 1), :);
    matchedPointsL2f = matchedPointsL2(indexPairsL12(:, 2), :);
    
%     showMatchedFeatures(IL1,IL2,matchedPointsL1f,matchedPointsL2f,'montage');

    PL1 = P1(:, indexPairsL12(:, 1));
    PL2 = P2(:, indexPairsL12(:, 2));
    
%     PP = PL1;
%     figure(2);
%     hold on;
%     scatter(PP(1,:), PP(2,:));
%     scatter3(PP(1,:), PP(2,:), PP(3,:), 'bo');
%     PP = PL2;
%     scatter(PP(1,:), PP(2,:));
%     scatter3(PP(1,:), PP(2,:), PP(3,:), 'ro');
%     xlabel('x/right');
%     ylabel('y/down');
%     zlabel('z/forward');
%     xlim([-5, 5])
%     ylim([-5, 5])
%     zlim([0, 20])
%     hold off;

    %% Compute the motion to the next frame
    % Camera matrix of the next frame
    [T, inliers] = ransac3D(PL1, PL2);
    fprintf('ptsInd: %d... total: %d, inliers: %d\n', ptsInd*interval, size(PL2, 2), length(inliers));
    RR = T(1:3, 1:3);
    tt = T(1:3, 4);
    
    % plotting
%     matchedPointsL1fin = matchedPointsL1f(inliers);
%     matchedPointsL2fin = matchedPointsL2f(inliers);
%     showMatchedFeatures(IL1,IL2,matchedPointsL1fin,matchedPointsL2fin,'montage');
%     PL1in = PL1(:, inliers);
%     PL2in = PL2(:, inliers);
%     PP = PL2in;
%     figure(2);
%     scatter(PP(1,:), PP(2,:));
%     scatter3(PP(1,:), PP(2,:), PP(3,:));
%     xlabel('x/right');
%     ylabel('y/down');
%     zlabel('z/forward');
%     xlim([-5, 5])
%     ylim([-5, 5])
%     zlim([0, 20])
    
    % Record results
    Ts(:, :, ptsInd) = T;
    Rs(:, :, ptsInd) = RR;
    ts(:, ptsInd) = tt;
    ZYXs(:, ptsInd) = RToZYX(RR);
    deg = RToZYX(RR) / pi*180;
    ZYXsdeg(:, ptsInd) = deg;
end

fprintf('done!\n');

