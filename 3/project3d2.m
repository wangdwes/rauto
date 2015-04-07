clear; clc; close all;
addpath('gtsam_toolbox');
import gtsam.*
addpath('lib');

%% Sensor image directory
dataDir = 'sensor_data';%'NSHLevel2_Images';

nImages = 600;%4068-1;
nIter = nImages;
startImage = 1;

%% Camera parameters (fx fy skew cx cy baseline)
fx = 164.255034407511;
fy = 164.255034407511;
skew = 0.0;
cx = 214.523999214172;
cy = 119.433252334595;
b = 0.1621;
% Camera intrinsic matrix
K = [fx,  skew, cx;
     0.0, fy,   cy;
     0.0, 0.0,  1.0];
% Right camera rotation wrt left
R = eye(3,3);
% Right camera translation wrt left (x = right; y = down, z = forward)
t = [-b, 0.0, 0.0]';
% Camera matrices (left and right)
ML = K * eye(3, 4);
MR = K * [R t];

%% Variables
% Result containers
Ms = zeros(3, 4, nIter);
cs = zeros(3, nIter);
Ks = zeros(3, 3, nIter);
Rs = zeros(3, 3, nIter);
ts = zeros(3, nIter);
Ts = zeros(4, 4, nIter);
ZYXs = zeros(3, nIter);
ZYXsdeg = zeros(3, nIter);

% Feature trail containers
trail = cell(1, 1); % Just like to declare outside (size varies every loop)
tmptrail = cell(1, 1);
trailThres = 5;
stereoThres = 5;
triThres = 2;

estimates = cell(1, nIter);

%% GTSAM Initialization
% Create graph container and add factors to it
graph = NonlinearFactorGraph;

% State and initial estimation containers
states = cell(1, nIter);
nLandmarks = 0;
initialEstimate = Values;

% Create realistic calibration and measurement noise model
% Format: fx fy skew cx cy baseline
Kgt = Cal3_S2Stereo(fx, fy, skew, cx, cy, b);
stereoModel = noiseModel.Isotropic.Sigma(3, 1);

%% Go through image pairs
fprintf('Starting...\n');

stereocount = 0;
tricount = 0;
negzcount = 0;

nSkip = 0;

for iIter=startImage:200%nIter
    fprintf('iIter: %d   ', iIter);

    %% Feature matching
    % Load image pair
    imgL = imread(fullfile(dataDir, sprintf('left%03d.jpg', iIter-1)));
    imgR = imread(fullfile(dataDir, sprintf('right%03d.jpg', iIter-1)));

    % Extract features across image pair
    IL = rgb2gray(imgL);
    IR = rgb2gray(imgR);
%     imdisp([IL IR]);

    pointsL = detectHarrisFeatures(IL);%, 'MinQuality', 0.0001);
    pointsR = detectHarrisFeatures(IR);%, 'MinQuality', 0.0001);

    [featuresL, valid_pointsL] = extractFeatures(IL, pointsL);
    [featuresR, valid_pointsR] = extractFeatures(IR, pointsR);

    % Match features across image pairs
    indexPairsLR = matchFeatures(featuresL, featuresR);
    
    matchedFeaturesL = binaryFeatures(featuresL.Features(indexPairsLR(:, 1), :));
    matchedPointsL = valid_pointsL(indexPairsLR(:, 1), :);
    matchedPointsR = valid_pointsR(indexPairsLR(:, 2), :);
    matchedLength = length(indexPairsLR);
    fprintf('Matched features: %d\n', matchedLength);

    pL = double(valid_pointsL(indexPairsLR(:, 1), :).Location');
    pR = double(valid_pointsR(indexPairsLR(:, 2), :).Location');
    
    % Triangulate
    Praw = triangulate(ML, pL, MR, pR);
    x_xp = pL(1, :) - pR(1, :);
    Z = b*fx ./ x_xp;
    P = Praw;
    P(3, :) = Praw(3, :) .* (Z ./ Praw(3, :));
    PPP = triangulate(ML, pL, MR, pR);
    
    %% Pose estimation
    % Get rotation and translation (except the first frame, which saves all features)
    if iIter == startImage
        % Record results
        Ms(:, :, iIter) = ML;
        Ks(:, :, iIter) = K;
        Rs(:, :, iIter) = eye(3, 3);
        Ts(:, :, iIter) = eye(4, 4);
    else
        indexPairsL12 = matchFeatures(lastFeaturesL, matchedFeaturesL);
        pL2 = matchedPointsL(indexPairsL12(:, 2), :).Location';
        
        % If not enough (< 6) correspondences, then skip this image
%         if size(pL2, 2) < 6
%             nSkip = nSkip + 1;
%             fprintf('Only %d features, skipping...\n', size(pL2, 2));
%             continue;
%         else
%             nSkip = 0;
%         end
        
        % Estimate camera matrix between last frame and this frame (in
        % last frame coordinates)
        [ML1, inliers] = ransacM(pL2, lastP(:, indexPairsL12(:, 1)));
        % Camera center
        cL1 = lsvect(ML1);
        % Intrinsic K and rotation R using googled rq script
        [KL1, RL1] = rq(ML1(:, 1:3));
        % Force diagonals of K to be positive (same as ground truth)
        RL1 = bsxfun(@times, RL1, sign(diag(KL1))); % Force rows to have the same signs
        KL1 = bsxfun(@times, KL1, sign(diag(KL1))'); % Force columns to have the same signs    

        % Record results
        Ms(:, :, iIter) = ML1;
        cs(:, iIter) = cL1(1:3);
        Ks(:, :, iIter) = KL1;
        Rs(:, :, iIter) = RL1;
        tL1 = -RL1 * cL1(1:3);
        ts(:, iIter) = tL1;
        Ts(:, :, iIter) = Ts(:, :, iIter-1-nSkip) * inv([RL1, tL1; 0 0 0 1]);
        ZYXs(:, iIter) = RToZYX(RL1);
        ZYXsdeg(:, iIter) = RToZYX(RL1) / pi*180;
    end
    
    %% Record feature trail
    for iFeature=1:matchedLength
        % If seen before, increament length and append point;
        if iIter ~= startImage && ismember(iFeature, indexPairsL12(:, 2))
            trailind = indexPairsL12(find(indexPairsL12(:, 2) == iFeature), 1); %#ok<FNDSB>
            oldfeature = trail{trailind};
            if oldfeature.length == 0
                tmptrail{iFeature}.length = 0;
            else
                tmptrail{iFeature}.length = oldfeature.length + 1;
            end
            tmptrail{iFeature}.imageIndex = [oldfeature.imageIndex, iIter];
            tmptrail{iFeature}.stereoPoint = [oldfeature.stereoPoint, {StereoPoint2(pL(1, iFeature), pR(1, iFeature), pL(2, iFeature))}];
            tmptrail{iFeature}.triangulation = [oldfeature.triangulation, P(:, iFeature)];
            
            % Pass landmark if it exists
            if oldfeature.length >= trailThres
                tmptrail{iFeature}.landmark = oldfeature.landmark;
                tmptrail{iFeature}.landmarkIndex = oldfeature.landmarkIndex;
            end
            
            % Invalidate feature if difference b/w stereo v/y coordinates is large
            if tmptrail{iFeature}.length > 0
                if abs(pL(2, iFeature) - pR(2, iFeature)) > stereoThres
                    tmptrail{iFeature}.length = 0;
                    fprintf('\tFeature: %d   stereo diff: %f\n', iFeature, pL(2, iFeature) - pR(2, iFeature));
                    stereocount = stereocount + 1;
                end
            end

            % Invalidate feature if range of 3D points is too large (noisy)
            if tmptrail{iFeature}.length > 0
                triPoints = tmptrail{iFeature}.triangulation;
                if norm(max(triPoints, [], 2) - min(triPoints, [], 2)) > triThres
                    tmptrail{iFeature}.length = 0;
                    fprintf('\tFeature: %d   3d diff: %f\n', iFeature, norm(max(triPoints, [], 2) - min(triPoints, [], 2)));
                    tricount = tricount + 1;
                end
            end

            % Invalidate feature if Z is negative (behind camera)
            if tmptrail{iFeature}.length > 0
                triPoint = tmptrail{iFeature}.triangulation(:, 1);
                if triPoint(3) < 0.0 || triPoint(3) > 20.0
                    tmptrail{iFeature}.length = 0;
                    fprintf('\tFeature: %d   negative z: %f\n', iFeature, triPoint(3));
                    negzcount = negzcount + 1;
                end
            end

        % Otherwise it's a new feature so set length 1 and make new cell to store point
        else
            tmptrail{iFeature}.length = 1;
            tmptrail{iFeature}.imageIndex = iIter;
            tmptrail{iFeature}.stereoPoint = {StereoPoint2(pL(1, iFeature), pR(1, iFeature), pL(2, iFeature))};
            tmptrail{iFeature}.triangulation = P(:, iFeature);
            
            % Invalidate feature if difference b/w stereo v/y coordinates is large
            if abs(pL(2, iFeature) - pR(2, iFeature)) > stereoThres
                tmptrail{iFeature}.length = 0;
                fprintf('stereo diff: %f\n', pL(2, iFeature) - pR(2, iFeature));
                stereocount = stereocount + 1;
            end
        end
        % Use the feature from this frame even if seen before to get most up-to-date feature
        %tmptrail{iFeature}.feature = featuresL.Features(iFeature, :);
    end
    % Update feature trail
    trail = tmptrail;
    tmptrail = cell(1, 1);

    % Preparing for next frame
    lastFeaturesL = matchedFeaturesL;
    lastP = P;
    
    %% GTSAM
    % Add state
    x = symbol('x',iIter);
    states{iIter} = x;
    
    % Set up GTSAM in first iteration
    if iIter == startImage
        %% Add a constraint on the starting pose
        firstPose = Pose3();
        graph.add(NonlinearEqualityPose3(x, firstPose));
        initialEstimate.insert(x, firstPose);
    else
        initialEstimate.insert(x, Pose3(Ts(:, :, iIter)));
    end
    
    % Add landmark measurements (add all five if length == trailThres) and initial estimate when first time we pass threshold
    for iFeature=1:length(trail)
        feature = trail{iFeature};
        
        % If feature invalidated, then don't use it
        if feature.length <= 0
            continue;
        end
        
        if feature.length > trailThres
            % Add feature for this landmark (current frame)
            graph.add(GenericStereoFactor3D(feature.stereoPoint{end}, stereoModel, x, feature.landmark, Kgt));
        elseif feature.length == trailThres
            % Add new landmark
            nLandmarks = nLandmarks + 1;
            feature.landmarkIndex = nLandmarks;
            feature.landmark = symbol('l', nLandmarks);
            trail{iFeature} = feature; % Store back since feature has changed
            % Add all five features for this landmark (past four and current frame)
            for iState=1:trailThres
                graph.add(GenericStereoFactor3D(feature.stereoPoint{iState}, stereoModel, states{feature.imageIndex(iState)}, feature.landmark, Kgt));
            end
            % Add initial estimate of the first frame the landmark's seen
            % (3D landmarks are stored in camera coordinates: transform
            % to world coordinates using the respective initial pose)
            pose = initialEstimate.at(states{feature.imageIndex(1)});
            worldPoint = pose.transform_from(Point3(feature.triangulation(:, 1)));
            initialEstimate.insert(feature.landmark, worldPoint);
        end
    end
    
    estimates{iIter} = Values(initialEstimate);
    % Plotting
%     if true || rem(iIter, 1) == 0
%         figure(1);
%         showMatchedFeatures(IL, IR, matchedPointsL, matchedPointsR, 'montage');
%     end
end

%% optimize
fprintf(1,'Optimizing\n');
tic
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
toc

fprintf('stereo: %d   tri: %d   negz: %d\n', stereocount, tricount, negzcount);
fprintf('done!\n');

%% Visualize initial trajectory, final trajectory, and final points
cla; hold on;
axis normal
%axis([-1.5 1.5 -2 2 -1 6]);
axis equal
view(-38,12)
camup([0;1;0]);
xlabel('x/right');
ylabel('y/down');
zlabel('z/forward');

%plot3DTrajectory(initialEstimate, 'r', 1, 0.3);
plot3DTrajectory(result, 'g', 1, 0.3);
plot3DPoints(result);
