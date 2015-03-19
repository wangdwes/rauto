% Author:      John Yao
% Description: Example matlab code for parsing image and IMU data for 
%              project 2B

clear; clc; close all;

fpstr = mfilename('fullpath');
load(['project2b/sensor/hand_carry.mat']);

% cell array for left camera images
left_images = cell(numel(left_image_names),1);

for ii = 1:numel(left_image_names)
    left_images{ii,1} = imread([pth left_image_names{ii}]);
end
  
% cell array for right camera images
right_images = cell(numel(right_image_names),1);

for ii = 1:numel(right_image_names)
    right_images{ii,1} = imread([pth right_image_names{ii}]);
end

%% Example Usage
xyz = {'x','y','z'};

% plot IMU body frame acceleration
figure; 
for ii = 1:3
    subplot(3,1,ii);
    plot(imu_timestamps,body_accel(ii,:),'-r');
    xlabel('time [s]');
    ylabel(['a_' xyz{ii} ' [m/s^2]']);
    grid on; box on; 
    xlim([imu_timestamps(1) imu_timestamps(end)]);
end

% plot IMU body frame angular velocity
figure; 
for ii = 1:3
    subplot(3,1,ii);
    plot(imu_timestamps,body_angvel(ii,:),'-r');
    xlabel('time [s]');
    ylabel(['\omega_' xyz{ii} ' [rad/s]']);
    grid on; box on; 
    xlim([imu_timestamps(1) imu_timestamps(end)]);
end

% find index corresponding to a time of ts seconds
ts = 10;
tind = find(image_timestamps < ts,1,'last');

% display left and right images
figure;
subplot(1,2,1);
imshow(left_images{tind,1});
title('left camera');
subplot(1,2,2);
imshow(right_images{tind,1});
title('right camera');
