% Prepared by John Yao in support of CMU RI Course:
% 16-662 (Spring 2015) Robot Autonomy, Instructors: Michael, Kitani
% Copyright (c) 2015, John Yao
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are 
% met:
%
% 1. Redistributions of source code must retain the above copyright notice,
%    this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright 
%    notice, this list of conditions and the following disclaimer in the 
%    documentation and/or other materials provided with the distribution.
%
% 3. Neither the name of the copyright holder nor the names of its 
%    contributors may be used to endorse or promote products derived from 
%    this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

clear; clc; close all;

fpstr = mfilename('fullpath');
pth = [fpstr(1:strfind(fpstr,'cmu_16662_p2')-1) 'cmu_16662_p2/sensor_data/'];
load([pth 'hand_carry.mat']);

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

% plot exteroceptive sensor observations
figure;
for ii = 1:4
    subplot(4,1,ii);
    plot(cor_timestamps,cor_observations(ii,:),'-r');
    xlabel('time [s]');
    if ii < 4
        ylabel([xyz{ii} ' [m]']);
    else
        ylabel('yaw [rad]');
    end
    grid on; box on;
    xlim([cor_timestamps(1) cor_timestamps(end)]);
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
