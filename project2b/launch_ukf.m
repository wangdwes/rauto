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

close all; clear all; clc;

%% Load results
load('sensor_data/hand_carry.mat');
addpath('ukf_utils');
addpath('geometry_utils');

%% Add observations to the observation synchronizer for postprocessing
synchronizer = observation_synchronizer();

for ii = 1:length(imu_timestamps)
    synchronizer.add_observation(observation(imu_timestamps(ii),...
        [body_angvel(:,ii); body_accel(:,ii)], [], measurementType.imu));
end

for ii = 1:length(cor_timestamps)
    synchronizer.add_observation(observation(cor_timestamps(ii), ...
               cor_observations(:,ii), [],...
               measurementType.a2d));
end

synchronizer.sort_observations();

%% UKF Parameters
%
% The UKF tuning parameters are:
%        init_pos_sigma, init_rpy_sigma, init_vel_sigma, init_bw_sigma, init_ba_sigma
%        (these are the 1 sigma uncertainty levels on the initial estimates
%         of the vehicle state)
%
%        diag([sigma_w^2*ones(3,1);
%               sigma_bw^2*ones(3,1);
%               sigma_a^2*ones(3,1);
%               sigma_ba^2*ones(3,1)]) is the covariance matrix associated
%         with the IMU process noise
%
%        diag([sigma_xy^2*ones(2,1);
%              sigma_alt^2;
%              sigma_yaw^2]) is the covariance matrix associated with the
%        exteroceptive sensor additive noise
%
% Note that before being able to run the UKF you must replace the nan
% values of these parameters with finite positive values in the struct
% ukf_params.

ukf_params = struct(...
'alpha',          1e-3,...
'beta',           2,...
'kappa',          0,...
'pos_ind',        [1 2 3],...
'rpy_ind',        [4 5 6],...
'vel_ind',        [7 8 9],...
'bw_ind',         [10 11 12],...
'ba_ind',         [13 14 15],...
'nw_ind',         [1 2 3],...
'nbw_ind',        [4 5 6],...
'na_ind',         [7 8 9],...
'nba_ind',        [10 11 12],...
'init_pos',       [0.05 0.05 0.03],...
'init_rpy',       [0 0 1.241],...
'init_vel',       [0 0 0],...
'init_bw',        [0 0 0],...
'init_ba',        [0 0 0],...
'gravity',        9.8066,...
'cur_time',       0,...
'last_imu',       zeros(6,1),...
'init_pos_sigma', 0.01,...
'init_rpy_sigma', 0.001,...
'init_vel_sigma', 0.001,...
'init_bw_sigma',  0.001,...
'init_ba_sigma',  0.001,...
'sigma_w',        0.03,...
'sigma_bw',       0.03,...
'sigma_a',        0.08,...
'sigma_ba',       0.03,...
'sigma_xy',       0.077,...
'sigma_z',        0.05,...
'sigma_yaw',      0.02);

sensor_models = createSensorModels(ukf_params);

%% Run UKF
% Before running the UKF you should verify that you have correctly
% implemented imuProcessDynamics, getSigmaPoints, processUpdate, and
% absoluteCorrection.
ukfsim = ukf(ukf_params, sensor_models);

tic;
for ii = 2:length(synchronizer.observation_vector)
    ukfsim.updateEstimate(synchronizer.observation_vector{ii,1});
    fprintf('sim time = %6.5f \t type = %s \t elapsed time = %5.2f\n', ...
             synchronizer.observation_vector{ii,1}.time,char(ukfsim.cur_obs_type),toc);
end

%% Get UKF Output
ukf_timestamps = ukfsim.t_history;
ukf_position   = ukfsim.x_history(ukf_params.pos_ind,:);
ukf_velocity   = ukfsim.x_history(ukf_params.vel_ind,:);
ukf_rpy        = ukfsim.x_history(ukf_params.rpy_ind,:);
ukf_accelbias  = ukfsim.x_history(ukf_params.bw_ind,:);
ukf_gyrobias   = ukfsim.x_history(ukf_params.ba_ind,:);

%% Plot Results
plot(ukf_timestamps, ukf_position(1, :)); 
