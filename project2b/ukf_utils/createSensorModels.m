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

function [smvec] = createSensorModels(params)

smvec = cell(0,1);

%% initialize IMU observation type
imu_params.state_indices = [params.pos_ind ...
                            params.rpy_ind ...
                            params.vel_ind ...
                            params.bw_ind ...
                            params.ba_ind];   
                        
imu_params.noise_indices = [params.nw_ind ...
                            params.nbw_ind ...
                            params.na_ind ...
                            params.nba_ind]; 
                        
imu_params.updateType = correctionType.process;

dyn_params.pos_ind = params.pos_ind;
dyn_params.rpy_ind = params.rpy_ind;
dyn_params.vel_ind = params.vel_ind;
dyn_params.bw_ind = params.bw_ind;
dyn_params.ba_ind = params.ba_ind;
dyn_params.nw_ind = params.nw_ind;
dyn_params.nbw_ind = params.nbw_ind;
dyn_params.na_ind = params.na_ind;
dyn_params.nba_ind = params.nba_ind;
dyn_params.dim = numel(imu_params.state_indices);
dyn_params.gravity = params.gravity;

imu_params.functionhandle = @(x,u,n,dt) processDynamics(dyn_params,x,u,n,dt); 
imu_params.dim_obs = numel(imu_params.state_indices);

tmp = zeros(1,numel(imu_params.noise_indices));
tmp(params.nw_ind) = params.sigma_w^2.*ones(1,numel(params.nw_ind));
tmp(params.nbw_ind) = params.sigma_bw^2.*ones(1,numel(params.nbw_ind));
tmp(params.na_ind) = params.sigma_a^2.*ones(1,numel(params.na_ind));
tmp(params.nba_ind) = params.sigma_ba^2.*ones(1,numel(params.nba_ind));

imu_params.covariance = diag(tmp);
  
imu_params.inputType = measurementType.imu;

imu = observation_type(imu_params);

smvec{end+1} = imu;

%% initialize Altitude + 2D Pose observation type
a2d_params.state_indices = [params.pos_ind params.rpy_ind(3)];
a2d_params.noise_indices = 1:numel(a2d_params.state_indices);
a2d_params.updateType = correctionType.absolute;
a2d_params.functionhandle = @(x,n) x(a2d_params.state_indices) + n;
a2d_params.dim_obs = numel(a2d_params.state_indices);

tmp = zeros(1,numel(a2d_params.state_indices));
tmp(1:2) = params.sigma_xy^2.*ones(1,2);
tmp(3) = params.sigma_z^2;
tmp(4) = params.sigma_yaw^2;
a2d_params.covariance = diag(tmp);
a2d_params.inputType = measurementType.a2d;
a2d = observation_type(a2d_params);

smvec{end+1} = a2d;

end


