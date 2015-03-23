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

classdef ukf < handle
    
    properties
        
        % UKF parameters that determine sigma point spread
        alpha
        beta
        kappa
        
        % dimension of the state vector
        dim_x
        
        % indices of the states within the state vector
        pos_ind
        rpy_ind
        vel_ind
        bw_ind
        ba_ind
        
        % indices of the process noise states within the process noise vector
        nw_ind
        nbw_ind
        na_ind
        nba_ind
        
        % current time
        cur_time
        
        % current state vector and state covariance
        state
        covariance
        
        % dimension of the augmented state vector
        dim_x_aug
        
        % dimension of the process noise vector
        dim_process_noise
        
        % the augmented state covariance matrix (with noise states)
        P_aug
        
        % a cell array of sensor objects
        sensors
        
        % matrix whose columns contain sigma points
        sigma_points
        
        % a vector of UKF weighting coefficients for determining the mean
        wimvec
        
        % a vector of UKF weighting coefficients for determining the
        % covariance
        wicvec
        
        % current and previous observation types
        cur_obs_type
        prev_obs_type
        
        % store the last IMU observation
        last_imu
        
        % a scalar, gravitational acceleration in m/s^2
        gravity
        
        % data containers for timestamps, state, and covariance
        t_history
        x_history
        P_history
        
    end
    
    methods
        
        function obj = ukf(params,sensors)
            
           % set indices of main states
           obj.pos_ind = params.pos_ind;
           obj.rpy_ind = params.rpy_ind;
           obj.vel_ind = params.vel_ind;
           obj.bw_ind = params.bw_ind;
           obj.ba_ind = params.ba_ind;
           
           % set indices of process noise states
           obj.nw_ind = params.nw_ind;
           obj.nbw_ind = params.nbw_ind;
           obj.na_ind = params.na_ind;
           obj.nba_ind = params.nba_ind;
           
           obj.state = [params.init_pos';
                        params.init_rpy';
                        params.init_vel';
                        params.init_bw';
                        params.init_ba'];
                    
           obj.dim_x = numel(obj.state);
           
           tmp = zeros(1,obj.dim_x);
           tmp(obj.pos_ind) = params.init_pos_sigma^2.*ones(1,numel(obj.pos_ind));
           tmp(obj.vel_ind) = params.init_vel_sigma^2.*ones(1,numel(obj.vel_ind));
           tmp(obj.rpy_ind) = params.init_rpy_sigma^2.*ones(1,numel(obj.rpy_ind));
           tmp(obj.bw_ind) = params.init_bw_sigma^2.*ones(1,numel(obj.bw_ind));
           tmp(obj.ba_ind) = params.init_ba_sigma^2.*ones(1,numel(obj.ba_ind));
           obj.covariance = diag(tmp);

           obj.dim_process_noise = numel(obj.nw_ind) + numel(obj.nbw_ind) + ...
                                   numel(obj.na_ind) + numel(obj.nba_ind);
            
           obj.alpha = params.alpha;
           obj.beta = params.beta;
           obj.kappa = params.kappa;
            
           obj.sensors = sensors;
           
           obj.cur_time = params.cur_time;
           obj.last_imu = params.last_imu;
            
           obj.gravity = params.gravity;
           
           obj.cur_obs_type = measurementType.unknown;
           obj.prev_obs_type = measurementType.unknown;
            
           obj.t_history = obj.cur_time;
           obj.x_history = obj.state;
           obj.P_history = obj.covariance;
            
        end
        
        [ ] = updateEstimate(obj,obsvec)
        
        [ ] = reseedSigmaPoints(obj, sensor_ind)
        
        [ ] = getSPWeights(obj)
        
        [xprior,Pprior,propagated_sp] = processUpdate(obj, dt, u, spmat) 
        
        [xout] = processDynamics(obj,x,u,n,dt)
        
        [xpost, Ppost] = absoluteCorrection(obj, z, spmat, xprior, Pprior)
        
        [ind] = measurementType2SensorIndex(obj, mt)
        
        % Class destructor
        function delete(obj)
        end
        
    end
    
end