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

function updateEstimate(obj, input)
    
    obj.cur_obs_type = input.type;
    
    % find the index of the sensor that corresponds with input.type
    sensor_ind = obj.measurementType2SensorIndex(input.type);
    
    % calculate dt
    dt = input.time - obj.cur_time;
    if dt < eps
        error('dt is zero or negative');
    end

    if obj.cur_obs_type == measurementType.imu
        
        if obj.prev_obs_type ~= measurementType.imu
             obj.reseedSigmaPoints(sensor_ind); 
        end
        
        obj.getSPWeights();
            
        [xprior, Pprior, propagated_sp] = obj.processUpdate(dt, ...
             input.data, obj.sigma_points);
        
        obj.state = xprior;
        obj.covariance = Pprior;
        obj.sigma_points(1:obj.dim_x,:) = propagated_sp;
        
        obj.last_imu = input.data;
        
    else
        
        imu_sensor_ind = obj.measurementType2SensorIndex(measurementType.imu);
        
        if obj.prev_obs_type ~= measurementType.imu
            obj.reseedSigmaPoints(imu_sensor_ind, correctionType.process);
        end
        
        obj.processUpdate(dt, obj.last_imu, obj.sigma_points);
          
        obj.reseedSigmaPoints(sensor_ind);
            
        obj.getSPWeights();
        
        [obj.state, obj.covariance] = obj.absoluteCorrection(input.data, ...
                                        obj.sigma_points, obj.state, ...
                                        obj.covariance);

    end
    
    % record history
    obj.t_history(end+1) = input.time; 
    obj.x_history(:,end+1) = obj.state;
    obj.P_history(:,:,end+1) = obj.covariance;
    
    % state transition
    obj.cur_time = input.time;
    obj.prev_obs_type = obj.cur_obs_type;

end