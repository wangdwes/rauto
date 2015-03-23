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

function [] = reseedSigmaPoints(obj, sensor_ind)

x_aug_mean = [obj.state;
              zeros(obj.sensors{sensor_ind}.dim_noise,1)];

obj.dim_x_aug = numel(x_aug_mean);
obj.P_aug = zeros(obj.dim_x_aug);
obj.P_aug(1:obj.dim_x,1:obj.dim_x) = obj.covariance;

if ~isempty(obj.sensors{sensor_ind}.covariance)
    obj.P_aug(obj.dim_x+1:end,obj.dim_x+1:end) = obj.sensors{sensor_ind}.covariance;
end

zeta = obj.alpha*sqrt(obj.dim_x_aug + obj.kappa);
    
obj.sigma_points = getSigmaPoints(x_aug_mean, obj.P_aug, zeta);

end