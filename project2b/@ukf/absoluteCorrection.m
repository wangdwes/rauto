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

function [xpost, Ppost] = absoluteCorrection(obj, z, spmat, xprior, Pprior)
% INPUTS:
%--------------------------------------------------------------------------
% z        a column vector of exteroceptive observations on [x y z yaw]'
% xprior   a column vector containing the current state mean
% Pprior   a matrix containing the current state covariance
% spmat    a matrix that stores the sigma points found by the most recent
%          call to the reseedSigmaPoints function
%
% OUTPUTS:
%--------------------------------------------------------------------------
% xpost          the posterior (after applying the correction) state mean
% Ppost          the posterior state covariance
%
% USEFUL VARIABLES:
%--------------------------------------------------------------------------
% obj.dim_x           dimension of the regular state
% obj.dim_x_aug       the dimension of the augmented state (regular state +
%                     noise vector)
% obj.wimvec          UKF weighting coefficients (used to compute mean)
% obj.wicvec          UKF weighting coefficients (used to compute covariance)
%                     The ith elements of wimvec and wicvec correspond to
%                     the ith sigma point in spmat.
%
% HINT:
%--------------------------------------------------------------------------
% Normally the innovation is computed as z - h, where z is the measurement
% and h is the predicted measurement, but this will not always produce the
% correct results for the yaw state. For example, if
%    z_yaw = pi-epsilon
%    h_yaw = -pi+epsilon
% Then the innovation computed by z - h will yield 2*pi-2*epsilon rad, which
% is huge. The correct way to handle this wraparound situation is to add
% 2*pi to h_yaw so that z_yaw - h_yaw is -2*epsilon. This is one of the
% drawbacks of using an Euler angle representation of rotation.
% The function shortest_angular_distance(h,z) in geometry_utils handles
% z - h correctly for the yaw state.

  % propagate the augmented state sigma points through the measurement model. 
  propagated_sp = spmat([obj.pos_ind, obj.rpy_ind(3)], :) + spmat(obj.dim_x + 1: end, :); 
  % repmat(z, 1, size(spmat, 2))

  % reconstruct the mean and the covariance
  pspmean = sum(bsxfun(@times, obj.wimvec, propagated_sp), 2);
  pspac = bsxfun(@minus, propagated_sp, pspmean); 
  pzz = sum(bsxfun(@times, bsxfun(@times, permute(pspac, [1 3 2]), ...
    permute(pspac, [3 1 2])), permute(obj.wicvec, [1 3 2])), 3); 
  pxz = sum(bsxfun(@times, bsxfun(@times, permute(bsxfun(@minus, spmat, spmat(:, end)), [1 3 2]), ...
    permute(pspac, [3 1 2])), permute(obj.wicvec, [1 3 2])), 3); 

  % correction update. 
  kalgain = pxz * inv(pzz); kalgain = kalgain(1: end - numel(z), :); 
  Ppost = Pprior - kalgain * pzz * kalgain';  
  xpost = xprior + kalgain * ([z(1: 3) - pspmean(1: 3); ...
    shortest_angular_distance(pspmean(4), z(4))]); 

end
