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

function [xprior,Pprior,propagated_sp] = processUpdate(obj, dt, u, spmat)
% INPUTS:
%--------------------------------------------------------------------------
% dt       a scalar denoting the size of the time interval we are forward
%          simulating over
% u        is the input vector of IMU measurements that should be passed
%          into the processDynamics function
% spmat    a matrix that stores the sigma points found by the most recent
%          call to the reseedSigmaPoints function
%
% OUTPUTS:
%--------------------------------------------------------------------------
% xprior          a column vector representing the forward propagated state
% Pprior          a matrix representing the forward propagated state covariance
% propagated_sp   a matrix whose columns contain the propagated sigma points
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
% USEFUL FUNCTIONS:
%--------------------------------------------------------------------------
% obj.processDynamics  propagates a sigma point through the process
%                      dynamics

  % feed all the sigma points into the dynamics propogation process and obtain the outputs.  
  % note that since our covariance matrix is known to be diagonal, computation is simplified.  
  propagated_sp = zeros(obj.dim_x, size(spmat, 2)); 
  for col = 1: size(spmat, 2),  
    propagated_sp(:, col) = obj.processDynamics(spmat(1: obj.dim_x, col), u, ...
      spmat(obj.dim_x + 1: end, col), dt); end

  % compute the forward propogated state and its covariance. 
  xprior = sum(bsxfun(@times, obj.wimvec, propagated_sp), 2); % per equation 23. 
  pspac = bsxfun(@minus, propagated_sp, xprior); % this and the following line: equation 24. 
  Pprior = sum(bsxfun(@times, bsxfun(@times, permute(pspac, [1 3 2]), ... 
    permute(pspac, [3 1 2])), permute(obj.wicvec, [1 3 2])), 3); 

end
