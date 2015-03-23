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

function [P,RPY,V,BW,BA] = imuProcessDynamics(p,rpy,v,bw,ba,nw,nbw,na,nba,wm,am,dt,gravity)
% INPUTS:
%--------------------------------------------------------------------------
% p        a 3 x 1 vector of position
% rpy      a 3 x 1 vector of roll, pitch, yaw
% v        a 3 x 1 vector of velocity
% bw       a 3 x 1 vector of IMU body frame angular velocity bias
% ba       a 3 x 1 vector of IMU body frame linear acceleration bias
% nw       a 3 x 1 vector of additive angular velocity noise
% nbw      a 3 x 1 vector of additive angular acceleration noise
% na       a 3 x 1 vector of additive linear acceleration noise
% nba      a 3 x 1 vector of additive linear jerk noise
% wm       a 3 x 1 vector of the measured body angular velocity
% am       a 3 x 1 vector of the measured body linear acceleration
% dt       a scalar denoting the size of the time interval we are forward
%          simulating over
%
% OUTPUTS:
%--------------------------------------------------------------------------
% The outputs are the forward propagated values of their respective
% lower-case quantities (e.g. P is p propagated dt into the future).
%
% HINT:
%--------------------------------------------------------------------------
% You may find the function ZYXToR.m in the geometry_utils folder useful.
% This function takes a 3 x 1 roll-pitch-yaw vector and converts it into a
% rotation matrix.

  % load the suggested useful function in a civilized manner. 
  seta = [1, sin(rpy(1)) * tan(rpy(2)), cos(rpy(1)) * tan(rpy(2)); ...
          0, cos(rpy(1)), -sin(rpy(1)); ...
          0, sin(rpy(1)) * sec(rpy(2)), cos(rpy(1)) * sec(rpy(2))]; 

  % derive the actual acceleration and angular velocity from the measurements, 
  % following equations 3 and 4 in the implemenation guide. 
  [a, w] = deal(inv(ZYXToR(rpy))' * (am - na - ba) - [0, 0, 1]' * gravity, wm - bw - nw); 

  % compute the propogated state variables, following equations 5, 6, 7, 8, 9.  
  [BA, BW] = deal(ba + nba * dt, bw + nbw * dt);
  [P, V, RPY] = deal(p + v * dt, v + a * dt, rpy + seta * w * dt); 

end
