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

function xout = processDynamics(obj,x,u,n,dt)

    p = x(obj.pos_ind);
    rpy = x(obj.rpy_ind);
    v = x(obj.vel_ind);
    bw = x(obj.bw_ind);
    ba = x(obj.ba_ind);
    
    nw = n(obj.nw_ind);
    nbw = n(obj.nbw_ind);
    na = n(obj.na_ind);
    nba = n(obj.nba_ind);
    
    wm = u(1:3);
    am = u(4:6);
    
    [p_next, rpy_next, v_next, bw_next, ba_next] = ...
        imuProcessDynamics(p,rpy,v,bw,ba,nw,nbw,na,nba,wm,am,dt,obj.gravity);
           
    xout = zeros(obj.dim_x,1);
    
    xout(obj.pos_ind) = p_next;
    xout(obj.rpy_ind) = rpy_next;
    xout(obj.vel_ind) = v_next;
    xout(obj.bw_ind) = bw_next;
    xout(obj.ba_ind) = ba_next;

end