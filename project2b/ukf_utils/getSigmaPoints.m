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

function spmat = getSigmaPoints(x,P,zeta)
% INPUTS:
%--------------------------------------------------------------------------
% x      the (augmented) state vector
% P      the (augmented) state covariance
% zeta   a UKF coefficient determining the spread of the sigma points
%
% OUTPUT:
%--------------------------------------------------------------------------
% spmat  a matrix whose columns contain sigma points
%        For a vector x of size n, let columns 1 to n of spmat store the
%        sigma points formed by perturbing each of the dimensions of x in
%        the positive direction. Let columns n+1 to 2n of spmat store the
%        sigma points formed by perturbing each of the dimensions of x in
%        the negative direction. Let the last column (2n+1) of spmat store
%        a copy of the vector x itself.
%
% HINT:
%--------------------------------------------------------------------------
% You may find the MATLAB function chol useful.

matroot = zeta * chol(P, 'lower');
spmat = horzcat(bsxfun(@plus, x, matroot), bsxfun(@plus, x, -matroot), x); 

end
