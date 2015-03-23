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

classdef observation_type < handle
   properties
       state_indices      % indices in the main state used as inputs to the observation equation
       noise_indices      % indices in the noise vector used as inputs to the observation equation
       
       dim_obs            % dimension of the observation output
       dim_noise          % dimension of the noise vector
       
       updateType         % absolute, process
       inputType          % a measurementType enum        
       
       functionhandle     % function handle for observation equation
       
       covariance         % optional, a symmetric positive definite dim_obs x dim_obs tuning matrix
       
   end
   
   methods
       
       function obj = observation_type(params)
           obj.state_indices = params.state_indices;
           obj.noise_indices = params.noise_indices;
           
           obj.dim_obs = params.dim_obs;
           obj.dim_noise = numel(obj.noise_indices);
           obj.updateType = params.updateType;
           
           obj.functionhandle = params.functionhandle;
           
           if isfield(params,'covariance')
              obj.covariance = params.covariance; 
              cov_size = size(obj.covariance);
              if numel(cov_size) ~= 2
                  error('covariance must be a 2D matrix');
              end
              if cov_size(1) ~= cov_size(2)
                  error('covariance must be a square matrix');
              end
              if cov_size(1) ~= obj.dim_noise
                  error('covariance must be dim_noise x dim_noise');
              end
              if norm(obj.covariance - obj.covariance','fro') > eps
                  error('covariance must be symmetric');
              end
              [~,isnotposdef] = chol(obj.covariance);
              if isnotposdef 
                  error('covariance must be positive definite');
              end
           else
              obj.covariance = [];
           end
           
           obj.inputType = params.inputType;
       end
       
       function h = predictObservation(obj,x_main, varargin)
          
           if nargin == 3
               n = varargin{1};
           elseif nargin == 4
               n = varargin{2};
           elseif nargin == 5
               u = varargin{1};
               n = varargin{2};
               dt = varargin{3};
           end
           
           switch obj.updateType
               case corretionType.process
                   h = obj.functionhandle(x_main,u,n,dt);
               case correctionType.absolute
                   h = obj.functionhandle(x_main,n);
               otherwise
                   error('obj.updateType invalid');
           end
       end
       
       % class destructor
       function delete(obj)
       end
       
   end
end