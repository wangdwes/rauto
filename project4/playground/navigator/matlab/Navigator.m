 
% Copyright (c) 2014-2015, Team Evolutus
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the copyright holder nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

classdef Navigator < handle

  properties (SetAccess = private, Hidden = true)
    objectHandle % handle to the underlying C++ class instance. 
  end 

  methods

    % constructor: create a new class instance.  
    function this = Navigator(varargin) 
      this.objectHandle = navigator_mex('new', varargin{:});
    end
    
    % deconstructor: destroy the created class instance.
    function delete(this) 
      navigator_mex('delete', this.objectHandle); 
    end

    % other member functions. see .h for detailed description.  

    function varargout = createObstacle(this, varargin)
      [varargout{1: nargout}] = navigator_mex('createObstacle', this.objectHandle, varargin{:}); end

    function varargout = updateObstacle(this, varargin)
      [varargout{1: nargout}] = navigator_mex('updateObstacle', this.objectHandle, varargin{:}); end 

    function varargout = removeObstacle(this, varargin)
      [varargout{1: nargout}] = navigator_mex('removeObstacle', this.objectHandle, varargin{:}); end 
    
    function varargout = setStart(this, varargin) 
      [varargout{1: nargout}] = navigator_mex('setStart', this.objectHandle, varargin{:}); end 

    function varargout = setGoal(this, varargin) 
      [varargout{1: nargout}] = navigator_mex('setGoal', this.objectHandle, varargin{:}); end 

    function varargout = plan(this, varargin) 
      [varargout{1: nargout}] = navigator_mex('plan', this.objectHandle, varargin{:}); end 

    function varargout = getArena(this, varargin) 
      [varargout{1: nargout}] = navigator_mex('getArena', this.objectHandle, varargin{:}); end 

    function varargout = getIdentifiers(this, varargin) 
      [varargout{1: nargout}] = navigator_mex('getIdentifiers', this.objectHandle, varargin{:}); end 

    % an additional function that is only available in the matlab interface. 
    function varargout = getArenaImage(this, varargin)
      arena = navigator_mex('getArena', this.objectHandle, varargin{:});
      arena(arena > 0) = 255; arena(arena < 0) = -arena(arena < 0); varargout = {uint8(arena)};
    end 

  end
end
 
