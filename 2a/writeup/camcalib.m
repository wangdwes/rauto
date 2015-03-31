% Estimates the single/stereo camera calibration parameters.
% @params:
%     - squareSize: the size of checker board squares in millimeters (mm).
%     - imageNames: a cell array of relative-path image file names. Left
%       image files in case of stereo.
%     - (optional) varargin{1}: a cell array of relative-path right image file
%       names in case of stereo.
%     - params: a struct that contains the camera calibration information.

function params = camcalib(squareSize, imageNames, varargin)
    % Detect images points on the checker board from top left in column major
    if nargin > 2
        [imagePoints, boardSize] = detectCheckerboardPoints(imageNames, varargin{1});
    else
        [imagePoints, boardSize] = detectCheckerboardPoints(imageNames);
    end

    % Generate world points on the checker board from top left in column major
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Estimate the camera parameters
    params = estimateCameraParameters(imagePoints, worldPoints);
end


