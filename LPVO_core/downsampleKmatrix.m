function [downsampledKmatrix] = downsampleKmatrix(srcKmatrix)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: downsampleKmatrix
%
% Description:
%   get the down sampling K matrix ( camera calibration matrix )
%
% Example:
%   OUTPUT:
%   downsampledKmatrix: downsampled camera calibration matrix
%
%   INPUT:
%   srcKmatrix: original camera calibration matrix ( intrinsic parameter K )
%
% NOTE:
%          srcKmatrix = [fx   0   cx;
%                             0   fy  cy;
%                             0   0   1]
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2016-04-05 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

% this is because we interpolate in such a way, that
% the image is discretized at the exact pixel-values (e.g. 3,7), and
% not at the center of each pixel (e.g. 3.5, 7.5).
downsampledKmatrix = [srcKmatrix(1,1)/2 0 (srcKmatrix(1,3)+0.5)/2-0.5;
    0 srcKmatrix(2,2)/2 (srcKmatrix(2,3)+0.5)/2-0.5;
    0 0 1];

end