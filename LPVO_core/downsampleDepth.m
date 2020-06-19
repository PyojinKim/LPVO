function [downsampledDepthImg] = downsampleDepth(srcDepthImg)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: downsampleDepth
%
% Description:
%   Get the down sampling (default down sample rate is 2, other down sample rate
%   like 4,8... can be realized by execute this func for multiple times) image of the input depth image.
%
% Example:
%   OUTPUT:
%   downsampledDepthImg: downsampled depth image. (double)  [m x n image] -> [m/2 x n/2 image]
%
%   INPUT:
%   srcDepthImg: The input source depth image (double)
%
% NOTE:
%      The input image must a single channel one.
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2015-04-05 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

%% do down sampling

downsampledDepthImg = ( srcDepthImg(0+(1:2:end), 0+(1:2:end)) + srcDepthImg(1+(1:2:end), 0+(1:2:end)) + srcDepthImg(0+(1:2:end), 1+(1:2:end)) + srcDepthImg(1+(1:2:end), 1+(1:2:end)) ) ./ ...
    ( sign(srcDepthImg(0+(1:2:end), 0+(1:2:end))) + sign(srcDepthImg(1+(1:2:end), 0+(1:2:end))) + sign(srcDepthImg(0+(1:2:end), 1+(1:2:end))) + sign(srcDepthImg(1+(1:2:end), 1+(1:2:end))) );

downsampledDepthImg(isnan(downsampledDepthImg)) = 0;

end