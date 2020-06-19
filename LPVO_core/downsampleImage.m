function [downsampledImg] = downsampleImage(srcImg)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: downsampleImage
%
% Description:
%   Get the down sampling (default down sample rate is 2, other down sample rate
%   like 4,8... can be realized by execute this func for multiple times) image of the input image.
%
% Example:
%   OUTPUT:
%   downsampledImg: downsampled image. (double)  [m x n image] -> [m/2 x n/2 image]
%
%   INPUT:
%   srcImg: The input source image (double)
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

%% check the srcImg

if size(srcImg,3) == 3
    srcImg = double(rgb2gray(srcImg));
else
    srcImg = double(srcImg);
end

%% do down sampling

downsampledImg = ( srcImg(0+(1:2:end), 0+(1:2:end)) + srcImg(1+(1:2:end), 0+(1:2:end)) + ...
    srcImg(0+(1:2:end), 1+(1:2:end)) + srcImg(1+(1:2:end), 1+(1:2:end)) ) * 0.25;

end