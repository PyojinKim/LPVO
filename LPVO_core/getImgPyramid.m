function [I1RefPyr, D1RefPyr] = getImgPyramid(I1Ref, D1Ref, maxPyramidLevel)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: getImgPyramid
%
% Description:
%   read input and depth images, and generate their pyramid images.
%
% Example:
%   OUTPUT :
%   I1RefPyr: gray image pyramid of the selected reference gray image
%   D1RefPyr: depth image pyramid with respect to I1RefPyr
%
%   INPUT :
%   I1Ref: A rgb or gray input image (with uint8 type)
%   D1Ref: A depth input image (with double type)
%   maxPyramidLevel: options for PIVO process like below
%
% NOTE:
%
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-03-22 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

% image pyramid reduction
for i = 2:maxPyramidLevel
    I1Ref = impyramid(I1Ref, 'reduce'); %downsampleImage(I1Ref);
    D1Ref = downsampleDepth(D1Ref);
end
I1RefPyr = I1Ref;
D1RefPyr = D1Ref;


end
