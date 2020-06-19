function [ICSLRGBDdataset] = rawICSLRGBDdataset_load(datasetPath, imInit, M)
% Project:    Depth enhanced visual odometry
% Function:  rawICSLRGBDdataset_load
%
% Description:
%   get raw rgb, depth from ICSL RGBD dataset
%
% Example:
%   OUTPUT:
%   ICSLRGBDdataset:
%
%   INPUT:
%   datasetPath: directory of folder
%   imInit: first image index (1-based index)
%   M: number of images
%
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-03-15: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


% load ICSL RGBD dataset data
imgName = dir([datasetPath '/depth/']);
imgName(1:2) = [];
imgName = imgName(imInit:(imInit+M-1));

depthImageDirs = 'depth/';
colorImageDirs = 'rgb/';


% re-arrange k-th image
for k = 1:M
    ICSLRGBDdataset.depth.imgName{k} = [depthImageDirs imgName(k).name];
    ICSLRGBDdataset.rgb.imgName{k} = [colorImageDirs imgName(k).name];
end


end
