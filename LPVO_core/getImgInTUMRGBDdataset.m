function [targetImage] = getImgInTUMRGBDdataset(datasetPath, TUMRGBDdataset, cam, frameIdx, imgType)
% Project:    Depth enhanced visual odometry
% Function:  getImgInTUMRGBDdataset
%
% Description:
%   get gray, rgb, and depth image from TUM RGBD dataset
%
% Example:
%   OUTPUT:
%   targetImage : imported images to workspace from [datasetPath]
%                  if imgType is 'gray' -> targetImage is a gray image. (uint8)
%                  if imgType is 'depth' -> targetImage is a depth image [m]. (double)
%                  if imgType is 'rgb' -> targetImage is a rgb image. (uint8)
%
%   INPUT:
%   datasetPath: absolute path to TUM RGBD dataset directory
%   TUMRGBDdataset: compact and synchronized rgb, depth, Vicon datasets
%   cam: camera calibration parameters
%   frameIdx: frame index of TUM RGBD dataset ( 1-based index )
%   ImgType: the type of images ( 'gray' or 'depth' or 'rgb' )
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2016-12-01: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


% read rgb and depth images
imRgb = imread([ datasetPath '/' TUMRGBDdataset.rgb.imgName{frameIdx} ]);
imDepth = imread([ datasetPath '/' TUMRGBDdataset.depth.imgName{frameIdx} ]);


% determine what imgType is ( 'gray' or 'depth' or 'rgb' )
if ( strcmp(imgType, 'gray') )
    
    % convert rgb to gray image
    targetImage = uint8(rgb2gray(imRgb));
    
    % show current situation
    fprintf('----''%s'' images are imported to workspace [%04d] ---- \n', imgType, frameIdx);
    
    
elseif ( strcmp(imgType, 'depth') )
    
    % convert raw depth image to depth image [m]
    targetImage = double( double(imDepth) / cam.scaleFactor );
    
    % show current situation
    fprintf('----''%s'' images are imported to workspace [%04d] ---- \n', imgType, frameIdx);
    
    
elseif ( strcmp(imgType, 'rgb') )
    
    % raw rgb image
    targetImage = imRgb;
    
    % show current situation
    fprintf('----''%s'' images are imported to workspace [%04d] ---- \n', imgType, frameIdx);
    
    
else
    
    fprintf('\nWrong input(imgType) parameter!!! \n');
    fprintf('What is the ''%s'' ??\n',imgType);
    error('Wrong input(imgType) parameter!!!')
    
end

end