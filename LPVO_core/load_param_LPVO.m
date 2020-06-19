function [optsLPVO] = load_param_LPVO
% Project:   Line And Plane Odometry
% Function: load_param_LPVO
%
% Description:
%   get the initial parameters with respect to the algorithm
%
% Example:
%   OUTPUT:
%   optsLPVO: options for LPVO process like below
%
%   INPUT:
%
%
% NOTE:
%     The parameters below are initialized as the CVPR paper
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-07-25: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


% line detection and matching parameters
optsLPVO.lineDetector = 'lsd';     % 'gpa'
optsLPVO.lineLength = 25;


% plane detection and tracking parameters
optsLPVO.imagePyramidLevel = 2;
optsLPVO.minimumDepth = 0.3;
optsLPVO.maximumDepth = 8;
optsLPVO.cellsize = 10;

optsLPVO.numInitialization = 200;
optsLPVO.iterNum = 200;
optsLPVO.convergeAngle = deg2rad(0.001);
optsLPVO.halfApexAngleNV = deg2rad(10);
optsLPVO.halfApexAngleVD = deg2rad(10);
optsLPVO.c = 20;
optsLPVO.ratio = 0.1;
optsLPVO.minSampleRatio = (1/10);
optsLPVO.planeDensityThreshold = 0.80;


end






