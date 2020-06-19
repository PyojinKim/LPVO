function [cam] = initialize_cam_ICSL_RGBD(cameraType, maxPyramidLevel)
% camera calibration parameters from ICSL RGBD dataset


% intrinsic camera calibration parameters from Matlab calibrated parameters (by me)
if (strcmp(cameraType, 'cam02'))
    load('cameraParams_ICSL_RGBD_2.mat');
elseif (strcmp(cameraType, 'cam03'))
    load('cameraParams_ICSL_RGBD_3.mat');
end
cam.K = cameraParams.IntrinsicMatrix.';
cam.k1 = cameraParams.RadialDistortion(1);
cam.k2 = cameraParams.RadialDistortion(2);
%cam.k3 = cameraParams.RadialDistortion(3);
cam.p1 = cameraParams.TangentialDistortion(1);
cam.p2 = cameraParams.TangentialDistortion(2);
cam.p3 = 0;


% intrinsic camera calibration K parameters for multi-level pyramid
K_pyramid = zeros(3,3,maxPyramidLevel);
K_pyramid(:,:,1) = cam.K;

% perform pyramid reduction with average values
for k = 2:maxPyramidLevel
    K_pyramid(:,:,k) = downsampleKmatrix(K_pyramid(:,:,k-1));
end
cam.K_pyramid = K_pyramid;


% image size parameters
cam.nRows = 480;
cam.nCols = 640;


% scale factor to recover depth image
cam.scaleFactor = 1000;


end