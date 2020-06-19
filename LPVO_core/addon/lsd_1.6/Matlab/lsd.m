function out=lsd(image,option,normalize_image)
% LSD  returns an array containing datas concerning line segments detected
% in an image
%   out = LSD(A) returns line segments belonging to A
%
% INPUT :
% =======
%
% A must be a double matrix.
% To load it from an image on your computer -which has to be a .pgm file- use the following command :
% A=double(imread('adress/of/file.pgm'))
%
% OPTIONS :
% =========
%
% Even if default options are optimized, you can change them as you wish.
% Just use a structure containing them or the ones you need.
% For example, if you want a scale equals to one and a sigma_coef equals to 0.7, type the following command :
%   option.scale=1;
%   option.sigma_coef=0.8;
%   out = LSD(A,option)
%
% You can name your structure as you want. However its components have to respect some rules :
%
% SCALE :
%   name as a component of the optional structure : scale
%   type : double
%   default value : 0.8
%   minimum value : 0.0
%   maximum value : -
%   Scale image by Gaussian filter before processing.
%
% SIGMA COEFFICIENT :
%   name as a component of the optional structure : sigma_coef
%   type : double
%   default value : 0.6
%   minimum value : 0.0
%   maximum value : -
%   Sigma for Gaussian filter is computed as sigma_coef/scale.
%
% QUANTIZATION :
%   name as a component of the optional structure : quant
%   type : double
%   default value : 2.0
%   minimum value : 0.0
%   maximum value : -
%   Bound to quantization error on the gradient norm.
%
% ANGLE TOLERANCE :
%   name as a component of the optional structure : ang_th
%   type : double
%   default value : 22.5
%   minimum value : 0.0
%   maximum value : 180.0
%   Gradient angle tolerance in degrees.
%
% DETECTION THRESHOLD :
%   name as a component of the optional structure : log_eps
%   type : double
%   default value : 0.0
%   minimum value : -
%   maximum value : -
%   Detection threshold, -log10(max. NFA)
%
% DENSITY :
%   name as a component of the optional structure : density_th
%   type : double
%   default value : 0.7
%   minimum value : 0.0
%   maximum value : 1.0
%   Minimal density of region points in a rectangle to be accepted.
%
% NUMBER OF BINS :
%   name as a component of the optional structure : n_bins
%   type : integer
%   default value : 1024
%   minimum value : 1
%   maximum value : -
%   Number of bins in 'ordering' of gradient modulus.
%
%   RETURNED VALUE :
%   ================
%
%   The returned value is a matrix.
%   Each row represents a segment.
%   The first column contains x1
%   The second column contains y1
%   The third column contains x2
%   The fourth column contains y2
%   The three last columns contain other informations available at the following adress :
%   http://www.ipol.im/pub/art/2012/gjmr-lsd/
%
% NOTE: think to modify the mex because it does not handle colors. Images
% have to be renormalized

if(~ismatrix(image))
    image=rgb2gray(image);
end

if(nargin<3)
    normalize_image=1;
end

if(normalize_image)
    image=floor(256*image/max(max(image)));
end

if (~exist('option', 'var'))
    out=MEX_lsd_cmd(image);
else
    out=MEX_lsd_cmd(image,option);
end