%--------------------------------------------------------------------------
% Real-time Camera Orientation and Vanishing Point Estimation (ROVE)
%     Copyright (C) 2015 Jeong-Kyun Lee and Kuk-Jin Yoon
%
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     any later version.
%
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
%
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.
%--------------------------------------------------------------------------
% If you use this code, please cite:
%   Jeong-Kyun Lee and Kuk-Jin Yoon, 'Real-Time Joint Estimation of Camera
%   Orientation and Vanishing Points', The IEEE Conference on Computer
%   Vision and Pattern Recognition (CVPR), Boston, June, 2015.
%--------------------------------------------------------------------------
function [ lines n ] = lsdf( img, thr )
% img : uint8 image with 0 ~ 255
% lines : < n x 7 >, x1,y1,x2,y2,width,p,-log10(NFA)
% n : the number of lines

lines = lsd(double(img));
[n c] = size(lines);
s = zeros(n,c);
s(1:end,1:4) = 1;
lines = lines + s;

dx = lines(:,1) - lines(:,3);
dy = lines(:,2) - lines(:,4);
iflag = dx.^2 + dy.^2 > thr;

lines = lines(iflag, :);
n = size(lines,1);

end
