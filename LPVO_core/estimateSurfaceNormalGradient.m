function [surfaceNormalVector, surfacePixelPoint] = estimateSurfaceNormalGradient(imageCur, depthCur, cam, optsLPVO)

% assign current image pyramid
L = optsLPVO.imagePyramidLevel;
K = cam.K_pyramid(:,:,L);
cellsize = optsLPVO.cellsize;
minDepth = optsLPVO.minimumDepth;
maxDepth = optsLPVO.maximumDepth;


%% depth image smoothing

% remove extreme depth values
rawDepthCur = depthCur;
rawDepthCur(rawDepthCur < minDepth | rawDepthCur > maxDepth) = -1000;
rawDepthCur(rawDepthCur == -1000) = 0;

% guided filter (not use)
%rawImageCur = imageCur;
%filterDepthCur = imguidedfilter(rawDepthCur, rawImageCur);
%filterDepthCur(filterDepthCur < 0.5 | filterDepthCur > 5) = -1000;
%filterDepthCur(filterDepthCur == -1000) = 0;

% final filtered depth map
depthMapFiltered = rawDepthCur;


%% normal vectors and pixel points of each cell

% assign current camera parameters
depthMap = depthMapFiltered;
imageHeight = size(depthMap, 1);
imageWidth = size(depthMap, 2);


% recover 3D coordinates
vertexMap = zeros(imageHeight, imageWidth, 3);
[u, v] = meshgrid(((1:imageWidth)-1), ((1:imageHeight)-1));  % for zero-based index like librealsense
vertexMap(:,:,1) = (u-K(1,3)).*depthMap/K(1,1);
vertexMap(:,:,2) = (v-K(2,3)).*depthMap/K(2,2);
vertexMap(:,:,3) = depthMap;


% tangential vectors of each point
tangeMask = zeros(imageHeight, imageWidth, 1);
uTangeMap = zeros(imageHeight, imageWidth, 3);
vTangeMap = zeros(imageHeight, imageWidth, 3);
for v = 2:(imageHeight-1)
    for u = 2:(imageWidth-1)
        
        % x and y index
        uNext = u+1;
        uPrev = u-1;
        vNext = v+1;
        vPrev = v-1;
        
        % test surrounding depth values
        testCenter = vertexMap(v,u,3);
        testLeft = vertexMap(v,uPrev,3);
        testRight = vertexMap(v,uNext,3);
        testUp = vertexMap(vPrev,u,3);
        testDown = vertexMap(vNext,u,3);
        testValues = [testCenter; testLeft; testRight; testUp; testDown];
        
        validIdx = (testValues >= minDepth) & (testValues <= maxDepth);
        if (sum(validIdx) == 5)
            tangeMask(v,u) = 1;
            
            % tangential vectors
            uTangeMap(v,u,:) = vertexMap(v,uNext,:) - vertexMap(v,uPrev,:);
            vTangeMap(v,u,:) = vertexMap(vNext,u,:) - vertexMap(vPrev,u,:);
        end
    end
end


% integral images of tangential vectors
uTangeIntegral = zeros(size(uTangeMap));
vTangeIntegral = zeros(size(vTangeMap));
for k = 1:3
    uTangeTemp = integralImage(uTangeMap(:,:,k));
    uTangeIntegral(:,:,k) = uTangeTemp(2:end,2:end);
    
    vTangeTemp = integralImage(vTangeMap(:,:,k));
    vTangeIntegral(:,:,k) = vTangeTemp(2:end,2:end);
end
tangeMaskIntegralTemp = integralImage(tangeMask);
tangeMaskIntegral = tangeMaskIntegralTemp(2:end,2:end);


% surface normal vectors with cross product
surfaceNormalVector = zeros(3, imageHeight*imageWidth);
surfacePixelPoint = zeros(2, imageHeight*imageWidth);
cellCount = 0;
for v = (cellsize+1):imageHeight
    for u = (cellsize+1):imageWidth
        
        if (tangeMask(v,u) == 1)
            
            % average tangential vectors
            numPts = tangeMaskIntegral(v,u) - tangeMaskIntegral(v-cellsize,u) - tangeMaskIntegral(v,u-cellsize) + tangeMaskIntegral(v-cellsize,u-cellsize);
            uVector = [uTangeIntegral(v,u,1) - uTangeIntegral(v-cellsize,u,1) - uTangeIntegral(v,u-cellsize,1) + uTangeIntegral(v-cellsize,u-cellsize,1);
                uTangeIntegral(v,u,2) - uTangeIntegral(v-cellsize,u,2) - uTangeIntegral(v,u-cellsize,2) + uTangeIntegral(v-cellsize,u-cellsize,2);
                uTangeIntegral(v,u,3) - uTangeIntegral(v-cellsize,u,3) - uTangeIntegral(v,u-cellsize,3) + uTangeIntegral(v-cellsize,u-cellsize,3)] ./ numPts;
            vVector = [vTangeIntegral(v,u,1) - vTangeIntegral(v-cellsize,u,1) - vTangeIntegral(v,u-cellsize,1) + vTangeIntegral(v-cellsize,u-cellsize,1);
                vTangeIntegral(v,u,2) - vTangeIntegral(v-cellsize,u,2) - vTangeIntegral(v,u-cellsize,2) + vTangeIntegral(v-cellsize,u-cellsize,2);
                vTangeIntegral(v,u,3) - vTangeIntegral(v-cellsize,u,3) - vTangeIntegral(v,u-cellsize,3) + vTangeIntegral(v-cellsize,u-cellsize,3)] ./ numPts;
            
            % local surface normal vector
            cellCount = cellCount + 1;
            
            normalVector = zeros(3,1);
            normalVector(1) = vVector(2) * uVector(3) - vVector(3) * uVector(2);
            normalVector(2) = vVector(3) * uVector(1) - vVector(1) * uVector(3);
            normalVector(3) = vVector(1) * uVector(2) - vVector(2) * uVector(1);
            surfaceNormalVector(:, cellCount) = normalVector;
            surfacePixelPoint(:, cellCount) = [u; v];
        end
    end
end
surfaceNormalVector(:, (cellCount+1):end) = [];
surfacePixelPoint(:, (cellCount+1):end) = [];


% normalize surface normal vectors
numNormalVector = size(surfaceNormalVector, 2);
for k = 1:numNormalVector
    surfaceNormalVector(:,k) = surfaceNormalVector(:,k) / norm(surfaceNormalVector(:,k));
end


end



% surface3DpointCloud = zeros(3, imageHeight*imageWidth);
% surface3DpointCloud(:, cellCount) = squeeze(vertexMap(v,u,:));
% surface3DpointCloud(:, (cellCount+1):end) = [];



% % surface normal vectors with cross product
% surfaceNormalVector = zeros(3, imageHeight*imageWidth);
% surfacePixelPoint = cell(1, imageHeight*imageWidth);
% cellCount = 0;
% for v = cellsize:imageHeight
%     for u = cellsize:imageWidth
%
%         if (tangeMask(v,u) == 1)
%
%             % u and v index
%             uStart = u - cellsize + 1;
%             uEnd = u;
%             vStart = v - cellsize + 1;
%             vEnd = v;
%
%             % average tangential vectors
%             numPts = sum(sum(tangeMask(vStart:vEnd,uStart:uEnd)));
%             uVector = sum(sum(uTangeMap(vStart:vEnd,uStart:uEnd,:))) ./ numPts;
%             vVector = sum(sum(vTangeMap(vStart:vEnd,uStart:uEnd,:))) ./ numPts;
%
%             % local surface normal vector
%             cellCount = cellCount + 1;
%
%             normalVector(1) = vVector(2) * uVector(3) - vVector(3) * uVector(2);
%             normalVector(2) = vVector(3) * uVector(1) - vVector(1) * uVector(3);
%             normalVector(3) = vVector(1) * uVector(2) - vVector(2) * uVector(1);
%             surfaceNormalVector(:, cellCount) = normalVector.';
%             surfacePixelPoint{cellCount} = [u; v];
%         end
%     end
% end
% surfaceNormalVector(:, (cellCount+1):end) = [];
% surfacePixelPoint((cellCount+1):end) = [];
%
%
% % normalize surface normal vectors
% numNormalVector = size(surfaceNormalVector, 2);
% for k = 1:numNormalVector
%     surfaceNormalVector(:,k) = surfaceNormalVector(:,k) / norm(surfaceNormalVector(:,k));
% end