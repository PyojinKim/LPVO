function plot_clustered_line(R_cM, vanishingDirectionVector, linePairIdx, lines, optsLPVO)

% assign parameters
halfApexAngleVD = optsLPVO.halfApexAngleVD;


%% project normal vectors to each Manhattan frame axis

numVDs = size(vanishingDirectionVector, 2);
manhattanAxisIdx = ones(1, numVDs) * -1000;
for a = 1:3
    % projection on each axis (x, y, z)
    R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
    n_j = R_Mc * vanishingDirectionVector;
    
    % check within half apex angle
    lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
    index = find(lambda <= sin(halfApexAngleVD));
    manhattanAxisIdx(:,index) = a;
end


%% plot clustered line results

xAxisIdx = (manhattanAxisIdx == 1);
yAxisIdx = (manhattanAxisIdx == 2);
zAxisIdx = (manhattanAxisIdx == 3);

xAxisLineIdx = linePairIdx(:,xAxisIdx);
xAxisLineIdx = unique(xAxisLineIdx(:));
yAxisLineIdx = linePairIdx(:,yAxisIdx);
yAxisLineIdx = unique(yAxisLineIdx(:));
zAxisLineIdx = linePairIdx(:,zAxisIdx);
zAxisLineIdx = unique(zAxisLineIdx(:));


% plot clustered lines on the image plane
for k = 1:size(xAxisLineIdx,1)
    lineIdx = xAxisLineIdx(k);
    linePixelPts = lines(lineIdx,1:4) / (2^(optsLPVO.imagePyramidLevel-1));
    plot([linePixelPts(1),linePixelPts(3)], [linePixelPts(2),linePixelPts(4)], 'r', 'LineWidth', 2);
end
for k = 1:size(yAxisLineIdx,1)
    lineIdx = yAxisLineIdx(k);
    linePixelPts = lines(lineIdx,1:4) / (2^(optsLPVO.imagePyramidLevel-1));
    plot([linePixelPts(1),linePixelPts(3)], [linePixelPts(2),linePixelPts(4)], 'g', 'LineWidth', 2);
end
for k = 1:size(zAxisLineIdx,1)
    lineIdx = zAxisLineIdx(k);
    linePixelPts = lines(lineIdx,1:4) / (2^(optsLPVO.imagePyramidLevel-1));
    plot([linePixelPts(1),linePixelPts(3)], [linePixelPts(2),linePixelPts(4)], 'b', 'LineWidth', 2);
end


end