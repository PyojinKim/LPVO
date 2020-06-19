function plot_display_text(R_cM, sNV, vDV, ipRelations, isTracked, optsLPVO)


% number of sNV and vDV
numNV = computeNumInXYZCone(R_cM, sNV, optsLPVO.halfApexAngleNV);
numVD = computeNumInXYZCone(R_cM, vDV, optsLPVO.halfApexAngleVD);


% number of tracked points
numPoints = size(ipRelations, 2);


% display current status
text(7, 10, sprintf('sNV: %05d, %05d, %05d', numNV(1), numNV(2), numNV(3)), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
text(7, 22, sprintf('vDV: %04d, %04d, %04d', numVD(1), numVD(2), numVD(3)), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
text(7, 34, sprintf('points: %04d', numPoints), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
text(7, 46, sprintf('isTracked: %d', isTracked), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');


end

