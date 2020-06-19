function [R_cM, isTracked] = checkManhattanFrameTracking(R_cM, surfaceNormal, vanishingDirection, optsLPVO)

% pre-defined variables
halfApexAngleNV = optsLPVO.halfApexAngleNV;
halfApexAngleVD = optsLPVO.halfApexAngleVD;
minSampleRatio = optsLPVO.minSampleRatio;
planeDensityThreshold = optsLPVO.planeDensityThreshold;
c = optsLPVO.c;


% determine minimum number of normal vector
numInConeNV = computeNumInXYZCone(R_cM, surfaceNormal, halfApexAngleNV);
numInConeVD = computeNumInXYZCone(R_cM, vanishingDirection, halfApexAngleVD);
numInCone = numInConeNV + numInConeVD;
numInCone = sort(numInCone);

numUnitVector = size(surfaceNormal, 2) + size(vanishingDirection, 2);
minSampleNum = round(numUnitVector*minSampleRatio);
if (numInCone(2) < minSampleNum)
    minSampleNum = (numInCone(1) + numInCone(2)) * 0.5;
end


% project to each Manhattan frame axis
planeDensity = zeros(1, 3);
for a = 1:3
    R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
    m_j_NV = projectManhattanFrame(R_Mc, surfaceNormal, halfApexAngleNV);
    m_j_VD = projectManhattanFrame(R_Mc, vanishingDirection, halfApexAngleVD);
    m_j = [m_j_NV, m_j_VD];
    
    if (size(m_j, 2) >= minSampleNum)
        [~, density] = MeanShift_mex(m_j, c);
        planeDensity(a) = density;
    end
end


% check validity of MF tracking
validNumPlane = sum(planeDensity >= planeDensityThreshold);
if (validNumPlane >= 2)
    isTracked = true;
else
    R_cM = eye(3);
    isTracked = false;
end


end

