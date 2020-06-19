function [R_cM_update, isTracked] = trackManhattanWorld_LPVO(R_cM, surfaceNormal, vanishingDirection, optsLPVO)

% pre-defined variables
iterNum = optsLPVO.iterNum;
convergeAngle = optsLPVO.convergeAngle;
halfApexAngleNV = optsLPVO.halfApexAngleNV;
halfApexAngleVD = optsLPVO.halfApexAngleVD;
minSampleRatio = optsLPVO.minSampleRatio;
c = optsLPVO.c;


%% track Manhattan world frame

% initial model parameters
R_cM_update = R_cM;
isTracked = 0;


% do mean shift iteration
for iterCount = 1:iterNum
    
    R_cM = R_cM_update;
    numDirectionFound = 0;
    directionFound = [];
    planeDensity = zeros(1,3) + 0.00001;
    
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
    for a = 1:3
        R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
        m_j_NV = projectManhattanFrame(R_Mc, surfaceNormal, halfApexAngleNV);
        m_j_VD = projectManhattanFrame(R_Mc, vanishingDirection, halfApexAngleVD);
        m_j = [m_j_NV, m_j_VD];
        
        if (size(m_j, 2) >= minSampleNum)
            
            % compute mean shift
            [s_j, planeDensity(a)] = MeanShift_mex(m_j, c);
            
            % compute the Ma
            alfa = norm(s_j);
            ma_p = tan(alfa)/alfa * s_j;
            R_cM_update(:,a) = R_Mc.' * [ma_p; 1];
            R_cM_update(:,a) = R_cM_update(:,a) / norm(R_cM_update(:,a));
            numDirectionFound = numDirectionFound + 1;
            directionFound = [directionFound a];
        end
    end
    
    % handle numDirectionFound is not three
    if (numDirectionFound < 2)
        R_cM_update = R_cM;
        isTracked = 0;
        return;
    elseif (numDirectionFound == 2)
        v1 = R_cM_update(:,directionFound(1));
        v2 = R_cM_update(:,directionFound(2));
        v3 = cross(v1,v2);
        R_cM_update(:,6-(directionFound(1)+directionFound(2))) = v3;
        if (abs(det(R_cM_update)+1) < 0.5)
            R_cM_update(:,6-(directionFound(1)+directionFound(2))) = -v3;
        end
    end
    
    % maintain orthogonality on SO(3)
    [U,~,V] = svd([R_cM_update(:,1)*planeDensity(1), R_cM_update(:,2)*planeDensity(2), R_cM_update(:,3)*planeDensity(3)]);
    R_cM_update = U * V';
    
    % check convergence
    if (acos((trace(R_cM.' * R_cM_update) - 1)/2) < convergeAngle)
        break;
    end
end

isTracked = 1;


end


