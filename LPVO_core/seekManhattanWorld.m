function [MF_can, FindMF] = seekManhattanWorld(surfaceNormalVector, optsLPVO)

% pre-defined variables
numInitialization = optsLPVO.numInitialization;
iterNum = optsLPVO.iterNum;
convergeAngle = optsLPVO.convergeAngle;
halfApexAngle = deg2rad(15);
c = optsLPVO.c;


%% seek Manhattan world frame

MF = {};
MF_can = {};
minSampleNum = round(size(surfaceNormalVector, 2)/20);

for k = 1:numInitialization
    
    % initial model parameters
    R_cM = GetRandomRotation();
    R_cM_update = R_cM;
    R_cM_Rec = eye(3);
    
    numDirectionFound = 0;
    directionFound = [];
    validMF = 0;
    
    
    % do mean shift iteration
    for iterCount = 1:iterNum
        
        R_cM = R_cM_update;
        numDirectionFound = 0;
        directionFound = [];
        validMF = 0;
        
        % project to each Manhattan frame axis
        for a = 1:3
            R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
            n_j = R_Mc * surfaceNormalVector;
            
            lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
            index = find(lambda < sin(halfApexAngle));
            n_j_inlier = n_j(:,index);
            tan_alfa = lambda(index)./abs(n_j(3,index));
            alfa = asin(lambda(index));
            m_j = [alfa./tan_alfa.*n_j_inlier(1,:)./n_j_inlier(3,:);
                alfa./tan_alfa.*n_j_inlier(2,:)./n_j_inlier(3,:)];
            
            select = ~isnan(m_j);
            select2 = select(1,:).*select(2,:);
            select3 = find(select2 == 1);
            m_j = m_j(:,select3);
            
            if (size(m_j, 2) >= minSampleNum)
                
                % compute mean shift
                s_j = MeanShift(m_j, c);
                
                % compute the Ma
                alfa = norm(s_j);
                ma_p = tan(alfa)/alfa * s_j;
                R_cM_Rec(:,a) = R_Mc.' * [ma_p; 1];
                R_cM_Rec(:,a) = R_cM_Rec(:,a) / norm(R_cM_Rec(:,a));
                numDirectionFound = numDirectionFound + 1;
                directionFound = [directionFound a];
            end
        end
        
        % handle numDirectionFound is not three
        if (numDirectionFound < 2)
            numDirectionFound = 0;
            directionFound = [];
            validMF = 0;
            break;
        elseif (numDirectionFound == 2)
            v1 = R_cM_Rec(:,directionFound(1));
            v2 = R_cM_Rec(:,directionFound(2));
            v3 = cross(v1,v2);
            R_cM_Rec(:,6-(directionFound(1)+directionFound(2))) = v3;
            if (abs(det(R_cM_Rec)+1) < 0.5)
                R_cM_Rec(:,6-(directionFound(1)+directionFound(2))) = -v3;
            end
        end
        
        % maintain orthogonality on SO(3)
        [U,~,V] = svd(R_cM_Rec);
        R_cM_update = U * V';
        validMF = 1;
        
        % check convergence
        if (acos((trace(R_cM.' * R_cM_update) - 1)/2) < convergeAngle)
            break;
        end
    end
    
    
    % valid Manhattan world frame
    if (validMF == 1)
        MF = [MF, {R_cM_update}];
        MF = MF(~cellfun('isempty',MF));
    end
end


% check whether we find at least one MF
if (numel(MF) == 0)
    MF_can = [];
    FindMF = 0;
    return;
end


% find the unique canonical form
MF_can = RemoveRedundancyMF2(MF);
FindMF = 1;


end



function [R] = GetRandomRotation()

q = rand(1,4);
R = quat2dcm( q./norm(q) );

end



function [MF_can] = RemoveRedundancyMF2(MF)
MF_can = {};
% 24 possbile
R_poss = cell(1,24);
cellIndex = 1;

for row1=1:3
    for row2=1:3
        
        if (row2 ~= row1)
            for row3=1:3
                
                if (row3 ~= row1 && row3 ~= row2)
                    
                    R = zeros(3,3);
                    
                    R(1,row1) = 1.0; R(2,row2) = 1.0; R(3,row3) = 1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                    R(1,row1) = 1.0; R(2,row2) = 1.0; R(3,row3) = -1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                    R(1,row1) = 1.0; R(2,row2) = -1.0; R(3,row3) = 1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                    R(1,row1) = 1.0; R(2,row2) = -1.0; R(3,row3) = -1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                    
                    R(1,row1) = -1.0; R(2,row2) = 1.0; R(3,row3) = 1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                    R(1,row1) = -1.0; R(2,row2) = 1.0; R(3,row3) = -1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                    R(1,row1) = -1.0; R(2,row2) = -1.0; R(3,row3) = 1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                    R(1,row1) = -1.0; R(2,row2) = -1.0; R(3,row3) = -1.0;
                    if det(R) > 0
                        R_poss{cellIndex} = R;
                        cellIndex = cellIndex + 1;
                    end
                    
                end
                
            end
        end
    end
end

% remove redundancy and convert to canonical coordinate
numMF = numel(MF);
for i = 1:numMF
    minTheta = 100;
    minID = -1;
    for j = 1:24
        M_star = MF{i}*R_poss{j};
        errTheta = acos((trace(M_star)-1)/2);
        if (errTheta < minTheta)
            minTheta = errTheta;
            minID = j;
        end
    end
    MF_can = [MF_can, {MF{i}*R_poss{minID}}];
end

% clean the Null cell entries in the cell array
MF_can = MF_can(~cellfun('isempty',MF_can));

end





