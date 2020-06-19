function [vanishingDirectionVector, linePairIdx, lines] = estimateVanishingDirectionGreatcircle(imageCurForLine, cam, optsLPVO)

% assign current parameters
lineDetector = optsLPVO.lineDetector;
lineLength = optsLPVO.lineLength;
K = cam.K;
Kinv = inv(K);


% line detection
if (strcmp(lineDetector,'lsd'))
    [lines, ~] = lsdf(double(imageCurForLine), (lineLength^2));
elseif (strcmp(lineDetector,'gpa'))
    [lines, ~] = gpa(imageCurForLine);
end


% great circle normal vector
numLines = size(lines, 1);
greatcircleNormal = zeros(numLines, 3);
for k = 1:numLines
    
    % line information
    linedata = lines(k,1:4);
    
    % normalized image plane
    ptEnd1_p_d = [linedata(1:2), 1].';
    ptEnd2_p_d = [linedata(3:4), 1].';
    ptEnd1_n_d = Kinv * ptEnd1_p_d;
    ptEnd2_n_d = Kinv * ptEnd2_p_d;
    ptEnd1_n_u = [undistortPts_normal_mex(ptEnd1_n_d(1:2), cam); 1];
    ptEnd2_n_u = [undistortPts_normal_mex(ptEnd2_n_d(1:2), cam); 1];
    
    % normal vector of great circle
    circleNormal = cross(ptEnd1_n_u.', ptEnd2_n_u.');
    circleNormal = circleNormal / norm(circleNormal);
    
    
    % save the result
    greatcircleNormal(k,:) = circleNormal;
end


% vanishing direction vector of every line pair
linePairIdx = combnk(1:numLines, 2).';
numAllVDs = size(linePairIdx, 2);
vanishingDirectionVector = zeros(3, numAllVDs);
for k = 1:numAllVDs
    
    % line pair index
    firstLineIdx = linePairIdx(1,k);
    secondLineIdx = linePairIdx(2,k);
    
    % candidate vanishing direction
    candidateVD = cross(greatcircleNormal(firstLineIdx,:).', greatcircleNormal(secondLineIdx,:).');
    candidateVD = candidateVD / norm(candidateVD);
    
    
    % save the result
    vanishingDirectionVector(:,k) = candidateVD;
end


end

