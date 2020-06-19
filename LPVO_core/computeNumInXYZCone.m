function [numInXYZCone] = computeNumInXYZCone(R_cM, unitVector, halfApexAngle)


numInXYZCone = zeros(1, 3);
for a = 1:3
    % projection on each axis (x, y, z)
    R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
    n_j = R_Mc * unitVector;
    
    % check within half apex angle
    lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
    index = find(lambda <= sin(halfApexAngle));
    numInXYZCone(a) = size(index, 2);
end


end

