function plot_vDV_unit_sphere(radius, R_cM, vanishingDirectionVector, optsLPVO)

% assign parameters
halfApexAngleVD = optsLPVO.halfApexAngleVD;


%% project vanishing direction vectors to each Manhattan frame axis

numVDs = size(vanishingDirectionVector, 2);
manhattanAxisIdx = ones(1, numVDs) * -1000;
for a = 1:3
    % projection on each axis (x, y, z)
    R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
    n_j = R_Mc * vanishingDirectionVector;
    
    % check within half apex angle
    lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
    index = find(lambda <= sin(halfApexAngleVD));
    manhattanAxisIdx(:, index) = a;
end


%% plot sphere compass results

xAxisIndex = (manhattanAxisIdx == 1);
yAxisIndex = (manhattanAxisIdx == 2);
zAxisIndex = (manhattanAxisIdx == 3);
otherIndex = (manhattanAxisIdx == -1000);

xAxisDV = vanishingDirectionVector(:,xAxisIndex) * radius;
yAxisDV = vanishingDirectionVector(:,yAxisIndex) * radius;
zAxisDV = vanishingDirectionVector(:,zAxisIndex) * radius;
otherDV = vanishingDirectionVector(:,otherIndex) * radius;


% plot sphere compass results with direction vector points
drawInterval = 1;
plot3(xAxisDV(1,1:drawInterval:end), xAxisDV(2,1:drawInterval:end), xAxisDV(3,1:drawInterval:end), 'r.');
plot3(yAxisDV(1,1:drawInterval:end), yAxisDV(2,1:drawInterval:end), yAxisDV(3,1:drawInterval:end), 'g.');
plot3(zAxisDV(1,1:drawInterval:end), zAxisDV(2,1:drawInterval:end), zAxisDV(3,1:drawInterval:end), 'b.');
plot3(otherDV(1,1:drawInterval:end), otherDV(2,1:drawInterval:end), otherDV(3,1:drawInterval:end), 'k.');


end