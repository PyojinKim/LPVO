function plot_sNV_unit_sphere(radius, R_cM, surfaceNormalVector, optsLAPO)

% assign parameters
halfApexAngleNV = optsLAPO.halfApexAngleNV;


%% project normal vectors to each Manhattan frame axis

numNormalVector = size(surfaceNormalVector, 2);
surfaceAxisIndex = ones(1, numNormalVector) * -1000;
for a = 1:3
    % projection on each axis (x, y, z)
    R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
    n_j = R_Mc * surfaceNormalVector;
    
    % check within half apex angle
    lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
    index = find(lambda <= sin(halfApexAngleNV));
    surfaceAxisIndex(:, index) = a;
end


%% plot sphere compass results

xAxisIndex = (surfaceAxisIndex == 1);
yAxisIndex = (surfaceAxisIndex == 2);
zAxisIndex = (surfaceAxisIndex == 3);
otherIndex = (surfaceAxisIndex == -1000);

xAxisNV = surfaceNormalVector(:,xAxisIndex) * radius;
yAxisNV = surfaceNormalVector(:,yAxisIndex) * radius;
zAxisNV = surfaceNormalVector(:,zAxisIndex) * radius;
otherNV = surfaceNormalVector(:,otherIndex) * radius;


% plot sphere compass results with normal vector points
drawInterval = 5;
plot3(xAxisNV(1,1:drawInterval:end), xAxisNV(2,1:drawInterval:end), xAxisNV(3,1:drawInterval:end), 'r.');
plot3(yAxisNV(1,1:drawInterval:end), yAxisNV(2,1:drawInterval:end), yAxisNV(3,1:drawInterval:end), 'g.');
plot3(zAxisNV(1,1:drawInterval:end), zAxisNV(2,1:drawInterval:end), zAxisNV(3,1:drawInterval:end), 'b.');
plot3(otherNV(1,1:drawInterval:end), otherNV(2,1:drawInterval:end), otherNV(3,1:drawInterval:end), 'k.');


end