function plot_tracked_point(ipRelations, cam, optsLPVO)


% plot tracked points on the image plane
ipRelationsNum = size(ipRelations, 2);
for k = 1:ipRelationsNum
    x = (cam.K(1,1) * ipRelations{k}.z + cam.K(1,3)) / (2^(optsLPVO.imagePyramidLevel-1));
    y = (cam.K(2,2) * ipRelations{k}.h + cam.K(2,3)) / (2^(optsLPVO.imagePyramidLevel-1));
    if (abs(ipRelations{k}.v) < 0.5)           % no depth point
        plot(x, y, 'rs');
    elseif (abs(ipRelations{k}.v - 1) < 0.5) % depth point from RGB-D
        plot(x, y, 'gs');
    end
end


end