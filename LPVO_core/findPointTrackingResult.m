function [ipRelations] = findPointTrackingResult(imagePointsLast, imagePointsCur, depthImage, imgIdx, cam, optsLPVO)

imagePointsLastNum = size(imagePointsLast, 2);
imagePointsCurNum = size(imagePointsCur, 2);


% find point feature correspondence
ipRelations = cell(0);
for i=1:imagePointsLastNum
    
    % find same index
    ipFound = false;
    for j=1:imagePointsCurNum
        if (imagePointsCur{j}.ind == imagePointsLast{i}.ind)
            ipFound = true;
        end
        if (imagePointsCur{j}.ind >= imagePointsLast{i}.ind)
            break;
        end
    end
    
    if (ipFound)
        ipr.x = imagePointsLast{i}.u;
        ipr.y = imagePointsLast{i}.v;
        ipr.z = imagePointsCur{j}.u;
        ipr.h = imagePointsCur{j}.v;
        
        ipr.age = imagePointsCur{j}.age;
        if (imgIdx >= 10 && ipr.age == 1)
            continue;
        end
        
        clear uTemp vTemp
        uTemp = cam.K(1,1) * ipr.x + cam.K(1,3);
        vTemp = cam.K(2,2) * ipr.y + cam.K(2,3);
        pointDepth = depthImage(round(vTemp),round(uTemp));
        
        if (pointDepth >= optsLPVO.minimumDepth && pointDepth <= optsLPVO.maximumDepth)
            ipr.s = pointDepth;
            ipr.v = 1;
        else
            ipr.s = 0;
            ipr.v = 0;
        end
        
        ipRelations{end+1} = ipr;
    end
end


% if there is no point matching with age
if (size(ipRelations, 2) <= 3)
    ipRelations = cell(0);
    for i=1:imagePointsLastNum
        
        % find same index
        ipFound = false;
        for j=1:imagePointsCurNum
            if (imagePointsCur{j}.ind == imagePointsLast{i}.ind)
                ipFound = true;
            end
            if (imagePointsCur{j}.ind >= imagePointsLast{i}.ind)
                break;
            end
        end
        
        if (ipFound)
            ipr.x = imagePointsLast{i}.u;
            ipr.y = imagePointsLast{i}.v;
            ipr.z = imagePointsCur{j}.u;
            ipr.h = imagePointsCur{j}.v;
            
            clear uTemp vTemp
            uTemp = cam.K(1,1) * ipr.x + cam.K(1,3);
            vTemp = cam.K(2,2) * ipr.y + cam.K(2,3);
            pointDepth = depthImage(round(vTemp),round(uTemp));
            
            if (pointDepth >= optsLPVO.minimumDepth && pointDepth <= optsLPVO.maximumDepth)
                ipr.s = pointDepth;
                ipr.v = 1;
            else
                ipr.s = 0;
                ipr.v = 0;
            end
            
            ipRelations{end+1} = ipr;
        end
    end
end


end



