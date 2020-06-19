clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;

addpath('addon/lsd_1.6');
addpath('addon/lsd_1.6/Matlab');


%% basic setup for LPVO

% choose the experiment case
% ICSL RGBD dataset (1~XX)
expCase = 1;

% are figures drawn?
% 1 : yes, draw figures to see current status
% 0 : no, just run LPVO
toVisualize = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run LPVO
toSave = 1;


setupParams_ICSL_RGBD;


% load ICSL RGBD dataset data
ICSLRGBDdataset = rawICSLRGBDdataset_load(datasetPath, imInit, M);


% camera calibration parameters
optsLPVO = load_param_LPVO;
cam = initialize_cam_ICSL_RGBD(cameraType, optsLPVO.imagePyramidLevel);


%% main LPVO part

% 1. feature tracking pre-defined variables for LPVO
systemInited_ft = false;

imageHeight = 480;
imageWidth = 640;
imagePixelNum = imageHeight * imageWidth;
imgSize = [imageWidth imageHeight];

imageCur = uint8(zeros(imgSize(2), imgSize(1)));
imageLast = uint8(zeros(imgSize(2), imgSize(1)));

showCount = 0;
showSkipNum = 2;
showDSRate = 2;
showSize = [imageWidth/showDSRate imageHeight/showDSRate];

imageShow = uint8(zeros(showSize(2), showSize(1)));
harrisLast = uint8(zeros(showSize(2), showSize(1)));

maxFeatureNumPerSubregion = 6;
xSubregionNum = 12;
ySubregionNum = 8;
totalSubregionNum = xSubregionNum * ySubregionNum;
MAXFEATURENUM = maxFeatureNumPerSubregion * totalSubregionNum;

xBoundary = 20;
yBoundary = 20;
subregionWidth = (imageWidth - 2 * xBoundary) / xSubregionNum;
subregionHeight = (imageHeight - 2 * yBoundary) / ySubregionNum;

maxTrackDis = 100;
winSize = 15;
ZNCCwinSize = 15;

featuresCur = cell(1, 2 * MAXFEATURENUM);
featuresLast = cell(1, 2 * MAXFEATURENUM);
featuresLastPatch = cell(1, 2 * MAXFEATURENUM);
featuresFound = zeros(1, 2 * MAXFEATURENUM);
featuresError = zeros(1, 2 * MAXFEATURENUM);

featuresIndFromStart = 0;
featuresInd = zeros(1, 2 * MAXFEATURENUM);
featuresAge = -1000*ones(1, 2 * MAXFEATURENUM);

totalFeatureNum = 0;
subregionFeatureNum = zeros(1, totalSubregionNum);

imagePointsCur = cell(0);
imagePointsLast = cell(0);


% 2. Manhattan frame tracking for LPVO
systemInited_MF = false;

R_gc1 = eye(3);
R_gc_LPVO = zeros(3,3,M);
R_gc_LPVO(:,:,1) = R_gc1;


% 3. frame to frame motion estimation pre-defined variables for LPVO
systemInited_VO = false;

T_gc_current = eye(4);
T_gc_LPVO = cell(1, M);
T_gc_LPVO{1} = T_gc_current;


% 4. make figures to visualize current status
if (toVisualize)
    % create figure
    h = figure(10);
    set(h,'Color',[1 1 1]);
    set(h,'Units','pixels','Position',[150 400 1700 600]);
    ha1 = axes('Position',[0.025,0.1 , 0.3,0.8]);
    axis off;
    ha2 = axes('Position',[0.350,0.1 , 0.3,0.8]);
    axis off;
    ha3 = axes('Position',[0.675,0.1 , 0.3,0.8]);
    grid on; hold on;
end


% do LPVO
for imgIdx = 1:M
    %% 1. feature tracking : re-assign variables for next ieration
    
    % image
    imageLast = imageCur;
    imageCur = getImgInTUMRGBDdataset(datasetPath, ICSLRGBDdataset, cam, imgIdx, 'gray');
    
    imageShow = imresize(imageLast, (1/showDSRate));
    harrisLast = cv.cornerHarris(imageShow, 'BlockSize', 3);
    
    % features
    featuresTemp = featuresLast;
    featuresLast = featuresCur;
    featuresCur = featuresTemp;
    
    % image points
    imagePointsTemp = imagePointsLast;
    imagePointsLast = imagePointsCur;
    imagePointsCur = imagePointsTemp;
    imagePointsCur = cell(0);
    
    % for the first time in this loop
    if (~systemInited_ft)
        systemInited_ft = true;
    elseif (systemInited_ft)
        %% 1. feature tracking : feature detection with goodFeaturesToTrack
        
        recordFeatureNum = totalFeatureNum;
        for i = 1:ySubregionNum
            for j = 1:xSubregionNum
                ind = xSubregionNum * (i-1) + j;
                numToFind = maxFeatureNumPerSubregion - subregionFeatureNum(ind);
                
                if (numToFind>0)
                    
                    subregionLeft = xBoundary + (subregionWidth * (j-1)) + 1;
                    subregionTop = yBoundary + (subregionHeight * (i-1)) + 1;
                    
                    uStart = subregionLeft;
                    uEnd = subregionLeft + subregionWidth - 1;
                    vStart = subregionTop;
                    vEnd = subregionTop + subregionHeight - 1;
                    subregionTemp = imageLast(vStart:vEnd, uStart:uEnd);
                    
                    featuresTemp = cv.goodFeaturesToTrack(subregionTemp, 'QualityLevel', 0.1, 'MinDistance', 5.0, 'BlockSize', 3, 'UseHarrisDetector', true, 'K', 0.04);
                    for k=1:size(featuresTemp, 2)
                        featuresLast(totalFeatureNum+k) = featuresTemp(k);
                    end
                    
                    numFound = 0;
                    for k=1:size(featuresTemp, 2)
                        featuresLast{totalFeatureNum+k}(1) = featuresLast{totalFeatureNum+k}(1) + (subregionLeft-1);
                        featuresLast{totalFeatureNum+k}(2) = featuresLast{totalFeatureNum+k}(2) + (subregionTop-1);
                        
                        xInd = round( (featuresLast{totalFeatureNum+k}(1) + 0.5) / showDSRate ) + 1;
                        yInd = round( (featuresLast{totalFeatureNum+k}(2) + 0.5) / showDSRate ) + 1;
                        
                        if (harrisLast(yInd, xInd) >= 1e-6)
                            numFound = numFound + 1;
                            featuresIndFromStart = featuresIndFromStart +1;
                            
                            featuresLast{totalFeatureNum+numFound}(1) = featuresLast{totalFeatureNum+k}(1);
                            featuresLast{totalFeatureNum+numFound}(2) = featuresLast{totalFeatureNum+k}(2);
                            featuresInd(totalFeatureNum+numFound) = featuresIndFromStart;
                            featuresAge(totalFeatureNum+numFound) = 0;
                            
                            clear uStart uEnd vStart vEnd
                            uStart = round(featuresLast{totalFeatureNum+k}(1)+1) - ((ZNCCwinSize-1)/2);
                            uEnd = uStart + ZNCCwinSize - 1;
                            vStart = round(featuresLast{totalFeatureNum+k}(2)+1) - ((ZNCCwinSize-1)/2);
                            vEnd = vStart + ZNCCwinSize - 1;
                            featuresLastPatch{totalFeatureNum+numFound} = imageLast(vStart:vEnd, uStart:uEnd);
                        end
                    end
                    
                    totalFeatureNum = totalFeatureNum + numFound;
                    subregionFeatureNum(ind) = subregionFeatureNum(ind) + numFound;
                end
            end
        end
        
        
        %% 1. feature tracking : feature tracking with KLT tracker
        
        [featuresCur, featuresFound, featuresError]  = cv.calcOpticalFlowPyrLK(imageLast, imageCur, featuresLast(1:totalFeatureNum), 'WinSize', [winSize winSize], 'MaxLevel', 3);
        
        for i=1:totalSubregionNum
            subregionFeatureNum(i) = 0;
        end
        
        featureCount = 0;
        for i=1:totalFeatureNum
            
            if (featuresAge(i) <= 120)
                
                trackDis = sqrt( (featuresLast{i}(1) - featuresCur{i}(1))*(featuresLast{i}(1) - featuresCur{i}(1)) + (featuresLast{i}(2) - featuresCur{i}(2))*(featuresLast{i}(2) - featuresCur{i}(2)) );
                
                if (~(trackDis > maxTrackDis || featuresCur{i}(1) < xBoundary || featuresCur{i}(1) > (imageWidth - xBoundary) ...
                        || featuresCur{i}(2) < yBoundary || featuresCur{i}(2) > (imageHeight - yBoundary)))
                    
                    I1imagePatch = double(featuresLastPatch{i});
                    
                    clear uStart uEnd vStart vEnd
                    uStart = round(featuresCur{i}(1)+1) - ((ZNCCwinSize-1)/2);
                    uEnd = uStart + ZNCCwinSize - 1;
                    vStart = round(featuresCur{i}(2)+1) - ((ZNCCwinSize-1)/2);
                    vEnd = vStart + ZNCCwinSize - 1;
                    I2imagePatch = double(imageCur(vStart:vEnd, uStart:uEnd));
                    
                    % ZNCC error metric
                    normI1imagePatch = I1imagePatch(:) - mean(I1imagePatch(:));
                    normI2imagePatch = I2imagePatch(:) - mean(I2imagePatch(:));
                    ZNCCError = sum(normI1imagePatch .* normI2imagePatch) / ...
                        sqrt(sum(normI1imagePatch.^2) * sum(normI2imagePatch.^2));
                    
                    if (ZNCCError >= 0.90)
                        xInd = floor( (featuresLast{i}(1) - xBoundary) / subregionWidth ) + 1;
                        yInd = floor( (featuresLast{i}(2) - yBoundary) / subregionHeight ) + 1;
                        ind = xSubregionNum * (yInd-1) + xInd;
                        
                        if (subregionFeatureNum(ind) < maxFeatureNumPerSubregion)
                            featureCount = featureCount + 1;
                            subregionFeatureNum(ind) = subregionFeatureNum(ind) + 1;
                            
                            featuresCur{featureCount}(1) = featuresCur{i}(1);
                            featuresCur{featureCount}(2) = featuresCur{i}(2);
                            featuresLast{featureCount}(1) = featuresLast{i}(1);
                            featuresLast{featureCount}(2) = featuresLast{i}(2);
                            featuresInd(featureCount) = featuresInd(i);
                            
                            featuresAge(featureCount) = featuresAge(i) + 1;
                            featuresLastPatch{featureCount} = featuresLastPatch{i};
                            
                            point.u = (featuresCur{featureCount}(1) - cam.K(1,3)) / cam.K(1,1);
                            point.v = (featuresCur{featureCount}(2) - cam.K(2,3)) / cam.K(2,2);
                            point.ind = featuresInd(featureCount);
                            point.age = featuresAge(featureCount);
                            imagePointsCur{end+1} = point;
                            
                            if (i > recordFeatureNum)
                                point.u = (featuresLast{featureCount}(1) - cam.K(1,3)) / cam.K(1,1);
                                point.v = (featuresLast{featureCount}(2) - cam.K(2,3)) / cam.K(2,2);
                                imagePointsLast{end+1} = point;
                            end
                        end
                    end
                end
            end
        end
        totalFeatureNum = featureCount;
    end
    
    
    %% 2. Manhattan frame tracking
    
    % image
    imageCurForLine = getImgInTUMRGBDdataset(datasetPath, ICSLRGBDdataset, cam, imgIdx, 'gray');
    imageCurForMW = getImgInTUMRGBDdataset(datasetPath, ICSLRGBDdataset, cam, imgIdx, 'rgb');
    depthCurForMW = getImgInTUMRGBDdataset(datasetPath, ICSLRGBDdataset, cam, imgIdx, 'depth');
    [imageCurForMW, depthCurForMW] = getImgPyramid(imageCurForMW, depthCurForMW, optsLPVO.imagePyramidLevel);
    
    % vanishing direction vector
    [vDV, linePairIdx, lines] = estimateVanishingDirectionGreatcircle(imageCurForLine, cam, optsLPVO);
    
    % surface normal vector
    [sNV, sPP] = estimateSurfaceNormalGradient_mex(imageCurForMW, depthCurForMW, cam, optsLPVO);
    totalUnitVector = [sNV, vDV];
    
    
    % for the first time in this loop
    if (~systemInited_MF)
        
        % initialize and seek the dominant MF
        [MF_can, FindMF] = seekManhattanWorld(totalUnitVector, optsLPVO);
        if (FindMF == 1)
            R_cM = ClusterMMF(MF_can, optsLPVO.ratio);
            if (isempty(R_cM) == 0)
                R_cM = R_cM{1};
                R_c1M = R_cM;
                R_gM = R_gc1 * R_c1M;
                systemInited_MF = true;
                disp('Initialization done!');
            end
        end
    elseif (systemInited_MF)
        
        % tracking MF
        [R_cM, isTracked] = trackManhattanWorld_LPVO(R_cM, sNV, vDV, optsLPVO);
        
        % if lost tracking
        if (isTracked == 0)
            disp(num2str(imgIdx));
            disp('lost tracking!');
            break;
        end
        
        % check MF tracking
        [R_cM, isTracked] = checkManhattanFrameTracking(R_cM, sNV, vDV, optsLPVO);
        
        
        % update current camera pose
        R_gc_current = R_gM * inv(R_cM);
        R_gc_LPVO(:,:,imgIdx) = R_gc_current;
    end
    
    
    %% 3. frame to frame motion estimation : re-assign variables for next ieration
    
    % for the first time in this loop
    if (~systemInited_VO)
        systemInited_VO = true;
    elseif  (systemInited_VO)
        %% 3. frame to frame motion estimation : find relationship between points
        
        depthImageTemp = getImgInTUMRGBDdataset(datasetPath, ICSLRGBDdataset, cam, imgIdx-1, 'depth');
        ipRelations = findPointTrackingResult(imagePointsLast, imagePointsCur, depthImageTemp, imgIdx, cam, optsLPVO);
        
        
        %% 3. frame to frame motion estimation : estimate 6 DoF motion
        
        if (isTracked)
            
            % interpret the rotational motion [R]
            R_21 = inv(R_gc_LPVO(:,:,imgIdx)) * R_gc_LPVO(:,:,imgIdx-1);
            
            % estimate the translational motion [t]
            xi_21 = estimatePoseTranslation(R_21, ipRelations, cam, optsLPVO);
        else
            
            % estimate the rotational and translational motion [R,t]
            xi_21 = estimatePoseRotationTranslation(ipRelations, cam, optsLPVO);
            
            % re-arrange rotation information
            R_21 = angle2rotmtx([xi_21(4); xi_21(5); xi_21(6)]);
            R_gc_LPVO(:,:,imgIdx) = R_gc_LPVO(:,:,imgIdx-1) * inv(R_21);
            R_cM = inv(R_gc_LPVO(:,:,imgIdx)) * R_gM;
        end
        
        
        % re-assign 6 DoF camera motion
        tx = xi_21(1);
        ty = xi_21(2);
        tz = xi_21(3);
        phi = xi_21(4);
        theta = xi_21(5);
        psi = xi_21(6);
        
        
    end
    
    
    %% 4. update 6 DoF camera pose and visualization
    
    if (imgIdx >= 2)
        % update current camera pose
        t_21 = [tx; ty; tz];
        R_21 = angle2rotmtx([phi; theta; psi]);
        T_21 = [R_21, t_21;
            0, 0, 0, 1];
        
        T_gc_current = T_gc_current * inv(T_21);
        T_gc_LPVO{imgIdx} = T_gc_current;
        
        
        % visualize current status
        plots_ICSL_RGBD;
    end
    
    
end

% convert camera pose representation
stateEsti_LPVO = zeros(6,M);
R_gc_LPVO = zeros(3,3,M);
for k = 1:M
    R_gc_LPVO(:,:,k) = T_gc_LPVO{k}(1:3,1:3);
    stateEsti_LPVO(1:3,k) = T_gc_LPVO{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_LPVO(:,:,k));
    stateEsti_LPVO(4:6,k) = [roll; pitch; yaw];
end


%% plot error metric value (RPE, ATE)


% 1) LPVO motion estimation trajectory results
figure;
plot3(stateEsti_LPVO(1,:),stateEsti_LPVO(2,:),stateEsti_LPVO(3,:),'r','LineWidth',2); grid on;
legend('LPVO Matlab'); plot_inertial_frame(0.5); axis equal; view(-158, 38);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;


% 2) LPVO motion estimation trajectory results
figure;
subplot(3,2,1);
plot(stateEsti_LPVO(1,:),'r','LineWidth',2); grid on; axis tight; ylabel('x (m)');
legend('LPVO Matlab');
subplot(3,2,3);
plot(stateEsti_LPVO(2,:),'r','LineWidth',2); grid on; axis tight; ylabel('y (m)');
subplot(3,2,5);
plot(stateEsti_LPVO(3,:),'r','LineWidth',2); grid on; axis tight; ylabel('z (m)');
subplot(3,2,2);
plot(stateEsti_LPVO(4,:),'r','LineWidth',2); grid on; axis tight; ylabel('roll (rad)');
subplot(3,2,4);
plot(stateEsti_LPVO(5,:),'r','LineWidth',2); grid on; axis tight; ylabel('pitch (rad)');
subplot(3,2,6);
plot(stateEsti_LPVO(6,:),'r','LineWidth',2); grid on; axis tight; ylabel('yaw (rad)');


%% save the experiment data for ICRA 2018

if (toSave)
    save([SaveDir '/LPVO.mat']);
end

