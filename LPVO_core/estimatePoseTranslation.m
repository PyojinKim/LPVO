function [xi] = estimatePoseTranslation(R_21, ipRelations, cam, optsLPVO)

% pre-defined variables
iterNum = optsLPVO.iterNum;


%% interpret the rotational motion [R]

% decompose R_21
tempEulerAngle = rotmtx2angle(R_21);
phi = tempEulerAngle(1);
theta = tempEulerAngle(2);
psi = tempEulerAngle(3);


%% estimate the translational motion [t]

% initialize 3 DoF model parameters
tx = 0;
ty = 0;
tz = 0;


% LM optimization
ipRelationsNum = size(ipRelations, 2);
ptNumNoDepthRec = 0;
ptNumWithDepthRec = 0;
meanValueWithDepthRec = 100000;
for iterCount = 1:iterNum
    
    ipRelations2 = cell(0);
    ipy2 = cell(0);
    
    ptNumNoDepth = 0;
    ptNumWithDepth = 0;
    meanValueNoDepth = 0;
    meanValueWithDepth = 0;
    
    for i = 1:ipRelationsNum
        
        ipr = ipRelations{i};
        
        uv1_d = [ipr.x; ipr.y];
        uv2_d = [ipr.z; ipr.h];
        [uv1_u] = undistortPts_normal_mex(uv1_d, cam);
        [uv2_u] = undistortPts_normal_mex(uv2_d, cam);
        u1 = uv1_u(1);
        v1 = uv1_u(2);
        u2 = uv2_u(1);
        v2 = uv2_u(2);
        
        if (abs(ipr.v) < 0.5) % no depth point
            
            jacobianTemp = df_nodepth_dt([tx; ty; tz; phi; theta; psi; u2; v2; 0; 0; 0; u1; v1]);
            
            ipr2.x = jacobianTemp(1);
            ipr2.y = jacobianTemp(2);
            ipr2.z = jacobianTemp(3);
            
            y2 = f_nodepth([tx; ty; tz; phi; theta; psi; u2; v2; 0; 0; 0; u1; v1]);
            
            
            if (ptNumNoDepthRec < 50 || iterCount < 25 || abs(y2) < 2 * meanValueWithDepthRec / 10000)
                scale = 100;
                ipr2.x = ipr2.x * scale;
                ipr2.y = ipr2.y * scale;
                ipr2.z = ipr2.z * scale;
                y2 = y2 * scale;
                
                ipRelations2{end+1} = ipr2;
                ipy2{end+1} = y2;
                
                ptNumNoDepth = ptNumNoDepth + 1;
            else
                ipRelations{i}.v = -1;
            end
        elseif (abs(ipr.v - 1) < 0.5 || abs(ipr.v - 2) < 0.5) % with depth point
            
            X_1 = u1 * ipr.s;
            Y_1 = v1 * ipr.s;
            Z_1 = ipr.s;
            jacobian1Temp = df_depth_1_dt([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
            jacobian2Temp = df_depth_2_dt([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
            
            ipr3.x = jacobian1Temp(1);
            ipr3.y = jacobian1Temp(2);
            ipr3.z = jacobian1Temp(3);
            
            ipr4.x = jacobian2Temp(1);
            ipr4.y = jacobian2Temp(2);
            ipr4.z = jacobian2Temp(3);
            
            y3 = f_depth_1([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
            y4 = f_depth_2([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
            
            
            if (ptNumWithDepthRec < 50 || iterCount < 25 || sqrt(y3 * y3 + y4 * y4) < 2 * meanValueWithDepthRec)
                ipRelations2{end+1} = ipr3;
                ipy2{end+1} = y3;
                
                ipRelations2{end+1} = ipr4;
                ipy2{end+1} = y4;
                
                ptNumWithDepth = ptNumWithDepth + 1;
                meanValueWithDepth = meanValueWithDepth + sqrt(y3 * y3 + y4 * y4);
            else
                ipRelations{i}.v = -1;
            end
        end
    end
    
    meanValueWithDepth = meanValueWithDepth / (ptNumWithDepth + 0.01);
    ptNumNoDepthRec = ptNumNoDepth;
    ptNumWithDepthRec = ptNumWithDepth;
    meanValueWithDepthRec = meanValueWithDepth;
    
    ipRelations2Num = size(ipRelations2, 2);
    if (ipRelations2Num > 5)
        
        matA = zeros(ipRelations2Num, 3);
        matB = zeros(ipRelations2Num, 1);
        
        for i=1:ipRelations2Num
            ipr2 = ipRelations2{i};
            
            matA(i, 1) = ipr2.x;
            matA(i, 2) = ipr2.y;
            matA(i, 3) = ipr2.z;
            matB(i, 1) = - 0.1 * ipy2{i};
        end
        
        matAt = matA.';
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        matX = cv.solve(matAtA, matAtB);   % deltaTemp = inv(matA.' * matA) * matA.' * matB;
        
        tx = tx + matX(1, 1);
        ty = ty + matX(2, 1);
        tz = tz + matX(3, 1);
        
        deltaT = sqrt(matX(1, 1) * 100 * matX(1, 1) * 100 +...
            matX(2, 1) * 100 * matX(2, 1) * 100 +...
            matX(3, 1) * 100 * matX(3, 1) * 100);
        
        if (deltaT < 0.0001)
            break;
        end
    end
end


% return 6 DoF camera motion in vector
xi = [tx; ty; tz; phi; theta; psi];


end

