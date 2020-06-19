clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% make symbolic variables


syms tx ty tz phi theta psi                   % 6 DoF state vector
syms X_2 Y_2 Z_2 x_2_norm y_2_norm  % point @ {k-1} body frame
syms X_1 Y_1 Z_1 x_1_norm y_1_norm  % point @ {k} inertial frame


% define feature points variables
P_2 = [X_2; Y_2; Z_2];
P_2_norm = [x_2_norm; y_2_norm; 1];
P_1 = [X_1; Y_1; Z_1];
P_1_norm = [x_1_norm; y_1_norm; 1];


% define camera motion variables
R_x = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
R_y = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R_12 = [R_z * R_y * R_x];     % [Inertial frame] = R_12 * [Body frame]
R_21 = R_12.';                     % [Body frame] = R_21 * [Inertial frame]
T_21 = [tx; ty; tz];


%%  calculate Jacobian matrix for translational motion [t]


% residual vector
f_depth_1 = (R_21(1,:) - P_2_norm(1)*R_21(3,:)) * P_1 + T_21(1) - P_2_norm(1) * T_21(3);
f_depth_2 = (R_21(2,:) - P_2_norm(2)*R_21(3,:)) * P_1 + T_21(2) - P_2_norm(2) * T_21(3);
f_nodepth = [-P_2_norm(2)*T_21(3) + T_21(2), P_2_norm(1)*T_21(3) - T_21(1), -P_2_norm(1)*T_21(2) + P_2_norm(2)*T_21(1)] * R_21 * P_1_norm;


% Jacobian of residual vector
df_depth_1_dt = jacobian(f_depth_1, [tx ty tz]);
df_depth_2_dt = jacobian(f_depth_2, [tx ty tz]);
df_nodepth_dt = jacobian(f_nodepth, [tx ty tz]);


% make Jacobian vector as matlab functions
matlabFunction(df_depth_1_dt,'file','df_depth_1_dt.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(df_depth_2_dt,'file','df_depth_2_dt.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(df_nodepth_dt,'file','df_nodepth_dt.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});


% make residual vector as matlab functions
matlabFunction(f_depth_1,'file','f_depth_1.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(f_depth_2,'file','f_depth_2.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(f_nodepth,'file','f_nodepth.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});


%%  calculate Jacobian matrix for 6 DoF camera motion [R,t]


% residual vector
f_depth_1 = (R_21(1,:) - P_2_norm(1)*R_21(3,:)) * P_1 + T_21(1) - P_2_norm(1) * T_21(3);
f_depth_2 = (R_21(2,:) - P_2_norm(2)*R_21(3,:)) * P_1 + T_21(2) - P_2_norm(2) * T_21(3);
f_nodepth = [-P_2_norm(2)*T_21(3) + T_21(2), P_2_norm(1)*T_21(3) - T_21(1), -P_2_norm(1)*T_21(2) + P_2_norm(2)*T_21(1)] * R_21 * P_1_norm;


% Jacobian of residual vector
df_depth_1_dxi = jacobian(f_depth_1, [tx ty tz phi theta psi]);
df_depth_2_dxi = jacobian(f_depth_2, [tx ty tz phi theta psi]);
df_nodepth_dxi = jacobian(f_nodepth, [tx ty tz phi theta psi]);


% make Jacobian vector as matlab functions
matlabFunction(df_depth_1_dxi,'file','df_depth_1_dxi.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(df_depth_2_dxi,'file','df_depth_2_dxi.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(df_nodepth_dxi,'file','df_nodepth_dxi.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});


% make residual vector as matlab functions
matlabFunction(f_depth_1,'file','f_depth_1.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(f_depth_2,'file','f_depth_2.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});
matlabFunction(f_nodepth,'file','f_nodepth.m','vars',{[tx; ty; tz; phi; theta; psi; x_2_norm; y_2_norm; X_1; Y_1; Z_1; x_1_norm; y_1_norm]});





