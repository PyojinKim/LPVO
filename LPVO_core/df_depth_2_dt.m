function df_depth_2_dt = df_depth_2_dt(in1)
%DF_DEPTH_2_DT
%    DF_DEPTH_2_DT = DF_DEPTH_2_DT(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    26-Jul-2017 13:55:12

y_2_norm = in1(8,:);
df_depth_2_dt = [0.0,1.0,-y_2_norm];
