function plot_body_frame(R_gb, axisLineStyle, axisLineWidth)


% x-,y-,z- axis of body frame ([x-|y-|z-| and (-) x-|y-|z-]) in body frame
bodyFrameAxis_wrtBody = [eye(3), -eye(3)];


% x-,y-,z- axis of body frame ([x-|y-|z-| and (-) x-|y-|z-]) in inertial frame
bodyFrameAxis_wrtGlobal  = R_gb * bodyFrameAxis_wrtBody;


% draw body frame 3 axis in inertial frame
line([bodyFrameAxis_wrtGlobal(1,1) 0],[bodyFrameAxis_wrtGlobal(2,1) 0],[bodyFrameAxis_wrtGlobal(3,1) 0],'Color','r','LineStyle',axisLineStyle,'LineWidth',axisLineWidth);
line([bodyFrameAxis_wrtGlobal(1,4) 0],[bodyFrameAxis_wrtGlobal(2,4) 0],[bodyFrameAxis_wrtGlobal(3,4) 0],'Color','r','LineStyle',axisLineStyle,'LineWidth',axisLineWidth);

line([bodyFrameAxis_wrtGlobal(1,2) 0],[bodyFrameAxis_wrtGlobal(2,2) 0],[bodyFrameAxis_wrtGlobal(3,2) 0],'Color','g','LineStyle',axisLineStyle,'LineWidth',axisLineWidth);
line([bodyFrameAxis_wrtGlobal(1,5) 0],[bodyFrameAxis_wrtGlobal(2,5) 0],[bodyFrameAxis_wrtGlobal(3,5) 0],'Color','g','LineStyle',axisLineStyle,'LineWidth',axisLineWidth);

line([bodyFrameAxis_wrtGlobal(1,3) 0],[bodyFrameAxis_wrtGlobal(2,3) 0],[bodyFrameAxis_wrtGlobal(3,3) 0],'Color','b','LineStyle',axisLineStyle,'LineWidth',axisLineWidth);
line([bodyFrameAxis_wrtGlobal(1,6) 0],[bodyFrameAxis_wrtGlobal(2,6) 0],[bodyFrameAxis_wrtGlobal(3,6) 0],'Color','b','LineStyle',axisLineStyle,'LineWidth',axisLineWidth);


end