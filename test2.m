close all; clear; clc;

%Initial angle values
theta=[0;90;-90;0;-90;-90];
wdot=[0;0;0];
direction=2;
a=1;
 %Numerical Method Step Size(variable)
    step=10; 
    
    %Plot start position
figure
P_end1=plotArm_2019(theta);
while a==1
    %Initial velocity vector
    if direction==1
        xdot=[1;0;0];
    elseif direction==2
        xdot=[0;1;0];
    else
        xdot=[0;0;1];
    end

    vdot=[xdot;wdot];

    [theta] = IK_6DOF(theta,vdot,step);

    %Plot end position
    P_end2=plotArm_2019(theta);

    %Input target position
    direction= input('Type target direction (X=1,Y=2,Z=3)\n');
end