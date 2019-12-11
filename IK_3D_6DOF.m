close all; clear; clc;

%Initial angle values (arm position)
theta=[0;90;-90;0;-90;-90];

%Initial cartesian velocity vector (xyz) (normalised)
xdot=[0;1;0];
xdot=xdot/norm(xdot);

%Restrict angular velocity to limit wrist rotation (Euler Angles)
wdot=[0;0;0];

%Combine into single velocity vector
vdot=[xdot;wdot];

%Numerical Method Step Size(variable)
step=1; 
n=50;

%Plot start position
figure
hold
P_end1=plotArm_2019(theta);

for i=1:n
    %Forward Kinematics for current theta values
    [T01,T02,T03,T04,T05,T06,P_end] = FK_2019(theta);

    %Calculate Jacobian Matrix
    [J] = Jacobian6DOF_2019(T01,T02,T03,T04,T05,T06,P_end);

    %Pseudoinverse Jacobian to find angular velocity
    thetadot=pinv(J)*vdot;

    %Convert angular velocity to deg/s
    dtheta=rad2deg(thetadot)*step;

    %Update angle based on weighted angular velocity
    theta=theta+dtheta;
end

%Plot end position
P_end2=plotArm_2019(theta);