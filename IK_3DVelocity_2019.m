close all; clear; clc;

%Initial angle values
theta=[0;90;-90;0;-90;-90];

%Initial velocity vector
%x=[1;0;0];
x=[0;1;0];
%x=[0;0;1];
%x=[1;1;1];
xdot=x/norm(x);

%Numerical Method Step Size(variable)
step=1;
n=50;

%Joint Control (3 or 6)
joints=3;
        
%Plot start position
figure
hold
P_end1=plotArm_2019(theta);

for i=1:n
    %Forward Kinematics for current theta values
    [T01,T02,T03,T04,T05,T06,P_end] = FK_2019(theta);

    %Calculate Jacobian Matrix
    [Jv] = Jacobian3DOF_2019(T01,T02,T03,T04,T05,T06,P_end,joints);

    %Pseudoinverse Jacobian to find angular velocity
    thetadot=pinv(Jv)*xdot;

    %Convert angular velocity to deg/s
    dtheta=rad2deg(thetadot)*step;

    %Update angle based on weighted angular velocity
    theta=theta+dtheta;
end

%Plot end position
P_end2=plotArm_2019(theta);
 
Range1=sqrt(P_end1(1)^2+P_end1(2)^2+P_end1(3)^2);
Range2=sqrt(P_end2(1)^2+P_end2(2)^2+P_end2(3)^2);
dRange=Range2-Range1;

dx=P_end2(1)-P_end1(1);
dy=P_end2(2)-P_end1(2);
dz=P_end2(3)-P_end1(3);