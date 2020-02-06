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
n=100;
dtheta_max=5;

%Joint Control (3 or 6)
joints=6;
        
%Plot start position
figure
hold
P_end1=plotArm_2020(theta);

for i=1:n
    %Forward Kinematics for current theta values
    [T01,T02,T03,T04,T05,T06,P_end] = FK_2020(theta);

    %Calculate Jacobian Matrix
    [Jv] = Jacobian3DOF_2020(T01,T02,T03,T04,T05,T06,P_end,joints);

    %Pseudoinverse Jacobian to find angular velocity
    thetadot=pinv(Jv)*xdot;

    %Convert angular velocity to deg/s
    dtheta=rad2deg(thetadot)*step;

    %Update angle based on weighted angular velocity
    %Update angle based on weighted angular velocity
    [theta,dtheta_weight] = update_theta(theta,dtheta,dtheta_max); 
end

%Plot end position
P_end2=plotArm_2020(theta);