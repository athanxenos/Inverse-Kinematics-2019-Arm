close all; clear; clc;

global qmin;
global qmax;

%Initial angle values (arm position)
theta=[0;90;-90;0;-90;0];

qmin=[-90,-30,-180,-180,-180,-180];
qmax=[90,120,180,180,180,180];
%qmin=[-180,-180,-180,-180,-180,-180];
%qmax=[180,180,180,180,180,180];

%Initial cartesian velocity vector (xyz) (normalised)
xdot=[-1;-1;0];
xdot=xdot/norm(xdot);

%Restrict angular velocity to limit wrist rotation (Euler Angles)
wdot=[0;0;0];

%Combine into single velocity vector
vdot=[xdot;wdot];

%Numerical Method Step Size(variable)
step=1; 
n=100;
dtheta_max=50;

H_prev = zeros(6,1);

%Plot start position
figure
hold
P_end1=plotArm_2020(theta);

for i=1:n
    fprintf('Step %d\n',i);
    disp(theta(1));
    
    %Forward Kinematics for current theta values
    [T01,T02,T03,T04,T05,T06,P_end] = FK_2020(theta);

    %Calculate Jacobian Matrix
    [J] = Jacobian6DOF_2020(T01,T02,T03,T04,T05,T06,P_end);
    
    %Return Weights for Joint Limits
    [W,H_prev] = weights(theta,H_prev);
    
    %Weighted Jacobian
    J_W =J*W^(0.5);
    
    %Pseudoinverse Jacobian to find angular velocity
    thetadot=pinv(J_W)*vdot;

    %Compare to non-weighted Jacobian
    thetadot_true=pinv(J)*vdot;
    
    %Convert angular velocity to deg/s
    dtheta=rad2deg(thetadot)*step;

    %Update angle based on weighted angular velocity
    [theta,dtheta_weight] = update_theta(theta,dtheta,dtheta_max); 
    
    fprintf('Weight is %.3f\n',W(1,1));
    disp(dtheta(1));
    disp(rad2deg(thetadot_true(1)));
end

%Plot end position
P_end2=plotArm_2020(theta);