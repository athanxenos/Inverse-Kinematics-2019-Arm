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
    [W] = weights(theta);
    
    %Pseudoinverse Jacobian with Weights
    %M = J*inv(W)*J';
    %t = M\vdot;
    %thetadot = inv(W)*J'*t;
    %thetadot = inv(W)*J'*inv(J*inv(W)*J')*vdot;
    
    %Pseudoinverse Jacobian to find angular velocity
    thetadot = W^(-0.5)*J'*inv(J*inv(W)*J')*vdot;
    %J_W =J*(W^(-0.5));
    %thetadot=W^(-0.5)*pinv(J_W)*vdot;
    %thetadot=W^(0.5)*thetadot;
    thetadot_test=pinv(J)*vdot;
    %thetadot = W^(0.5)*thetadot;

    %Convert angular velocity to deg/s
    dtheta=rad2deg(thetadot)*step;

    %Update angle based on weighted angular velocity
    [theta,dtheta_weight] = update_theta(theta,dtheta,dtheta_max); 
    
    
    %disp(W(1,1));
    fprintf('Weight is %.3f\n',W(1,1));
    disp(dtheta(1));
    disp(rad2deg(thetadot_test(1))*step);
end

%Plot end position
P_end2=plotArm_2020(theta);