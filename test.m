close all; clear; clc

theta=[0;90;-90;0;-90;-90];
xdot=[1;1;1];
xdotnorm=xdot/norm(xdot);
joints=6;
step=10;

[dtheta] = IK_Velocity_2019(theta,xdotnorm,step,joints);


a1=150;
b1=285;
L1=330;
[dL(1)] = angular2linear(a1,b1,dtheta(2),L1);

a2=320;
b2=80;
L2=360;
[dL(2)] = angular2linear(a2,b2,dtheta(3),L2);

a3=460;
b3=55;
L3=410;
[dL(3)] = angular2linear(a3,b3,dtheta(4),L3);