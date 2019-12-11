function [theta_end] = IK_6DOF(theta_start,vdot,step)
%IK_6DOF
%Inverse Kinematic Function that controls the 6DOF of the arm (xyz and
%euler angles). Constrained to keep euler angles constant and preserve
%wrist orientation

%Input:
%theta_start - 6x1 vector of initial theta values
%vdot - 6x1 velocity vector [xdot;wdot], wdot=[0;0;0], xdot is normalised
%velocity vector for xyz direction

%Output:
%theta_end - 6x1 vector of final theta values

%Initialise theta values
theta=theta_start;

%Forward Kinematics for current theta values
[T01,T02,T03,T04,T05,T06,P_end] = FK_2019(theta);

%Calculate Jacobian Matrix
[J] = Jacobian6DOF_2019(T01,T02,T03,T04,T05,T06,P_end);

%Pseudoinverse Jacobian to find angular velocity
thetadot=pinv(J)*vdot;

%Convert angular velocity to deg/s
dtheta=rad2deg(thetadot)*step;

%Update angle based on weighted angular velocity
theta_end=theta+dtheta;
end