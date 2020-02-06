function [dtheta] = IK_Velocity_2020(theta_start,xdot,step,joints)
%IK_Velocity_2019
%Inverse Kinematics that moves based on a velocity vector using 3 or 6
%joints

%Input:
%theta_start - 6x1 vector of initial angle values
%xdot - 3x1 velocity vector
%step - size of jump in numerical method
%joints - 3 or 6, number of angles used to achieve desired position

%Output:
%dtheta - angular velocity to change theta values by

%Initial angle values
theta=theta_start;
     
%Forward Kinematics for current theta values
[T01,T02,T03,T04,T05,T06,P_end] = FK_2019(theta);

%Calculate Jacobian Matrix
[Jv] = Jacobian3DOF_2019(T01,T02,T03,T04,T05,T06,P_end,joints);

%Pseudoinverse Jacobian to find angular velocity
thetadot=pinv(Jv)*xdot;

%Convert angular velocity to deg/s
dtheta=rad2deg(thetadot)*step;

%Update angle based on weighted angular velocity
%theta_end=theta+dtheta;
end

