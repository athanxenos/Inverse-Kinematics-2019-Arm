function [T01,T02,T03,T04,T05,T06,P_end] =FK_2019(theta)
%Forward Kinematics Function for 2019 Arm
%DH values are predetermined by arm geometry except for theta values,units
%in cm/deg

%Input:
%theta-6x1 vector containing all theta values(deg)

%Output:
%(4x4) T Matrices of 6 joints
%(4x1) P_end- position of end effector respect to origin (with 1 as last
%entry)

%Constant DH Paramters 
d1=0;a0=0;alpha0=0; %parameter joint1
d2=0;a1=7.5;alpha1=90; %parameter joint2
d3=0;a2=65;alpha2=0; %parameter joint3
d4=0;a3=70;alpha3=0; %parameter joint4
d5=0;a4=0;alpha4=-90; %parameter joint5
d6=0;a5=0;alpha5=-90; %parameter joint6

a6=25;
P6_end=[0; 0; a6; 1]; %End effector position w.r.t frame 6,

%Determine T matrices for current theta values
T01=DHmatrix(theta(1),d1,a0,alpha0);
T12=DHmatrix(theta(2),d2,a1,alpha1);
T23=DHmatrix(theta(3),d3,a2,alpha2);
T34=DHmatrix(theta(4),d4,a3,alpha3);
T45=DHmatrix(theta(5),d5,a4,alpha4);
T56=DHmatrix(theta(6),d6,a5,alpha5);

T02=T01*T12;
T03=T02*T23;
T04=T03*T34;
T05=T04*T45;
T06=T05*T56;

%Position of end effector respect to ground frame
P_end=transpose(T06*P6_end); 
end

