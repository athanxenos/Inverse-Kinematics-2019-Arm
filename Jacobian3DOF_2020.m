function [Jv] = Jacobian3DOF_2020(T01,T02,T03,T04,T05,T06,P_end,joints)
%Function to calculate 3x6 Jacobian Matrix at given arm orientation
%Revolute: Jv = zi x (on - oi)
 
%Input:
%T Matrices of first 6 joints
%P_end - Positon of end effector
%joints - Number of joint angles used, either (3) or (6), determines if Jacobian is calculated about 
%position of spherical wrist or end effector
      
%Output:
%Jv - (3x6) Jacobian Matrix 
               
%Toggle for 6DOF/3DOF
if (joints==6)
    %Move about end effector
    O6=P_end(1:3)'; 
else
    %Move about wrist
    O6=T04(1:3,4);  
end

%Calculate each column of Jacobian for each joint
Z1=T01(1:3,3);O1=T01(1:3,4);
Jv1=cross(Z1,(O6-O1));

Z2=T02(1:3,3);O2=T02(1:3,4);
Jv2=cross(Z2,(O6-O2));

Z3=T03(1:3,3);O3=T03(1:3,4); 
Jv3=cross(Z3,(O6-O3));

Z4=T04(1:3,3);O4=T04(1:3,4); 
Jv4=cross(Z4,(O6-O4));

Z5=T05(1:3,3);O5=T05(1:3,4); 
Jv5=cross(Z5,(O6-O5));

Z6=T06(1:3,3);O6=T06(1:3,4); 
Jv6=cross(Z6,(O6-O6));

%Form Jacobian Matrix
Jv=[Jv1 Jv2 Jv3 Jv4 Jv5 Jv6];
end

