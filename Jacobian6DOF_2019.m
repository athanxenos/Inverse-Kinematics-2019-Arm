function [J] = Jacobian6DOF_2019(T01,T02,T03,T04,T05,T06,P_end)
%Function to calculate 6x6 Jacobian Matrix at given arm orientation
%Revolute: Jv = zi x (on - oi, Jw=zi

 
%Input:
%T Matrices of first 6 joints
%P_end - Positon of end effector
      
%Output:
%J - (6x6) Jacobian Matrix 
               
O6=P_end(1:3)'; 

%Calculate each column of Jacobian(velocity) for each joint
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

%Form Jacobian (Velocity) Matrix
Jv=[Jv1 Jv2 Jv3 Jv4 Jv5 Jv6];

%Form Jacobian (Angular Velocity) Matrix
Jw=[Z1 Z2 Z3 Z4 Z5 Z6];

%Combine to form 6x6 Jacobian Matrix
J=[Jv;Jw];
end

