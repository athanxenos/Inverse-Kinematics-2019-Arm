function [P_end] = plotArm_2020(theta)
%plotArm_2019
%Plots current position of arm and returns position of end effector

%Input:
%theta - 6x1 vector of theta values

%Output:
%P_end - 3x1 vector of position of end effector
%Plot of current arm position

%Forward Kinematics for current theta values
[T01,T02,T03,T04,T05,~,P_end] = FK_2020(theta);

%Calculate position of each joint
P1=transpose(T01(1:3,4));
P2=transpose(T02(1:3,4));
P3=transpose(T03(1:3,4));
P4=transpose(T04(1:3,4));
P5=transpose(T05(1:3,4)); 

%Convert to (x,y,z) coordinates for each joint
Q1=[P1(1,1) P2(1,1) P3(1,1) P4(1,1)  P5(1,1) P_end(1,1)];
Q2=[P1(1,2) P2(1,2) P3(1,2) P4(1,2) P5(1,2) P_end(1,2)];
Q3=[P1(1,3) P2(1,3) P3(1,3) P4(1,3) P5(1,3) P_end(1,3)];

%Plot (x,y,z) points for each joint
plot3(Q1,Q2,Q3,'-o','LineWidth',4);
axis([-150,150,-150,150,-150,150]);
xlabel('x')
ylabel('y')
zlabel('z')
grid on;

end

