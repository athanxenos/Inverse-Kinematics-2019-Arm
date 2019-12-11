close all; clear; clc;

%Initial angle values
theta=[0;90;-90;0;-90;-90];

%Initial angular velocities set to 0
dtheta=[0;0;0;0;0;0];
wdot=[0;0;0];

%Initial Target
Xend=100;Yend=100;Zend=50; 

%Numerical Method Step Size(variable)
step=0.5; 

%Error Threshold (variable)
error_threshold=0.5;

%Joint Control (3 or 6)
joints=3;
a=0;b=0;n=0;

while a==0 %Infinite Loop
    while b==0 %Loop until under error threshold
        %Iteration Counter
        n=n+1;
        
        %Check for position outside range
        if(n>100)
           b=1;
        end
        
        %Update angle based on weighted angular velocity
        theta(1)=theta(1)+dtheta(1)*step;
        theta(2)=theta(2)+dtheta(2)*step;
        theta(3)=theta(3)+dtheta(3)*step;
        theta(4)=theta(4)+dtheta(4)*step;
        theta(5)=theta(5)+dtheta(5)*step;
        theta(6)=theta(6)+dtheta(6)*step;
        
        %Forward Kinematics
        [T01,T02,T03,T04,T05,T06,P_end] = FK_2019(theta);
        
        %Plot orientation of arm
        P_end1=plotArm_2019(theta);
        
        %Display end effector or spherical wrist position
        disp('end effector pos:');disp(P_end(1,1:3)); 

        %Calculate Jacobian Matrix
        [Jv] = Jacobian3DOF_2019(T01,T02,T03,T04,T05,T06,P_end,joints);
        
        %Set current end effector/wrist position as initial
        Xinit=P_end(1,1);Yinit=P_end(1,2); Zinit=P_end(1,3);
         
        %Estimate speed based on difference in position
        Xspeed=(Xend-Xinit);
        Yspeed=(Yend-Yinit);
        Zspeed=(Zend-Zinit);
        
        %Pseudoinverse Jacobian to find angular velocity
        thetadot=pinv(Jv)*[Xspeed;Yspeed;Zspeed];
        
        %Take difference in distance as error
        X_error=Xend-Xinit;
        Y_error=Yend-Yinit;
        Z_error=Zend-Zinit;
        
        %Stop iteration when positional error reaches threshold
        OriError=sqrt(X_error^2+Y_error^2+Z_error^2);
        if OriError<=error_threshold
            b=1;
        end
        
        %Convert angular velocity to deg/s
        dtheta=rad2deg(thetadot);
           
        pause(0.01);
    end
    if b==1
       fprintf('n=%d\n',n);
       
       if (n>100)
         fprintf('Target position can''t be reached\n');
       end
       n=0;
       %Input target position
       Xend= input('Type target X position\n');
       Yend= input('Type target Y position\n');
       Zend= input('Type target Z position\n');
       b=0;
       
    end
end