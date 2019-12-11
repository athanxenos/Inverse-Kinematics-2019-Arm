function [theta_end,n] = IK_Position_2019(theta_start,p_end,joints)
%Inverse Kinematics Function for 2019 Arm
%Returns theta values for desired end effector position

%Input:
%theta_start - initial theta values, current position of arm, (6x1) vector
%p_end - final target position of end effector, (3x1) vector
%joints - either (3) or (6), toggles using only arm or arm+wrist movement

%Output:
%theta_end - final theta values to achieve target position, (6x1) vector

%Initialise theta values
theta=theta_start;

%Initial angular velocities set to 0
dtheta=[0;0;0;0;0;0];

%Target Positon and Range
Xend=p_end(1);Yend=p_end(2);Zend=p_end(3); 
range=sqrt(Xend^2+Yend^2+Zend^2);

%Numerical Method Step Size(variable)
step=0.5; 

%Error Threshold (variable)
error_threshold=0.5;

b=0;n=0;

while b==0 %Loop until under error threshold
    %Iteration Counter
    n=n+1; 
    
    %Check for position outside range
    if(n>100)
       b=1;
    end

    %Update angle based on weighted angular velocity
    theta=theta+dtheta*step;
    
    %Forward Kinematics
    [T01,T02,T03,T04,T05,T06,P_end] = FK_2019(theta);

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
end

if b==1
   %Return nothing if outside of arm range
   fprintf('Target range is %.1fcm\n',range);
    
   if(range>167.5)
      fprintf('Target position outside of arm range of 167.5cm\n');
   end 
   
   if (n>100)
     fprintf('Target position could not be reached, try 6 DOF if within arm range of 167.5cm\n');
     theta_end=theta_start;
   else
    %Return final theta values
    theta_end=theta;
    fprintf('Target position reached\n');
   end
end

end

