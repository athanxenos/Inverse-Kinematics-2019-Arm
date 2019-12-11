function [dL] = angular2linear(a,b,dtheta,L)
%Converts Angular velocity (dtheta) to Linear Actuator Velocity (dL)
%dtheta refers to internal angle of each joint, different to DH angle
%DH Angle Velocity is same magnitude as dtheta but may have different sign
%depending on joint

%Inputs
%a---side length of triangle (mm) (connected to theta)
%b---side length of triangle (mm) (connected to theta)
%dtheta---angular velocity of interior angle theta(related to DH Angle)
%(deg/s)
%L---current length of linear actuator (mm)

%Outputs
%dL---velocity of linear actuator (mm/s)

L_max=sqrt(a^2+b^2+2*a*b);
L_min=sqrt(a^2+b^2-2*a*b);

 if (L>L_max)||(L<L_min)
        fprintf('Error: Outside Bounds\n');
        dL=0;
 else
    root=sqrt(1-(a^2+b^2-L^2)^2/(4*a^2*b^2));
    dL=dtheta*root*a*b/L;
 end

end

