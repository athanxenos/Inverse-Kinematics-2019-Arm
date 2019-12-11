function [euler] = EulerAngles(T06)
%Returns Euler angles for given T matrix
%Input:
%T06 - 4x4 Transformation Matrix

%Output:
%euler - 3x1 vector of euler angles, alpha,beta,gamma (radians)
beta=asind(-T06(3,1));

if beta==90
    beta=asin(-T06(3,1));
    alpha=0;
    gamma=atan2(T06(1,2),T06(2,2));
elseif beta==-90
    beta=asin(-T06(3,1));
    alpha=0;
    gamma=-atan2(T06(1,2),T06(2,2));
else
    beta=asin(-T06(3,1));
    alpha=atan2(T06(2,1),T06(1,1));
    gamma=atan2(T06(3,2),T06(3,3));
end

euler=[alpha;beta;gamma];
end

