function [T] = DHmatrix(theta,d,a,alpha)
%Returns DH Matrix, input angles in degrees, lengths in cm
T=[cosd(theta),-sind(theta),0,a;
    sind(theta)*cosd(alpha),cosd(theta)*cosd(alpha),-sind(alpha),-d*sind(alpha);
    sind(theta)*sind(alpha),cosd(theta)*sind(alpha),cosd(alpha),d*cosd(alpha);
    0,0,0,1];
end

