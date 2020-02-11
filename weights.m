function [W] = weights(theta)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global qmin;
global qmax;

n = length(theta);
m=6;
W = zeros(n);
dH = zeros(n,1);

for i=1:n
    dH(i) = (qmax(i)-qmin(i))^2*(2*theta(i)-qmax(i)-qmin(i));
    dH(i) = dH(i)/((2*m*(qmax(i)-theta(i))^2*(theta(i)-qmin(i))^2));
    W(i,i) = 1 + abs(dH(i));
end

end

