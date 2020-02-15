function [W,H_next] = weights(theta,H_prev)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global qmin;
global qmax;

n = length(theta);
m=2;
W = zeros(n);
dH = zeros(n,1);
H_next = zeros(n,1);

for i=1:n
    dH(i) = (qmax(i)-qmin(i))^2*(2*theta(i)-qmax(i)-qmin(i));
    dH(i) = dH(i)/((2*m*(qmax(i)-theta(i))^2*(theta(i)-qmin(i))^2));
    H_next(i) = dH(i);
    
    if (abs(H_next(i))-abs(H_prev(i))>=0)
        W(i,i) = 1 + abs(dH(i));
    else
        W(i,i) = 1;
    end
end

end

