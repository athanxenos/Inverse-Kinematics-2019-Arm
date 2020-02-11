function [H] = clamping(theta)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global qmin;
global qmax;
n=length(theta);
H=eye(n,n);

for i=1:n
    if (theta(i)>=qmax(i))|| (theta(i)<=qmin(i))
        %H(i,i) = 0;
        
        %fprintf('Clamped\n');
    else
        H(i,i) = 1;
    end
    
end

end

