function [theta_new,dtheta_weight] = update_theta(theta,dtheta,dtheta_max)
%Updates theta with proportional weighting
%Input:
%theta - current theta values
%dtheta - angular velocities from jacobian method
%dtheta_max - upper limit on dthetha values

%Output:
%theta_new - updated theta values
%dtheta_weight - weighted dtheta values
max_delta = max(dtheta);
alpha = 1;

if max_delta > dtheta_max
    alpha = dtheta_max/max_delta;
end
disp(alpha);
dtheta_weight = dtheta*alpha;
theta_new = theta + dtheta_weight;
end

