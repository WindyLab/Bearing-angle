function [g_mear,theta_mear, g_real, theta_real] = observe(p_t, p_c, size, sigma_theta, sigma_g)
%OBSERVE, calculate noised bearing and angle measurements
%% calculate groundtruth bearing
r = norm(p_t-p_c);
g = (p_t-p_c)/r;

%% calculate groundtruth angle
theta = size/r;

g_real = g;
theta_real = theta;
%% add Gauss noise
epsilon =  0.8*normrnd(0, sigma_g);  % this makes the final standard deviation of the bearing noise to be around 0.01
R = [ cos(epsilon), sin(epsilon);
    -sin(epsilon), cos(epsilon)];
g = R*g;
theta = theta + normrnd(0, sigma_theta);  % the standard deviation of the angle noise is 0.01

g_mear = g;
theta_mear = theta;
end

 