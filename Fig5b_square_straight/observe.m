function [g, theta, size_now] = observe(p_t, p_c, tar_yaw, size, sigma_theta, sigma_g)
%OBSERVE, calculate noised bearing and angle measurements
% together with groundtruth of ell
%% calculate groundtruth bearing (g)
r = norm(p_t-p_c);
g = (p_t-p_c)/r;

%% calculate groundtruth angle (theta)
angle_tem = atan2(g(2, 1), g(1, 1));
angle_tem = angle_tem - tar_yaw;
if angle_tem < 0
    angle_tem = angle_tem + 2*pi;
end
while angle_tem > pi/2
    angle_tem = angle_tem - pi/2;
end
angle_tem = angle_tem - pi/4;
size_now = size*cos(angle_tem);   % the groundtruth of ell
theta = size_now/r;

%% add Gauss noise
epsilon =  normrnd(0, sigma_g);
R = [ cos(epsilon), sin(epsilon);
    -sin(epsilon), cos(epsilon)];
g = R*g;
theta = theta + normrnd(0, sigma_theta);

end

 