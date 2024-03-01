%% Numerical simulation for Fig 5b
% observer: straight motion along bearing vector
% target: square-shape, stationary and spin
% 
clear all
clc
close all

%% initial parameters (can be modified)
total_time = 12;                % total simulation time
dt = 0.02;                      % time interval
total_t = round(total_time/dt); % total simulation steps
target_size = 1;                % target's size (\ell) 
est_init_tar_size = 0.8;        % initial estimated target's size

% motion states (pos, vel)
reserve_state_obs = zeros(4, total_t);
reserve_state_tar = zeros(4, total_t);
reserve_state_obs(:, 1) = [0;5;0;4];
reserve_state_tar(:, 1) = [0;10.5;0;0];
reserve_state_tar_yaw = zeros(1, total_t);   % target's yaw influence ell
reserve_state_tar_size = ones(1, total_t);   % groundtruth of ell

% parameters for target's motion
tar_omega = pi/8;     % target's spining speed

% parameters for observer's motion
ao = 2;               % acceleration


%% states initialization
sigma_theta = 0.01; 
sigma_g = 0.01;

reserve_g_mear = zeros(2, total_t);
reserve_theta_mear = zeros(1, total_t);

reserve_est_state_bo = zeros(4, total_t);
reserve_est_state_ba = zeros(5, total_t); 

reserve_est_dis_bo = zeros(1, total_t);
reserve_est_dis_ba = zeros(1, total_t);



%% start 
for t = 1:(total_t-1)
    % measurement(obtain bearing and theta)
    [g, theta, size_now] = observe(reserve_state_tar(1:2, t), reserve_state_obs(1:2, t), ...
                        reserve_state_tar_yaw(1, t), ...
                        target_size, sigma_theta, sigma_g);
    reserve_g_mear(:, t) = g;
    reserve_theta_mear(1, t) = theta;
    reserve_state_tar_size(1, t) = size_now;   % groundtruth of ell
    % Pseudo linear Kalman filter estimator
    if t == 1
        reserve_est_state_ba(:,t)=[[0; 8]; [0;0]; est_init_tar_size];
        reserve_est_state_bo(:,t) = [[0; 8]; [0;0]];
    else
        % bearing-angle method
        reserve_est_state_ba(:,t) = KF_bearing_angle(g, theta, ...
            reserve_est_state_ba(:,t-1), reserve_state_obs(1:2, t), dt);
        % bearing-only method
        reserve_est_state_bo(:, t) = KF_bearing(g, ...
            reserve_est_state_bo(:, t-1), reserve_state_obs(1:2, t), dt);
    end
    % motion 
    if t < total_t
        % target's motion states update (stationary and spin)
        yaw_tem = reserve_state_tar_yaw(1, t) + dt * tar_omega;
        if yaw_tem > pi
            yaw_tem = yaw_tem - 2*pi;
        elseif yaw_tem < -pi
            yaw_tem = yaw_tem + 2*pi;
        end
        reserve_state_tar_yaw(1, t+1) = yaw_tem; 
        reserve_state_tar(1:2, t+1) = reserve_state_tar(1:2, t);
        %  observer's motion states update (straight motion along the bearing vector)
        reserve_state_obs(:, t+1) = control_observer_1_line(ao, ...
                                                 reserve_state_obs(:, t), dt);
    end
    % calculate the error of distance
    reserve_est_dis_ba(1, t) = norm(reserve_state_tar(1:2, t) - reserve_est_state_ba(1:2, t));
    reserve_est_dis_bo(1, t) = norm(reserve_state_tar(1:2, t) - reserve_est_state_bo(1:2, t));
end

%% save data
% because each run obtain random noises, result in different curves.
% the "data_Fig5b.mat" is the simulation results shown in Fig 5a.
save data.mat 

%% for figure plot
% to see simulation results shown in the paper Fig 5a, 
% please change the data file to "data_5a.mat" in the plot_my.m file
plot_my;



