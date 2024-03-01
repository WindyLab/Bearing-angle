%% Numerical simulation for Fig 4b
% observer: straight motion towards the target
% target: circul-shape, stationary
% 
clear all
clc
close all

%% initial parameters (can be modified)
total_N = 100;            % total Monte Carlo simulation iterations 
total_time = 12;          % total simulation time for each Monte Carlo iteration
dt = 0.02;                % time interval
total_t = round(total_time/dt); % total simulation steps for each Monte Carlo iteration
target_size = 1;                % target's size (\ell) 
est_init_tar_size = 0.8;        % initial target's size
% motion states (pos, vel)
reserve_state_obs = zeros(4, total_t);
reserve_state_tar = zeros(4, total_t);
% initial motion states
reserve_state_obs(:, 1) = [0;5;0;4];
reserve_state_tar(:, 1) = [0;10;0;0];
% parameters for observer's motion
ao = 2;    % acceleration value

      
%% states initialization
sigma_theta = 0.01;                                   
sigma_g = 0.01;
reserve_g_real = zeros(2, total_t);
reserve_theta_real = zeros(1, total_t);

reserve_est_state_ba = zeros(5, total_t, total_N);       
reserve_est_state_bo = zeros(4, total_t, total_N);
reserve_est_dis_ba = zeros(1, total_t, total_N);
reserve_est_dis_bo = zeros(1, total_t, total_N);
reserve_g_mear = zeros(2, total_t, total_N);
reserve_theta_mear = zeros(1, total_t, total_N);
reserve_P_bo = zeros(4, 4, total_t, total_N);
reserve_P_ba = zeros(5, 5, total_t, total_N);
reserve_nees_bo = zeros(1, total_t, total_N);
reserve_nees_ba = zeros(1, total_t, total_N);

%% start 
% collect target's and observer's position and velocity
for t = 1:total_t
    if t < total_t
        reserve_state_tar(:, t+1) = reserve_state_tar(:, t);
        reserve_state_obs(:, t+1) = control_observer_1_line(ao, ...
                                                 reserve_state_obs(:, t), dt);
    end
end
% Mote Carlo simulation
for n = 1:total_N
    P_bo = 0.08*diag([1, 1, 1, 1]);
    P_ba = 0.08*diag([1, 1, 1, 1, 1]);
    for t = 1:total_t
        % measurement(obtain bearing and theta)
        [g_mear, theta_mear, g_real, theta_real] = observe(reserve_state_tar(1:2, t), reserve_state_obs(1:2, t), ...
                            target_size, sigma_theta, sigma_g);
        reserve_g_mear(:, t, n) = g_mear;
        reserve_theta_mear(1, t, n) = theta_mear;
        if n == 1
            reserve_g_real(:, t) = g_real;
            reserve_theta_real(1, t) = theta_real;
        end
        % Pseudo linear Kalman filter estimator
        if t == 1 
            reserve_est_state_ba(:,t, n)=[[0; 8]; [0;0]; est_init_tar_size];
            reserve_est_state_bo(:,t, n) = [[0; 8]; [0;0]];
            reserve_P_bo(:, :, t, n) = P_bo;
            reserve_P_ba(:, :, t, n) = P_ba;
        else
            % bearing-angle method
            [reserve_est_state_ba(:,t, n), P_ba] = KF_bearing_angle(P_ba, g_mear, theta_mear, ...
                reserve_est_state_ba(:,t-1, n), reserve_state_obs(1:2, t), dt);
            reserve_P_ba(:,:,t, n) = P_ba;
            % bearing-only method
            [reserve_est_state_bo(:, t, n), P_bo] = KF_bearing(P_bo, g_mear, ...
                reserve_est_state_bo(:, t-1, n), reserve_state_obs(1:2, t), dt);
            reserve_P_bo(:,:,t, n) = P_bo;
        end
        % calculate error of distance
        reserve_est_dis_ba(1, t, n) = norm(reserve_state_tar(1:2, t) - reserve_est_state_ba(1:2, t, n));
        reserve_est_dis_bo(1, t, n) = norm(reserve_state_tar(1:2, t) - reserve_est_state_bo(1:2, t, n));
        % calculate nees
        err_bo = reserve_state_tar(:, t) - reserve_est_state_bo(:, t, n);
        err_ba = [reserve_state_tar(:, t); target_size] - reserve_est_state_ba(:, t, n);
        reserve_nees_bo(1, t, n) = (err_bo' * pinv(reserve_P_bo(:,:,t, n)) * err_bo);
        reserve_nees_ba(1, t, n) = (err_ba' * pinv(reserve_P_ba(:,:,t, n)) * err_ba);
    end
end

%% for figure plot
plot_my;




