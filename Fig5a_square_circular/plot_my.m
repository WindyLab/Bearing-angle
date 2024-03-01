%% Plot fun
clear all
close all
clc

% to see simulation results shown in the paper Fig 5a, 
% please change the data file to "data_Fig5a.mat"
load('data.mat')   % "data_Fig5a.mat"
speed = 1;         % plot speed 

%% setup fontsize and color
color_obs = '#EDB120';              % color of groundtruth observer 
color_tar = [0.8500 0.3250 0.0980]; % color of groundtruth target 
color_est_t_bo = '#0072BD';         % color of estimated target using bearing-only 
color_est_t_ba = '#77AC30';         % color of estimated target using bearing-angle 
fontsize = 11;
title_fontsize = 13;

rectan_points_x = [-sqrt(2)/4, -sqrt(2)/4, sqrt(2)/4, sqrt(2)/4];
rectan_points_y = [-sqrt(2)/4, sqrt(2)/4, sqrt(2)/4, -sqrt(2)/4];  
rectan_points = [rectan_points_x; rectan_points_y];

%% make figure
f_video = figure(1);
set(f_video, 'position', [10 ,50, 1440, 480])

%% trajectory axes
ax_map = axes('Parent',f_video);
set(ax_map,'units','normalized','OuterPosition',[0,0,1/3,1]);
x_min = min([reserve_state_tar(1, :), reserve_state_obs(1, :), ...
            reserve_est_state_ba(1, :), reserve_est_state_bo(1, :)]);
x_max = max([reserve_state_tar(1, :), reserve_state_obs(1, :), ...
            reserve_est_state_ba(1, :), reserve_est_state_bo(1, :)]);
y_min = min([reserve_state_tar(2, :), reserve_state_obs(2, :), ...
            reserve_est_state_ba(2, :), reserve_est_state_bo(2, :)])-1;
y_max = max([reserve_state_tar(2, :), reserve_state_obs(2, :), ...
            reserve_est_state_ba(2, :), reserve_est_state_bo(2, :)])+7;

% plot start positions
plot(reserve_state_obs(1,1), reserve_state_obs(2,1), ...
    'LineWidth',1,'LineStyle','-', 'Color', color_obs, 'Marker', '*', 'MarkerSize', 8);
hold on
plot(reserve_state_tar(1,1), reserve_state_tar(2,1), ...
    'LineWidth',1,'LineStyle','-', 'Color', color_tar, 'Marker', '*', 'MarkerSize', 8);
hold on
plot(reserve_est_state_bo(1,1), reserve_est_state_bo(2,1), ...
    'LineWidth',1,'LineStyle','-', 'Color', color_est_t_bo, 'Marker', '*', 'MarkerSize', 8);
hold on
plot(reserve_est_state_bo(1,1), reserve_est_state_bo(2,1), ...
    'LineWidth',1,'LineStyle','-', 'Color', color_est_t_ba, 'Marker', '*', 'MarkerSize', 5);

% plot initial trajectories
hold on
handle_c_real_traj = plot(ax_map, NaN, NaN, 'LineWidth',1,'LineStyle','-', 'Color', color_obs);
hold on 
handle_t_est_b_traj = plot(ax_map, NaN, NaN, 'LineWidth',2,'LineStyle',':', 'Color', color_est_t_bo);
hold on
handle_t_est_ba_traj = plot(ax_map, NaN, NaN, 'LineWidth',2,'LineStyle',':', 'Color', color_est_t_ba);
hold on
handle_t_real_traj = plot(ax_map, NaN, NaN, 'LineWidth',1,'LineStyle','-', 'Color', color_tar);

% plot initial positions
handle_c = rectangle(ax_map, 'Position', [reserve_state_obs(1,1)-1/4,reserve_state_obs(2,1)-1/4,1/2,1/2],...
                    'Curvature',[1 1],'FaceColor',color_obs,'LineStyle','none');
hold on
theta_tem = reserve_state_tar_yaw(1, 1);
R_tem = [cos(theta_tem), -sin(theta_tem);
         sin(theta_tem), cos(theta_tem)];
rectan_points_now = R_tem * rectan_points;
rectan_points_now = rectan_points_now + reserve_state_tar(1:2, 1);
handle_t = patch(rectan_points_now(1, :), rectan_points_now(2, :), color_tar,'LineStyle','none');
hold on
handle_t_est_ba = rectangle(ax_map, ...
                    'Position',[reserve_est_state_ba(1,1)-reserve_est_state_ba(5,1)/2,reserve_est_state_ba(2,1)-reserve_est_state_ba(5,1)/2,reserve_est_state_ba(5,1),reserve_est_state_ba(5,1)],...
                    'Curvature',[1 1],'FaceColor','none','LineStyle','-', 'LineWidth',1.2, 'EdgeColor', color_est_t_ba);
hold on
handle_t_est_b = rectangle(ax_map, ...
                    'Position',[reserve_est_state_bo(1,1)-1/4,reserve_est_state_bo(2,1)-1/4,1/2,1/2],...
                    'Curvature',[1 1],'FaceColor','none','LineStyle','-', 'LineWidth',1.2, 'EdgeColor', color_est_t_bo);

handle_c_tem = plot(ax_map, NaN,NaN, 'o', 'MarkerFaceColor', color_obs, 'MarkerEdgeColor', 'none');
handle_t_tem = plot(ax_map, NaN,NaN, 'o', 'MarkerFaceColor', color_tar, 'MarkerEdgeColor', 'none');
handle_t_est_b_tem = plot(ax_map, NaN,NaN, 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', color_est_t_bo);
handle_t_est_ba_tem = plot(ax_map, NaN,NaN, 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', color_est_t_ba); 
axis equal;
length_x = x_max - x_min;
length_y = y_max - y_min;
delta_length = length_x - length_y;
if delta_length > 0
    y_min = y_min - norm(delta_length)/2;
    y_max = y_max + norm(delta_length)/2;
else
    x_min = x_min - norm(delta_length)/2;
    x_max = x_max + norm(delta_length)/2;
end

axis(ax_map, [-7.5, 7.5, 4, 19]);
title('Trajectory','FontSize', title_fontsize);
xlabel('x (m)','FontSize', title_fontsize);
ylabel('y (m)','FontSize', title_fontsize);


% legend
[leg1,objh,outh,outm]=legend(ax_map, [handle_c_real_traj, handle_t_real_traj, handle_t_est_b_traj, handle_t_est_ba_traj],...
                {'true traj of observer', 'true traj of target', 'est by bearing-only', 'est by bearing-angle'}, ...
                'position', [0.187,0.76,0.1,0.1], 'FontSize', fontsize);    
ax_legend = axes('Parent',f_video, 'OuterPosition',[0,0,1,1], 'visible', 'off');
[leg2,objh,outh,outm]=legend(ax_legend, [handle_c_tem, handle_t_tem, handle_t_est_b_tem, handle_t_est_ba_tem],...
    {'pos of observer', 'pos of target', 'est by bearing-only', 'est by bearing-angle'},...
    'FontSize', fontsize,'position', [0.058,0.76,0.1,0.1]);


%% setup error of distance axes
ax_dis = axes('Parent',f_video);
set(ax_dis,'units','normalized','OuterPosition',[1/3,0,1/3,1]);

y_max = max([reserve_est_dis_ba, reserve_est_dis_bo]);
h_dis_b = plot(ax_dis, 0, reserve_est_dis_bo(1,1), 'color', color_est_t_bo, 'LineWidth',1.5);
hold on
h_dis_ba = plot(ax_dis, 0, reserve_est_dis_ba(1,1), 'color', color_est_t_ba, 'LineWidth',1.5);
legend([h_dis_b, h_dis_ba], {'bearing-only', 'bearing-angle'}, 'FontSize', fontsize);
axis(ax_dis, [0, (total_t)*dt, 0, y_max*1.2]);
xlabel('time (s)','FontSize', title_fontsize);
ylabel('error of distance (m)','FontSize', title_fontsize);

%% setup size axes
ax_l = axes('Parent', f_video);
set(ax_l,'units','normalized','OuterPosition',[2/3,0,1/3,1]);

y_max = max([reserve_est_state_ba(5, :), target_size]);
y_min = min([reserve_est_state_ba(5, :), target_size]);
handle_ell_real = plot(ax_l, 0, reserve_state_tar_size(1, 1), 'color', color_tar, 'LineWidth',1.5);
hold on 
handle_ell = plot(0, reserve_est_state_ba(5, 1), 'color', color_est_t_ba, 'LineWidth',1.5);
axis(ax_l, [0, (total_t)*dt, 0.5, y_max*1.1]);
legend([handle_ell_real, handle_ell], {'real target size', 'estimated size'}, 'FontSize', fontsize);
title('Target Size','FontSize', title_fontsize);
xlabel('time (s)','FontSize', title_fontsize);
ylabel('Size (m)','FontSize', title_fontsize);


%% animated plot
n = speed;
for t = 1:n:(total_t-1)
    % groundtruth position 
    handle_c.Position = [reserve_state_obs(1,t)-1/4,reserve_state_obs(2,t)-1/4,1/2,1/2];
    theta_tem = reserve_state_tar_yaw(1, t);
    R_tem = [cos(theta_tem), -sin(theta_tem);
            sin(theta_tem), cos(theta_tem)];
    rectan_points_now = R_tem * rectan_points;
    rectan_points_now = rectan_points_now + reserve_state_tar(1:2, t);
    handle_t.XData = rectan_points_now(1, :);
    handle_t.YData = rectan_points_now(2, :);
    % estimated position
    handle_t_est_ba.Position = [reserve_est_state_ba(1,t)-reserve_est_state_ba(5,t)/2,reserve_est_state_ba(2,t)-reserve_est_state_ba(5,t)/2,reserve_est_state_ba(5,t),reserve_est_state_ba(5,t)];
    handle_t_est_b.Position = [reserve_est_state_bo(1,t)-est_init_tar_size/2,reserve_est_state_bo(2,t)-est_init_tar_size/2,est_init_tar_size,est_init_tar_size];
    % groundtruth trajectory
    handle_c_real_traj.XData(t:t+n-1) = reserve_state_obs(1, t:t+n-1);
    handle_c_real_traj.YData(t:t+n-1) = reserve_state_obs(2, t:t+n-1);
    handle_t_real_traj.XData(t:t+n-1) = reserve_state_tar(1, t:t+n-1);
    handle_t_real_traj.YData(t:t+n-1) = reserve_state_tar(2, t:t+n-1);
    % estimated trajectory
    handle_t_est_b_traj.XData(t:t+n-1) = reserve_est_state_bo(1, t:t+n-1);
    handle_t_est_b_traj.YData(t:t+n-1) = reserve_est_state_bo(2, t:t+n-1);
    handle_t_est_ba_traj.XData(t:t+n-1) = reserve_est_state_ba(1, t:t+n-1);
    handle_t_est_ba_traj.YData(t:t+n-1) = reserve_est_state_ba(2, t:t+n-1);
    % error of distance
    h_dis_ba.XData(t:t+n-1) = (t:t+n-1)*dt;
    h_dis_ba.YData(t:t+n-1) = reserve_est_dis_ba(1, t:t+n-1);
    h_dis_b.XData(t:t+n-1) = (t:t+n-1)*dt;
    h_dis_b.YData(t:t+n-1) = reserve_est_dis_bo(1, t:t+n-1);
    % estimated size
    handle_ell_real.XData(t:t+n-1) = (t:t+n-1)*dt;
    handle_ell_real.YData(t:t+n-1) = reserve_state_tar_size(1, t:t+n-1);
    handle_ell.XData(t:t+n-1) = (t:t+n-1)*dt;
    handle_ell.YData(t:t+n-1) = reserve_est_state_ba(5, t:t+n-1);
    % animated
    drawnow limitrate;
    pause(dt);

end










