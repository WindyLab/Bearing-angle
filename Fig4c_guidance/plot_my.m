
speed = 1;    % plot speed

%% some calculations 
% boundary of the estimated size
reserve_est_size_ba_upper = prctile(reserve_est_state_ba(5, :,:), 99, 3);
reserve_est_size_ba_lower = prctile(reserve_est_state_ba(5, :,:), 1, 3);

% average value of estimated states
reserve_est_state_bo = mean(reserve_est_state_bo, 3);
reserve_est_state_ba = mean(reserve_est_state_ba, 3);

% 99% boundary of the error of distance
reserve_est_dis_bo_upper = prctile(reserve_est_dis_bo, 99, 3);
reserve_est_dis_bo_lower = prctile(reserve_est_dis_bo, 1, 3);
reserve_est_dis_ba_upper = prctile(reserve_est_dis_ba, 99, 3);
reserve_est_dis_ba_lower = prctile(reserve_est_dis_ba, 1, 3);

reserve_est_dis_bo_max = max(reserve_est_dis_bo, [], 3);
reserve_est_dis_bo_min = min(reserve_est_dis_bo, [], 3);
reserve_est_dis_ba_max = max(reserve_est_dis_ba, [], 3);
reserve_est_dis_ba_min = min(reserve_est_dis_ba, [], 3);

% average error of distance
reserve_est_dis_box_bo = reserve_est_dis_bo(1, :, :);
reserve_est_dis_box_ba = reserve_est_dis_ba(1, :, :);
reserve_est_dis_bo = mean(reserve_est_dis_bo, 3);
reserve_est_dis_ba = mean(reserve_est_dis_ba, 3);

% average value of nees
reserve_nees_bo = mean(reserve_nees_bo, 3);
reserve_nees_ba = mean(reserve_nees_ba, 3);


%% setup fontsize and color
color_obs = '#EDB120';              % color of groundtruth observer 
color_tar = [0.8500 0.3250 0.0980]; % color of groundtruth target 
color_est_t_bo = '#0072BD';         % color of estimated target using bearing-only 
color_est_t_ba = '#77AC30';         % color of estimated target using bearing-angle
fontsize = 11;
title_fontsize = 13;

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
handle_c = rectangle(ax_map, 'Position', [reserve_state_obs(1,1)-1/2,reserve_state_obs(2,1)-1/2,1,1],...
                    'Curvature',[1 1],'FaceColor',color_obs,'LineStyle','none');
hold on

handle_t = rectangle(ax_map, 'Position', ...
                    [reserve_state_tar(1,1)-target_size/2, reserve_state_tar(2,1)-target_size/2,target_size,target_size],...
                    'Curvature',[1 1],'FaceColor',color_tar,'LineStyle','none');
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

axis(ax_map, [x_min, x_max, y_min, y_max]);
title('Average trajectory','FontSize', title_fontsize);
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

% plot 99% boundary
x = (1:total_t)*dt;
xconf = [x x(end:-1:1)];
yconf = [reserve_est_dis_ba_lower reserve_est_dis_ba_upper(end:-1:1)];
p = fill(xconf,yconf,'red');
p.FaceColor = [0.85 1  0.85];      
p.EdgeColor = [0.7 1  0.7];   
hold on

y_max = max([reserve_est_dis_ba, reserve_est_dis_bo]);
h_dis_b = plot(ax_dis, 0, reserve_est_dis_bo(1,1), 'color', color_est_t_bo, 'LineWidth',1.5);
hold on
h_dis_ba = plot(ax_dis, 0, reserve_est_dis_ba(1,1), 'color', color_est_t_ba, 'LineWidth',1.5);
hold on
legend([h_dis_b, h_dis_ba, p], {'bearing-only', 'bearing-angle', '99% error bounds'}, 'FontSize', fontsize);
axis(ax_dis, [0, (total_t)*dt, 0, y_max*1.1]);
xlabel('time (s)','FontSize', title_fontsize);
ylabel('average error of distance (m)','FontSize', title_fontsize);

%% setup NEES axes
ax_nees = axes('Parent', f_video);
set(ax_nees,'units','normalized','OuterPosition',[2/3,1/2,1/3,1/2]);
% plot boundary
x = (1:total_t)*dt;
xconf = [x x(end:-1:1)];
y_lower = chi2inv(0.01/2, 5*total_N)/total_N*ones(1, total_t);
y_upper = chi2inv(1-0.01/2, 5*total_N)/total_N*ones(1, total_t);
yconf = [y_lower y_upper(end:-1:1)];
p = fill(xconf,yconf,'red', 'linestyle', '--');
p.FaceColor = [1 0.85  0.85];      
p.EdgeColor = [1 0.6  0.6];  
p.LineWidth = 0.8;
hold on

x = [0, 0.1, 0.1, 0];
y = [100, 100, 100, 100];
p = fill(x,y,'red', 'linestyle', '--');
p.FaceColor = [1 0.85  0.85];      
p.EdgeColor = [1 0.6  0.6];  
p.LineWidth = 1.1;

h_ness_ba = plot(ax_nees, (0:total_t-1)*dt, reserve_nees_ba(1, 1:total_t),  'color', color_est_t_ba, 'LineWidth',1.5);
hold on
h_ness_bo = plot(ax_nees, (0:total_t-1)*dt, reserve_nees_bo(1, 1:total_t),  'color', color_est_t_bo, 'LineWidth',1.5);
hold on
y_max = max(reserve_nees_ba);
axis(ax_nees, [0, (total_t)*dt, 0, 85]);
legend([h_ness_bo, h_ness_ba, p], {'bearing-only', 'bearing-angle', '99% confidence interval'}, 'FontSize', fontsize);
xlabel('time (s)','FontSize', title_fontsize);
title('Average NEES','FontSize', title_fontsize);

%% setup size axes
ax_l = axes('Parent', f_video);
set(ax_l,'units','normalized','OuterPosition',[2/3,0,1/3,1/2]);
% boundary
x = (1:total_t)*dt;
xconf = [x x(end:-1:1)];
yconf = [reserve_est_size_ba_lower reserve_est_size_ba_upper(end:-1:1)];
p = fill(xconf,yconf,'red');
p.FaceColor = [0.85 1 0.85];      
p.EdgeColor = [0.7 1 0.7];  
hold on
y_max = max([reserve_est_state_ba(5, :), target_size]);
y_min = min([reserve_est_state_ba(5, :), target_size]);
handle_ell_real = plot(ax_l, 0:dt:total_t*dt, target_size*ones(1, total_t+1), 'color', color_tar, 'LineWidth',1.5);
hold on 
handle_ell = plot(0, reserve_est_state_ba(5, 1), 'color', color_est_t_ba, 'LineWidth',1.5);
axis(ax_l, [0, (total_t)*dt, 0.87, 1.66]);
legend([handle_ell_real, handle_ell, p], {'real target size', 'average estimated size', '99% error bounds'}, 'FontSize', fontsize);
title('Target Size','FontSize', title_fontsize);
xlabel('time (s)','FontSize', title_fontsize);
ylabel('Target size (m)','FontSize', title_fontsize);



%% animated plot
n = speed;
for t = 1:n:(total_t-1)
    % groundtruth position 
    handle_c.Position = [reserve_state_obs(1,t)-1/4,reserve_state_obs(2,t)-1/4,1/2,1/2];
    handle_t.Position = [reserve_state_tar(1,t)-target_size/2, reserve_state_tar(2,t)-target_size/2,target_size,target_size];
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
    handle_ell.XData(t:t+n-1) = (t:t+n-1)*dt;
    handle_ell.YData(t:t+n-1) = reserve_est_state_ba(5, t:t+n-1);
    % animated
    drawnow limitrate;
    pause(dt);

end







