%%Generates metric plots
close all; %close the figures of the previous run

%% Normalized speed plot
figure(1);
grid on
hold all
speed_plot = analyze_normalized_speed_plot(dt,sim_vel_list,double(sim_kk),quadNum,migSpd);

%% Normalized inter-agent distance plot
figure(2);
grid on
hold all
[interagent_distance_error_plot,interagent_data_PFM] = analyze_interagent_distance(dt,sim_pos_list,inter_agent_dist,double(sim_kk),quadNum,quad_rad_real,sense_range,quadK,d_safety_rob);

%% Order plot
figure(3);
grid on
hold all
plotAccuracy = 0; %0-1
[order_plot,order_data_PFM,accuracy_data_PFM] = analyze_order_vel(dt,sim_vel_list,double(sim_kk),quadNum,migVel,plotAccuracy);

%% Min. obstacle distance plot
figure(4);
grid on
hold all
%the whole obstacle position history is passed since the obstacles move
[obstacle_distance_plot,obstacle_distance_data_PFM] = analyze_obstacle_distance(dt,sim_pos_list,obs_pos_list_,obs_rad_list_real,double(sim_kk),quadNum,obsNum,quad_rad,d_safety_obs,obs_rad);

%% Trajectory plots
color1 = [0.4940 0.1840 0.5560]; %robot trajectory color
color2 = [0.1 1.0 0.5]; %obstacle trajectory color

%top view:
figure(5);
grid on;

for i = 1:quadNum
    hold all; plot(sim_pos_list(:,(i-1)*3+1), sim_pos_list(:,(i-1)*3+2), 'Color', color1, 'LineWidth', 1.2);
end

for i = 1:obsNum
    hold all; plot(squeeze(obs_pos_list_(i,1,1:length(sim_pos_list))), squeeze(obs_pos_list_(i,2,1:length(sim_pos_list))), 'Color', color2, 'LineWidth', 1.2);
end

set(gca,'XLim',[-2 2]) %axis limits
set(gca,'YLim',[-4 4]) %axis limits
set(gca,'fontsize', 22);
set(get(gca,'XLabel'),'String','$x [m]$','interpreter', 'latex','fontsize', 24)
set(get(gca,'YLabel'),'String','$y [m]$','interpreter', 'latex','fontsize', 24)

%side view:
figure(6);
hold all;
grid on;

for i = 1:quadNum
    hold all; plot(sim_pos_list(:,(i-1)*3+2), sim_pos_list(:,(i-1)*3+3), 'Color', color1, 'LineWidth', 1.2);
end

for i = 1:obsNum
    hold all; plot(squeeze(obs_pos_list_(i,2,1:length(sim_pos_list))), squeeze(obs_pos_list_(i,3,1:length(sim_pos_list))), '.', 'Color', color2, 'LineWidth', 1.2, 'MarkerSize', 15);
end

set(gca,'XLim',[-4 4]) %axis limits
set(gca,'YLim',[0 1.5]) %axis limits
set(gca,'fontsize', 22);
set(get(gca,'XLabel'),'String','$y [m]$','interpreter', 'latex','fontsize', 24)
set(get(gca,'YLabel'),'String','$z [m]$','interpreter', 'latex','fontsize', 24)
