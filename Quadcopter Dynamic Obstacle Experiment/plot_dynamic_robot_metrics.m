%Loads the VICON trajectories of the dynamic obstacle experiment and plots the metrics

clc;
clear;
close all;

quadNum = 5; %number of quadcopters
obsNum = 2; %number of obstacle quadcopters

%approximate initial positions used to identify the VICON rigid bodies,
%the first quadNum rows are the robots and the last obsNum rows are the obstacles:
ini_pos_list = [
    +0.00 -2.5 % a1
    +0.50 -2.0 % a2
    -0.50 -2.0 % a3
    -0.50 -3.0 % a4
    +0.50 -3.0 % a5
    -0.75 +0.0 % e1
    +0.75 +1.0 % e2
    ];

%% load the VICON log
filename = "vicon_logs\ralCfOnboardDynamicObstacle_01_03_24_Test_5_Trajectories_120.csv";

%time steps between which the experiment is evaluated, tuned for Test 5:
idx_start = 1100;
idx_stop = 2500;

pos_log_vicon = readmatrix(filename)/1E3; %VICON logs are in millimeters
pos_log_vicon = pos_log_vicon(4:end, 3:3*length(ini_pos_list)+2); %drop the header rows and the frame columns

%% match trajectories
%VICON does not export the rigid bodies in a fixed order, so each column
%triplet is matched to the closest initial position:
cc = 0;
for c = 1:3:length(ini_pos_list)*3
    cc = cc + 1;
    pos_list(cc, :) = pos_log_vicon(1,c+0:c+1); %first logged position of each rigid body
end

match_idx = [];
for i = 1:length(ini_pos_list)
    for j = 1:length(ini_pos_list)
        dist(j) = sqrt((pos_list(j,1)-ini_pos_list(i,1))^2+(pos_list(j,2)-ini_pos_list(i,2))^2);
    end
    [~, idx] = min(dist);
    match_idx(i) = idx; %column triplet belonging to the i-th rigid body
end

%collect the robot trajectories:
sim_pos_list_ = [];
for i = 1:quadNum
    idx = match_idx(i);
    sim_pos_list_ = [sim_pos_list_, pos_log_vicon(:, (idx-1)*3+1:(idx-1)*3+3)];
end

%collect the obstacle trajectories:
obs_pos_list_ = [];
for i = quadNum+1:quadNum+obsNum
    idx = match_idx(i);
    obs_pos_list_ = [obs_pos_list_, pos_log_vicon(:, (idx-1)*3+1:(idx-1)*3+3)];
end

%% params
dt = 0.01; %dt in seconds
migVel = [0,0.5,0]; %migration velocity
migSpd = norm(migVel); %migration speed
inter_agent_dist = 0.8; %referance inter-robot distance in meters
quad_rad = 0.1; %quadcopter radius in meters
quad_rad_real = quad_rad;
sense_range = 150; %sensing range in meters, it has taken extremely large to ensure there is no metric selection
quadK = 3; %max. number of neighbors for topological selection
obsK = 2; %max. number of obstacles for topological selection
d_safety_rob = 0.3; %robot safety distance
d_safety_obs = 0.5; %obstacle safety distance in meters
obsRad = 0.1; %obstacle radius in meters, a quadcopter is used as a moving obstacle
obs_rad = obsRad;
obs_rad_list_real = obsRad*ones(obsNum,1);

%% agents
sim_pos_list = sim_pos_list_;
%velocity calculations via discrete differentiation:
sim_vel_list = vertcat(zeros(1, 3*quadNum), (sim_pos_list(2:end,1:end) - sim_pos_list(1:end-1,1:end))./dt);

%% obstacles
obs_pos_list = obs_pos_list_;
obs_vel_list = vertcat(zeros(1, 3*obsNum), (obs_pos_list(2:end,1:end) - obs_pos_list(1:end-1,1:end))./dt);

%% select the evaluated part of the experiment
sim_kk = idx_stop-idx_start; %number of time steps that are evaluated
sim_time = sim_kk*dt;

%drop the steps in which VICON lost a rigid body:
sim_pos_list(isnan(sum(sim_pos_list'))', :) = [];
sim_vel_list(isnan(sum(sim_vel_list'))', :) = [];
obs_pos_list(isnan(sum(obs_pos_list'))', :) = [];
obs_vel_list(isnan(sum(obs_vel_list'))', :) = [];

sim_pos_list = sim_pos_list(idx_start:idx_stop, :);
sim_vel_list = sim_vel_list(idx_start:idx_stop, :);

obs_pos_list = obs_pos_list(idx_start:idx_stop, :);
obs_vel_list = obs_vel_list(idx_start:idx_stop, :);

%%Generates metric plots
%% Normalized speed plot
figure;
grid on
hold on
speed_plot = analyze_normalized_speed_plot(dt,sim_vel_list,double(sim_kk),quadNum,migSpd);

%% Normalized inter-agent distance plot
figure;
grid on
hold on
[interagent_distance_error_plot,interagent_data_PFM] = analyze_interagent_distance(dt,sim_pos_list,inter_agent_dist,double(sim_kk),quadNum,quad_rad_real,sense_range,quadK,d_safety_rob);

%% Order plot
figure;
grid on
hold on
plotAccuracy = 0; %0-1
[order_plot,order_data_PFM,accuracy_data_PFM] = analyze_order_vel(dt,sim_vel_list,double(sim_kk),quadNum,migVel,plotAccuracy);

%% Min. obstacle distance plot
figure;
grid on
hold on
%reshape the obstacle positions as obstacle x coordinate x step since the obstacles move
obs_pos_list_ = permute(reshape(obs_pos_list', [3, obsNum, size(obs_pos_list,1)]), [2 1 3]);
[obstacle_distance_plot,obstacle_distance_data_PFM] = analyze_obstacle_distance(dt,sim_pos_list,obs_pos_list_,obs_rad_list_real,double(sim_kk),quadNum,obsNum,quad_rad,d_safety_obs,obs_rad);

%% Trajectory plots
color1 = [0.4940 0.1840 0.5560]; %robot trajectory color
color2 = [0.1 1.0 0.5]; %obstacle trajectory color

%top view:
figure;
grid on;

for i = 1:quadNum
    hold on; plot(sim_pos_list(:,(i-1)*3+1), sim_pos_list(:,(i-1)*3+2), 'Color', color1, 'LineWidth', 1.2);
end

for i = 1:obsNum
    hold on; plot(obs_pos_list(:,(i-1)*3+1), obs_pos_list(:,(i-1)*3+2), 'Color', color2, 'LineWidth', 1.2);
end

set(gca,'XLim',[-2 2]) %axis limits
set(gca,'YLim',[-4 4]) %axis limits
set(gca,'fontsize', 22);
set(get(gca,'XLabel'),'String','$x [m]$','interpreter', 'latex','fontsize', 24)
set(get(gca,'YLabel'),'String','$y [m]$','interpreter', 'latex','fontsize', 24)

%side view:
figure;
grid on;

for i = 1:quadNum
    hold on; plot(sim_pos_list(:,(i-1)*3+2), sim_pos_list(:,(i-1)*3+3), 'Color', color1, 'LineWidth', 1.2);
end

for i = 1:obsNum
    hold on; plot(obs_pos_list(:,(i-1)*3+2), obs_pos_list(:,(i-1)*3+3), 'Color', color2, 'LineWidth', 1.2);
end

set(gca,'XLim',[-4 4]) %axis limits
set(gca,'YLim',[0 1.5]) %axis limits
set(gca,'fontsize', 22);
set(get(gca,'XLabel'),'String','$y [m]$','interpreter', 'latex','fontsize', 24)
set(get(gca,'YLabel'),'String','$z [m]$','interpreter', 'latex','fontsize', 24)
