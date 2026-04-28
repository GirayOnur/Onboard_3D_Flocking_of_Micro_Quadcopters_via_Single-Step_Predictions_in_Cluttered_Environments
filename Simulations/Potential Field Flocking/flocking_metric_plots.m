%%Generate metric plots
%% Normalized speed plot
grid on
hold on
speed_plot = analyze_normalized_speed_plot(dt,sim_vel_list,double(sim_kk),quadNum,migSpd);

%% Normalized inter-agent distance plot
grid on
hold on
[interagent_distance_error_plot,interagent_data_PFM] = analyze_interagent_distance(dt,sim_pos_list,inter_agent_dist,double(sim_kk),quadNum,quad_rad_real,sense_range,quadK);

%% Order plot
grid on
hold on
plotAccuracy = 0; %0-1
[order_plot,order_data_PFM,accuracy_data_PFM] = analyze_order_vel(dt,sim_vel_list,double(sim_kk),quadNum,migVel,plotAccuracy);

%% Min. obstacle distance plot
grid on
hold on

[obstacle_distance_plot,obstacle_distance_data_PFM] = analyze_obstacle_distance(dt,sim_pos_list,obs_pos_list,obs_rad_list_real,double(sim_kk),quadNum,obsNum,quad_rad,obs_rad);

