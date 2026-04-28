clear
clc

real = 1;
%method and experiment parameters:
dt = 0.05; %dt in seconds
quadNum = 5; %number of quadcopters
obsNum = 5; %number of obstacles
quadK = 3; %max. number of neighbors for topological selection
obsK = 2; %max. number of obstacles for topological selection
quad_rad = 0.1; %quadcopter radius in meters
obsRad = 0.30; %obstacle radius in meters
obs_rad = obsRad;
obs_rad_list_real = obsRad*ones(obsNum,1);
migVel = [0,0.5,0]; %migration velocity
migSpd = norm(migVel); %migration speed
inter_agent_dist = 0.8; %referance inter-robot distance in meters
sense_range = 150; %sensing range in meters, it has taken extremely large to ensure there is no metric selection
quad_rad_real = quad_rad;
d_safety_rob = 0.3; %robot safety distance
d_safety_obs = 0.5; %obstacle safety distance
obs_mode = 1;

%%
z_pos_obs = 2;
%obstacle positions:
obs_pos_list = [-1.50, -0.34, 0.75;-0.35, 0.90, 0.75;-1.49, 2.19,0.75;0.75, -0.35, 0.75;0.75, 2.17, 0.75];
obs_pos_list = obs_pos_list + [0, 4, 0];

%loads vicon logs:
sim_pos_list = load('pos_log_vicon.mat','pos_log_sim');
start_index = 215; %time step when the experiment starts
sim_kk = 241; %number of time steps till the experiment is completed

%Position calculations:
sim_pos_list = sim_pos_list.pos_log_sim;
sim_pos_list = sim_pos_list(start_index:end,:);
sim_pos_list = sim_pos_list(1:sim_kk,:);
sim_pos_list = sim_pos_list + repmat([0, 4, 0],1,quadNum);

%%Velocity calculations via discrete differentiation:
sim_vel_list_l = zeros(1,15) + repmat([0, 0, 0],1,quadNum);
sim_vel_list = vertcat(sim_vel_list_l, (sim_pos_list(2:end,1:end) - sim_pos_list(1:end-1,1:end))./dt);

sim_kk = sim_kk;
sim_time = sim_kk*dt;



