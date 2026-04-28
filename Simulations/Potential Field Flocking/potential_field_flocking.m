%Runs a simulation of the flocking using the Potential Field method

clc
clear

scale = 1; %1 -> 5 robots, 2 -> 20 robots, 3 -> 80 robots
obs_mode = 1;
obs_density = 3; % 1 -> 0.06m^-2, 2 -> 0.12m^-2, 3 -> 0.20m^-2
des_height = 0.75; %reference altitude
z_pos_obs = des_height;
z_pos = des_height;
obs_rad = 0.35; %obstacle radius in meters
dt = 0.01; %dt in seconds
quad_rad_real = 0.1; %robot radius in meters
quad_rad = quad_rad_real;
quadK = 3; %maximum number of considered neighbors for the topological selection
inter_agent_dist = 0.8; %desired inter-robot distance in meters
migAng = pi/2; %migration direction angle
migSpd = 0.5; %migration speed
migVel = migSpd*[cos(migAng), sin(migAng), 0]; %migration velocity
sense_range = 150.0; %sensing range in meters, it has taken as very large value to ensure there is no metric selection
sigma_d = 0.02; %standart deviation of velocity noise
rand_pos_num = 5; %random seed for initial positions
rand_pos_obs_num = rand_pos_num + 10;

%% inital positions of quads and obstacles

%finish lines and robot numbers for different scales:
if scale == 1
    quad_num = 5;
    y_final = 10; %8+2
elseif scale == 2
    quad_num = 20;
    y_final = 18; %2*8+2 
else
    quad_num = 80;
    y_final = 34;%4*8+2
end


quad_pos_list_i = quad_pos_rand_scalable(rand_pos_num,z_pos,scale); %robot positions

quad_velvel_list_i = zeros(quad_num,3); %robot velocities

obs_pos_list = rand_obs_with_scale(scale,obs_density,rand_pos_obs_num); %obstacle positions

obs_num = length(obs_pos_list(:,1));

obs_pos_list(:,3) = z_pos_obs*ones(obs_num,1);

obs_rad_list_real = obs_rad*ones(obs_num,1); %obstacle radii

obs_rad_list = obs_rad_list_real;
quadNum = length(quad_pos_list_i(:,1)); %number of robots
obsNum = length (obs_pos_list(:,1)); %number of obstacles

quad_head_list_alpha_i = zeros(quadNum,1) + migAng;
quad_head_vec_list_i = zeros(quadNum,3) + [cos(migAng), sin(migAng), 0];
quad_spd_list_i = zeros(quadNum,1) + 0.0;

velocity_command_list = zeros(quadNum,3); %velocity control command list
position_command_list = zeros(quadNum,3); %position control command list


%% sim params & data storage arrays
sim_pos_list(1,:) = reshape(quad_pos_list_i.',1,[]); %position vector history
sim_vel_list(1,:) = reshape(quad_velvel_list_i.',1,[]); %velocity vector history
sim_head_list(1,:) = reshape(quad_head_list_alpha_i.',1,[]); %heading history
sim_spd_list(1,:) = reshape(quad_spd_list_i.',1,[]); %speed history


run param_swarm.m %run swarm parameters for the Potential Field method

p_swarm.nb_agents = quadNum;
p_swarm.cylinders = [obs_pos_list(:,1:2), obs_rad_list_real]';
p_swarm.n_cyl = length(p_swarm.cylinders(1, :));

run param_vasarhelyi.m %run parameters of the Potential Field method


%% determine linear position predictions for searching:
tic

mission_not_completed = 1; %becomes 0 if all agents complete the task

kk = int16(2); %number of discrete steps passed

while mission_not_completed

    velocity_command_list = compute_vel_vasarhelyi(p_swarm,quad_rad,dt,quad_pos_list_i',quad_velvel_list_i',quadNum); %calculate velocities using the Potential Field method
    quad_velvel_list_i = velocity_command_list';
    velocity_command_list_noisy = quad_velvel_list_i + sigma_d*randn(quadNum,3);  %add 3D noise to the velocities
    quad_pos_list_i = quad_pos_list_i + dt*velocity_command_list_noisy; %update positions of the all agents with the noisy velocities
    sim_pos_list(kk,:) = reshape(quad_pos_list_i.',1,[]); %save position history of the agents
    sim_vel_list(kk,:) = reshape(velocity_command_list_noisy.',1,[]); %save velocity history of the agents 
    sim_head_list(kk,:) = reshape(quad_head_list_alpha_i.',1,[]); %save heading history of the agents 
    sim_spd_list(kk,:) = reshape(quad_spd_list_i.',1,[]); %save speed history of the agents 

    %disp(quad_pos_list_i)
    kk = kk +1; %increase the simulation time step by one
    mission_not_completed = any(quad_pos_list_i(:,2) < y_final); %if all the robots pass y_f, mission is completed
end
toc

sim_kk = double(kk)-1;
sim_time = (sim_kk-1)*dt; %calculating the simulation time for animation and analysis functions

disp(sim_time) %show mission completion time