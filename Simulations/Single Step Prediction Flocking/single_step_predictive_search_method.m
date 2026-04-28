%Runs a flocking simulation using the Single-step Predictive Search method

clc
clear

scale = 1; %1 -> 5 robots, 2 -> 20 robots, 3 -> 80 robots
obs_mode = 1;
obs_density = 1; % 1 -> 0.06m^-2, 2 -> 0.12m^-2, 3 -> 0.20m^-2
des_height = 0.75; %reference altitude
height_margin = 0.5; %altitude margin
z_pos_obs = des_height;
z_pos = des_height;
obs_rad = 0.35; %obstacle radius in meters
dt = 0.05; %dt in seconds
quad_rad_real = 0.1; %robot radius in meters
quad_rad = quad_rad_real;
d_safety_obs = 0.5; %obstacle safety distance in meters
d_safety_rob = 0.3; %robot safety distance in meters
stepNum = 1; %number of predicted steps
quadK = 3; %maximum number of considered neighbors for the topological selection
obsK = 2; %maximum number of considered obstacles for the topological selection
inter_agent_dist = 0.8; %desired inter-robot distance in meters
migAng = pi/2; %migration direction angle
migSpd = 0.5; %migration speed
migVel = migSpd*[cos(migAng), sin(migAng), 0]; %migration velocity
maxSpd = 5; %max. speed
biasSpd = 0.05; %min. speed
maxAcc = 2; %max. acceleration
sense_range = 150.0; %sensing range in meters, it has taken as very large value to ensure there is no metric selection
sigma_d = 0.02; %standart deviation of velocity noise
rand_pos_num = 5; %random seed for initial positions
rand_pos_obs_num = rand_pos_num + 10;

%predicted accelerations where a_\alpha selected as a_max:
accel_list = maxAcc.*[1, 0, 0; 
               0 ,1, 0;
              -1, 0 ,0;
               0, -1, 0;
               0, 0, 1;
               0, 0, -1;
               0, 0, 0];

nodeNum = length(accel_list); %number of nodes at the next time step

%finish lines and robot numbers for different scales:
if scale == 1
    y_final = 10; %8+2
    quad_num = 5;
elseif scale == 2
    quad_num = 20;
    y_final = 18; %2*8+2 
else
    quad_num = 80;
    y_final = 34;%4*8+2
end

%% inital positions of quads and obstacles

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


%% determine linear position predictions for searching:
tic

quad_pos_list = zeros(quadNum,3*(stepNum+1)); %matrix to keep position estimations for the future steps

mission_not_completed = 1; %becomes 0 if all agents complete the task

kk = int16(2); %number of discrete steps passed

while mission_not_completed
    quad_pos_list(:,1:3) = quad_pos_list_i; %get current positions as the initial positions
    m=1;

    %calculate the future positions of the neighbors assuming that neighbors will move with
    %the migration velocity
    for i=4:3:(3*stepNum+1)
        quad_pos_list(:,i:i+2) = quad_pos_list_i+ m*dt*migVel;
        m=m+1;
    end

    %start seaching to determine the next velocity input of the robot
    for i=1:quadNum
        quad_pos_list_neigs = quad_pos_list;
        quad_pos_list_neigs(i,:) = [];
        quad_neig_rel_dist_list = vecnorm(quad_pos_list_neigs(:,1:3) - quad_pos_list_i(i,:),2,2); %relative neighbor distances
        quad_neig_rel_dist_pos_list = horzcat(quad_neig_rel_dist_list,quad_pos_list_neigs);
        quad_neig_rel_dist_pos_list = sortrows(quad_neig_rel_dist_pos_list,1); %sorted relative neighbor distances
        
        %consider closest quadK neighbors due to topological selection
        if length(quad_neig_rel_dist_pos_list(:,1)) > quadK
            quad_neig_rel_dist_pos_list = quad_neig_rel_dist_pos_list(1:quadK,:);
        end
        %consider the neighbors in sensing range
        quad_neig_rel_dist_pos_list = quad_neig_rel_dist_pos_list(quad_neig_rel_dist_pos_list(:,1)<sense_range,:);
        
        quadKK = length(quad_neig_rel_dist_pos_list(:,1)); %number of considered neighbors

        quad_obs_rel_dist_list = vecnorm([obs_pos_list(:,1:2), zeros(obs_num,1)] - [quad_pos_list_i(i,1:2), 0],2,2); %relative obstacle distances
        quad_obs_rel_dist_pos_list = horzcat(quad_obs_rel_dist_list,obs_pos_list);
        quad_obs_rel_dist_pos_list = sortrows(quad_obs_rel_dist_pos_list,1); %sorted relative obstacle distances

        %consider closest obsK obstacles due to topological selection
        if length(quad_obs_rel_dist_pos_list(:,1)) > obsK
            quad_obs_rel_dist_pos_list = quad_obs_rel_dist_pos_list(1:obsK,:);
        end
        %consider the obstacles in sensing range
        quad_obs_rel_dist_pos_list = quad_obs_rel_dist_pos_list(quad_obs_rel_dist_pos_list(:,1)<sense_range,:);
        
        obs_detected_point_list = quad_obs_rel_dist_pos_list(:,2:end);

        obsKK = length(obs_detected_point_list(:,1)); %number of considered obstacles
        
        quad_vel_vec_i = quad_velvel_list_i(i,:); %current velocity of the robot

        
        quad_pos_list_fstep = quad_neig_rel_dist_pos_list(:,5:7); %predicted positions of neighbors at the next step

        search_arr = zeros(nodeNum,7); %main array that keeps heuristic values, future velocity and position states
        
        search_index_counter = 1;  
        for n=1:length(accel_list)

            search_accel = accel_list(n,:); %predicted acceleration
            
            predicted_vel = quad_vel_vec_i + search_accel*dt; %predicted velocity without checking speed limits
            predicted_spd = norm(predicted_vel); %predicted speed without checking speed limits
            
            %apply speed limits
            if predicted_spd<biasSpd
                predicted_spd = biasSpd;
            elseif predicted_spd>maxSpd
                predicted_spd = maxSpd;
            else
                %pass
            end
            
            predicted_head_vec = predicted_vel./predicted_spd; %predicted heading
            
            %predicted speed is zero, then take predicted heading as in the
            %migration direction
            if predicted_head_vec==[0,0,0]
                predicted_head_vec = [0,1,0];
            end

            predicted_vel = predicted_spd.*predicted_head_vec; %predicted velocity after limiting the speed

            predicted_pos = quad_pos_list_i(i,:) + dt*predicted_spd*predicted_head_vec; %predicted position

            % calculate inter-robot distance heuristic:
            quad_dist_vec_list =  quad_pos_list_fstep - predicted_pos; %predicted relative neighbor positions
            quad_dist_mag_list = vecnorm(quad_dist_vec_list,2,2); %predicted relative neighbor distances
            quad_dist_relmag_list = quad_dist_mag_list;

            heur = quadCost(quad_dist_relmag_list,quadKK,inter_agent_dist,d_safety_rob); %calculates the inter-robot distance heuristic
            
            % calculate obstacle avoidance heuristic:
            obs_dist_vec_list =  [obs_detected_point_list(:,1:2),zeros(obsKK,1)] - [predicted_pos(1:2),0]; %predicted relative obstacle positions
            obs_dist_mag_list = vecnorm(obs_dist_vec_list,2,2); %predicted relative obstacle distances
            obs_dist_relmag_list = obs_dist_mag_list;
            %calculate predicted approach terms:
            obs_approach_list = (obs_dist_vec_list./obs_dist_mag_list)*predicted_head_vec';
            obs_approach_list(obs_approach_list<0) = 0; 
            heur = heur + obsCost(obs_dist_relmag_list,obs_approach_list,obsKK,sense_range,d_safety_obs); %calculate obstacle avoidance heuristic and add it to the total cost     
              
            heur = heur + migCost(migAng,migSpd,predicted_spd,predicted_head_vec); %calculate the migration heuristic and add it to the total cost

            heur = heur + heightCost(predicted_pos(3),des_height,height_margin); %calculate the altitude heuristic and add it to the total cost

            search_arr(search_index_counter,:) = [heur,predicted_vel, predicted_pos]; %add the heuristic cost, predicted velocity and predicted position to the search array as a node
            search_index_counter = search_index_counter + 1;
        end

        search_arr = sortrows(search_arr,1); %sort nodes based on their costs
        vel_input = search_arr(1,2:4); %select the best velocity with the smallest cost
        pos_input =  search_arr(1,5:7);
           
        quad_spd_list_i(i) = norm(vel_input); %next speed of the robot
        quad_head_vec_input = vel_input./norm(vel_input); %next heading of the robot
        quad_head_vec_list_i(i,:) = quad_head_vec_input;
        velocity_command_list(i,:) = vel_input;
        position_command_list(i,:) = pos_input;
    end
    quad_velvel_list_i = velocity_command_list;
    velocity_command_list_noisy = velocity_command_list + sigma_d*randn(quadNum,3); %add 3D noise to the velocities
    quad_pos_list_i = quad_pos_list_i + dt*velocity_command_list_noisy; %update positions of the all agents with the noisy velocities
    sim_pos_list(kk,:) = reshape(quad_pos_list_i.',1,[]); %save position history of the agents 
    sim_vel_list(kk,:) = reshape(velocity_command_list_noisy.',1,[]); %save velocity history of the agents 
    sim_head_list(kk,:) = reshape(quad_head_list_alpha_i.',1,[]); %save heading history of the agents 
    sim_spd_list(kk,:) = reshape(quad_spd_list_i.',1,[]); %save speed history of the agents 

    kk = kk +1; %increase the simulation time step by one
    mission_not_completed = any(quad_pos_list_i(:,2) < y_final); %if all the robots pass y_f, mission is completed
end
toc

sim_kk = double(kk)-1;
sim_time = (sim_kk-1)*dt; %calculate the simulation time for animation and analysis functions

disp(sim_time) %display mission completion time