%Runs a flocking simulation using the Multi-step Predictive Beam Search method

% clc
clearvars -except scale obs_density stepNum Nbest stepNumArr NbestArr rand_pos_num RepeatCount Repeat_i stepNum_i Nbest_i

%% Simulation parameters
% scale = 1; %1 -> 5 robots, 2 -> 20 robots, 3 -> 80 robots
obs_mode = 1;
% obs_density = 1; % 1 -> 0.06m^-2, 2 -> 0.12m^-2, 3 -> 0.20m^-2
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
% stepNum = 10; %number of predicted steps
% Nbest = 2; %beam width for multi-step search
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
% rand_pos_num = 5; %random seed for initial positions
rand_pos_obs_num = rand_pos_num + 10;

%% Candidate accelerations (search space)
accel_list = maxAcc.*[1, 0, 0;   %positive x-direction
    0 ,1, 0;                     %positive y-direction
    -1, 0 ,0;                    %negative x-direction
    0, -1, 0;                    %negative y-direction
    0, 0, 1;                     %positive z-direction
    0, 0, -1;                    %negative z-direction
    0, 0, 0];                    %no acceleration

nodeNum = length(accel_list); %number of candidate nodes in search tree

%% Finish line and robot numbers for different scales
if scale == 1
    y_final = 10; %finish line for 5 robots
    quad_num = 5;
elseif scale == 2
    quad_num = 20;
    y_final = 18; %finish line for 20 robots
else
    quad_num = 80;
    y_final = 34; %finish line for 80 robots
end

%% Initialize positions of robots and obstacles
quad_pos_list_i = quad_pos_rand_scalable(rand_pos_num,z_pos,scale); %initial robot positions
quad_velvel_list_i = zeros(quad_num,3); %initial robot velocities (all zero)

obs_pos_list = rand_obs_with_scale(scale,obs_density,rand_pos_obs_num); %random obstacle positions
obs_num = length(obs_pos_list(:,1)); %number of obstacles
obs_pos_list(:,3) = z_pos_obs*ones(obs_num,1); %set obstacle altitude

obs_rad_list_real = obs_rad*ones(obs_num,1); %obstacle radii
obs_rad_list = obs_rad_list_real;

quadNum = length(quad_pos_list_i(:,1)); %number of robots
obsNum = length(obs_pos_list(:,1)); %number of obstacles

quad_head_list_alpha_i = zeros(quadNum,1) + migAng; %initial heading angles
quad_head_vec_list_i = zeros(quadNum,3) + [cos(migAng), sin(migAng), 0]; %initial heading vectors
quad_spd_list_i = zeros(quadNum,1) + 0.0; %initial speeds

velocity_command_list = zeros(quadNum,3); %velocity command storage
position_command_list = zeros(quadNum,3); %position command storage

%% Simulation data storage
sim_pos_list(1,:) = reshape(quad_pos_list_i.',1,[]); %initial positions
sim_vel_list(1,:) = reshape(quad_velvel_list_i.',1,[]); %initial velocities
sim_head_list(1,:) = reshape(quad_head_list_alpha_i.',1,[]); %initial headings
sim_spd_list(1,:) = reshape(quad_spd_list_i.',1,[]); %initial speeds

%% Predictive search setup
% tic
quad_pos_list = zeros(quadNum,3*(stepNum+1)); %matrix for predicted positions
mission_not_completed = 1; %flag for mission completion
kk = int16(2); %simulation step counter

while mission_not_completed
    quad_pos_list(:,1:3) = quad_pos_list_i; %set current positions
    m=1;

    %predict neighbor positions assuming migration velocity
    for i=4:3:(3*stepNum+1)
        quad_pos_list(:,i:i+2) = quad_pos_list_i + m*dt*migVel;
        m=m+1;
    end

    %% Main loop: compute velocity input for each robot
    for i=1:quadNum
        %remove current robot from neighbor list
        quad_pos_list_neigs = quad_pos_list;
        quad_pos_list_neigs(i,:) = [];

        %compute relative distances to neighbors
        quad_neig_rel_dist_list = vecnorm(quad_pos_list_neigs(:,1:3) - quad_pos_list_i(i,:),2,2);
        quad_neig_rel_dist_pos_list = horzcat(quad_neig_rel_dist_list,quad_pos_list_neigs);
        quad_neig_rel_dist_pos_list = sortrows(quad_neig_rel_dist_pos_list,1);

        %select closest quadK neighbors within sensing range
        if length(quad_neig_rel_dist_pos_list(:,1)) > quadK
            quad_neig_rel_dist_pos_list = quad_neig_rel_dist_pos_list(1:quadK,:);
        end
        quad_neig_rel_dist_pos_list = quad_neig_rel_dist_pos_list(quad_neig_rel_dist_pos_list(:,1)<sense_range,:);
        quadKK = length(quad_neig_rel_dist_pos_list(:,1));

        %compute relative distances to obstacles
        quad_obs_rel_dist_list = vecnorm([obs_pos_list(:,1:2), zeros(obs_num,1)] - [quad_pos_list_i(i,1:2), 0],2,2);
        quad_obs_rel_dist_pos_list = horzcat(quad_obs_rel_dist_list,obs_pos_list);
        quad_obs_rel_dist_pos_list = sortrows(quad_obs_rel_dist_pos_list,1);

        %select closest obsK obstacles within sensing range
        if length(quad_obs_rel_dist_pos_list(:,1)) > obsK
            quad_obs_rel_dist_pos_list = quad_obs_rel_dist_pos_list(1:obsK,:);
        end
        quad_obs_rel_dist_pos_list = quad_obs_rel_dist_pos_list(quad_obs_rel_dist_pos_list(:,1)<sense_range,:);
        obs_detected_point_list = quad_obs_rel_dist_pos_list(:,2:end);
        obsKK = length(obs_detected_point_list(:,1));

        quad_vel_vec_i = quad_velvel_list_i(i,:); %current velocity
        quad_pos_list_fstep = quad_neig_rel_dist_pos_list(:,5:7); %predicted neighbor positions

        %% Multi-step beam search
        predicted_vel0 = quad_vel_vec_i; %initial velocity
        predicted_spd0 = norm(predicted_vel0); %initial speed

        %apply speed limits
        if predicted_spd0 < biasSpd
            predicted_spd0 = biasSpd;
        elseif predicted_spd0 > maxSpd
            predicted_spd0 = maxSpd;
        end

        %compute heading
        if predicted_spd0 ~= 0
            predicted_head0 = predicted_vel0./predicted_spd0;
        else
            predicted_head0 = [0,1,0]; %default heading
        end

        predicted_vel0 = predicted_spd0 .* predicted_head0;
        predicted_pos0 = quad_pos_list_i(i,:);

        prev_search_arr = [0, predicted_vel0, predicted_pos0]; %initialize search array

        %iterate over prediction steps
        predicted_vel_0 = [0, 0, 0];
        predicted_pos_0 = [0, 0, 0];

        for step_i = 1:stepNum
            search_arr = [];
            base_col = 5 + (step_i-1)*3; %dynamic column selection
            quad_pos_list_nstep = quad_neig_rel_dist_pos_list(:, base_col : base_col+2);

            %expand search tree
            for ni = 1:size(prev_search_arr,1)
                heur_prev = prev_search_arr(ni,1); %previous heuristic cost
                vel_prev  = prev_search_arr(ni,2:4); %previous velocity
                pos_prev  = prev_search_arr(ni,5:7); %previous position

                %apply speed limits to previous state
                spd_prev = norm(vel_prev);
                if spd_prev < biasSpd
                    spd_prev = biasSpd;
                elseif spd_prev > maxSpd
                    spd_prev = maxSpd;
                end

                %compute heading
                if spd_prev ~= 0
                    head_prev = vel_prev./spd_prev;
                else
                    head_prev = [0,1,0];
                end

                %try all candidate accelerations
                for n = 1:length(accel_list)
                    search_accel = accel_list(n,:); %candidate acceleration

                    %predict velocity and apply limits
                    predicted_vel = vel_prev + search_accel*dt;
                    predicted_spd = norm(predicted_vel);
                    if predicted_spd < biasSpd
                        predicted_spd = biasSpd;
                    elseif predicted_spd > maxSpd
                        predicted_spd = maxSpd;
                    end

                    %compute heading
                    if predicted_spd ~= 0
                        predicted_head_vec = predicted_vel./predicted_spd;
                    else
                        predicted_head_vec = [0,1,0];
                    end

                    predicted_vel = predicted_spd .* predicted_head_vec;
                    predicted_pos = pos_prev + dt*predicted_spd*predicted_head_vec;

                    %% heuristic cost calculations
                    %inter-robot distance cost
                    quad_dist_vec_list = quad_pos_list_nstep - predicted_pos;
                    quad_dist_mag_list = vecnorm(quad_dist_vec_list,2,2);
                    heur = heur_prev + quadCost(quad_dist_mag_list,quadKK,inter_agent_dist,d_safety_rob);

                    %obstacle avoidance cost
                    obs_dist_vec_list = [obs_detected_point_list(:,1:2),zeros(obsKK,1)] - [predicted_pos(1:2),0];
                    obs_dist_mag_list = vecnorm(obs_dist_vec_list,2,2);
                    obs_approach_list = (obs_dist_vec_list./obs_dist_mag_list)*predicted_head_vec';
                    obs_approach_list(obs_approach_list<0) = 0;
                    heur = heur + obsCost(obs_dist_mag_list,obs_approach_list,obsKK,sense_range,d_safety_obs);

                    %migration cost
                    heur = heur + migCost(migAng,migSpd,predicted_spd,predicted_head_vec);

                    %altitude cost
                    heur = heur + heightCost(predicted_pos(3),des_height,height_margin);

                    %store candidate
                    if step_i == 1
                        search_arr(end+1,:) = [heur, predicted_vel, predicted_pos, predicted_vel, predicted_pos];
                    else
                        search_arr(end+1,:) = [heur, predicted_vel, predicted_pos, prev_search_arr(ni, 8:10), prev_search_arr(ni, 11:13)];
                    end

                    % search_arr(end+1,:) = [heur, predicted_vel, predicted_pos, predicted_vel_0, predicted_pos_0];
                end
            end

            %sort candidates by heuristic cost
            search_arr = sortrows(search_arr,1);
            %keep best quadK candidates for next step
            prev_search_arr = search_arr(1:min(Nbest,size(search_arr,1)),:);
        end

        %select best candidate after beam search
        search_arr = prev_search_arr;
        vel_input = search_arr(1,8:10);
        pos_input = search_arr(1,11:13);

        %update robot state
        quad_spd_list_i(i) = norm(vel_input);
        quad_head_vec_input = vel_input./norm(vel_input);
        quad_head_vec_list_i(i,:) = quad_head_vec_input;
        velocity_command_list(i,:) = vel_input;
        position_command_list(i,:) = pos_input;
    end

    %% Update states of all robots
    quad_velvel_list_i = velocity_command_list; %update velocities
    velocity_command_list_noisy = velocity_command_list + sigma_d*randn(quadNum,3); %add noise
    quad_pos_list_i = quad_pos_list_i + dt*velocity_command_list_noisy; %update positions

    %store simulation history
    sim_pos_list(kk,:) = reshape(quad_pos_list_i.',1,[]);
    sim_vel_list(kk,:) = reshape(velocity_command_list_noisy.',1,[]);
    sim_head_list(kk,:) = reshape(quad_head_list_alpha_i.',1,[]);
    sim_spd_list(kk,:) = reshape(quad_spd_list_i.',1,[]);

    kk = kk +1; %increment simulation step

    %mission completion check: all robots must cross y_final
    mission_not_completed = any(quad_pos_list_i(:,2) < y_final);
end

% toc

%% Simulation results
sim_kk = double(kk)-1; %number of simulation steps
sim_time = (sim_kk-1)*dt; %total simulation time
% disp(sim_time) %display mission completion time
