%generate random initial positions for obstacles based on the scale and obstacle density:
%obs_density = 1 -> 0.06m^-2
%obs_density = 2 -> 0.12m^-2
%obs_density = 3 -> 0.20m^-2
function [obs_pos_list] = rand_obs_with_scale(scale,obs_density,rand_pos_obs_num)

map_size = 8;
bulding_width = 0.7;

if obs_density == 1
    nb_blocks_i = 3;
elseif obs_density == 2
    nb_blocks_i = 4;
elseif obs_density == 3
    nb_blocks_i = 5;
else
    %pass
end

obs_pos_list = [];

if scale == 1
    map = generate_map([], map_size, nb_blocks_i, bulding_width, rand_pos_obs_num,obs_density,1);
    
    nb_obstacles = length(map.buildings_east);
    cylinder_radius = map.building_width / 2;
    
    cylinders = [
            map.buildings_east'; % x_obstacle
            map.buildings_north'; % y_obstacle
            repmat(cylinder_radius, 1, nb_obstacles)]';
    
    obs_pos_list = cylinders;
elseif scale == 2
    if obs_density == 1
        obs_x_shift1 = 1.715;
        obs_x_shift2 = -0.94833;
    elseif obs_density == 2
        obs_x_shift1 = 1.2;
        obs_x_shift2 = -0.8;
    else
        obs_x_shift1 = 0.89;
        obs_x_shift2 = -0.71;        
    end
    obs_shift_list = [-(map_size/2)+obs_x_shift1,0,0;
                      -(map_size/2)+obs_x_shift1,map_size,0;
                        map_size/2+obs_x_shift2,0,0;
                        map_size/2+obs_x_shift2,map_size,0];
    for ii = 1:4
        map = generate_map([], map_size, nb_blocks_i, bulding_width, rand_pos_obs_num,obs_density,ii);
        nb_obstacles = length(map.buildings_east);
        cylinder_radius = map.building_width / 2;
        
        cylinders_ii = [
                map.buildings_east'; % x_obstacle
                map.buildings_north'; % y_obstacle
                repmat(cylinder_radius, 1, nb_obstacles)]';
        
        obs_pos_list_ii = cylinders_ii + obs_shift_list(ii,:);
        obs_pos_list = [obs_pos_list;obs_pos_list_ii];
    end


else

    if obs_density == 1
        obs_x_shift1 = 1.715;
        obs_x_shift2 = -0.94833;
        obs_x_shift3 = 2.666;
        obs_x_shift4 = -2*2.666;
    elseif obs_density == 2
        obs_x_shift1 = 1.2;
        obs_x_shift2 = -0.8;
        obs_x_shift3 = 2;
        obs_x_shift4 = -2*2;
    else
        obs_x_shift1 = 0.89;
        obs_x_shift2 = -0.71;
        obs_x_shift3 = 1.6;
        obs_x_shift4 = -2*1.6;
    end
    obs_shift_list = [-(3*map_size/2)+obs_x_shift1+obs_x_shift3,0,0;
                      -(3*map_size/2)+obs_x_shift1+obs_x_shift3,map_size,0;
                      -(3*map_size/2)+obs_x_shift1+obs_x_shift3,2*map_size,0;
                      -(3*map_size/2)+obs_x_shift1+obs_x_shift3,3*map_size,0;
                      -(map_size/2)+obs_x_shift1,0,0;
                      -(map_size/2)+obs_x_shift1,map_size,0;
                      -(map_size/2)+obs_x_shift1,2*map_size,0;
                      -(map_size/2)+obs_x_shift1,3*map_size,0;
                        map_size/2+obs_x_shift2,0,0;
                        map_size/2+obs_x_shift2,map_size,0;
                        map_size/2+obs_x_shift2,2*map_size,0;
                        map_size/2+obs_x_shift2,3*map_size,0;
                        
                       (3*map_size/2)+obs_x_shift1+obs_x_shift4,0,0;
                      (3*map_size/2)+obs_x_shift1+obs_x_shift4,map_size,0;
                      (3*map_size/2)+obs_x_shift1+obs_x_shift4,2*map_size,0;
                      (3*map_size/2)+obs_x_shift1+obs_x_shift4,3*map_size,0;];
    for ii = 1:16
        map = generate_map([], map_size, nb_blocks_i, bulding_width, rand_pos_obs_num,obs_density,ii);
        nb_obstacles = length(map.buildings_east);
        cylinder_radius = map.building_width / 2;
        
        cylinders_ii = [
                map.buildings_east'; % x_obstacle
                map.buildings_north'; % y_obstacle
                repmat(cylinder_radius, 1, nb_obstacles)]';
        
        obs_pos_list_ii = cylinders_ii + obs_shift_list(ii,:);
        obs_pos_list = [obs_pos_list;obs_pos_list_ii];
    end

end


end

