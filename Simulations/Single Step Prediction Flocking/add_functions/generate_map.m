%Copyright 2020 Enrica Soria
%places obstacles on the arena based on desired obstacle density
function map = generate_map(map, map_size, nb_blocks, bulding_width, ranNum, obs_density, ii)

    map.shift_ind = mod(ii,2);

    % map parameters
    map.width = map_size; % width of the map
    map.bl_corner_north = +map.width/5;
    map.bl_corner_east = -map.width/2;
    map.max_height = map.width; % maximum height of obstacles
    map.nb_blocks = nb_blocks; % the number of blocks per row
    
    map.street_width_perc = 0.45; % percentage of block that is empty
    map.building_width = bulding_width;
    map.street_width = map.width/map.nb_blocks*map.street_width_perc;

    map.building_shape = ('cylinder');

    % Create map parameters
    map = create_random_buildings(map,ranNum);
    map.buildings_north = map.buildings_north + map.bl_corner_north;
    map.buildings_east = map.buildings_east + map.bl_corner_east;
    
end