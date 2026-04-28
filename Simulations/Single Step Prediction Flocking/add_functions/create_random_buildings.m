%Copyright 2020 Enrica Soria
%adapted to generate obstacle positions for different swarm scales
function map = create_random_buildings(map,ranNum)

    for i = 1:map.nb_blocks
        buildings_north(i) = 0.5 * map.width/map.nb_blocks * (2*(i-1)+1);
    end
    buildings_north = buildings_north';

    map.buildings_east = [];
    map.buildings_north = [];
    %rng(ranNum);
    for i = 1:map.nb_blocks
        rand_n = 0.5 + (1.5-0.5) .* rand;
        offsets = repmat(rand_n/2*map.width/map.nb_blocks,map.nb_blocks-1,1);
        if mod(i,2) == map.shift_ind
            map.buildings_north = [map.buildings_north; ...
                repmat(buildings_north(i), map.nb_blocks-1, 1)];
            map.buildings_east = [map.buildings_east; ...
                buildings_north(1:(end-1)) + ...
                (map.building_width+map.street_width)/2 - ...
                offsets/2];
        else
            map.buildings_north = [map.buildings_north; ...
                repmat(buildings_north(i), map.nb_blocks-1, 1)];
            map.buildings_east = [map.buildings_east; ...
                buildings_north(1:(end-1)) + ...
                (map.building_width+map.street_width)/2 + ...
                offsets/2];
        end
    end

    nb_buildings = length(map.buildings_east);
    map.buildings_heights = map.max_height*ones(nb_buildings,1);
    
end