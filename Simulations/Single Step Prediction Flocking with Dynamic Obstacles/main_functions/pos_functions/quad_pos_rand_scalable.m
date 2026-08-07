%generate random initial positions for robots based on the scale:
% scale = 1 -> 5 robots
% scale = 2 -> 20 robots
% scale = 3 -> 80 robots
function [genPos] = quad_pos_rand_scalable(ranNum,z_pos,scale)
 
genPos = [];

rng(ranNum)
if scale == 1
   pos_0s = [-0.67,-0.4,z_pos; 0.67,-0.4,z_pos;
            0,-0.8,z_pos;
           -0.67,-1.2,z_pos; 0.67,-1.2,z_pos];
   genPos = pos_0s + (-0.25 + (0.25-(-0.25)) .* rand(5,3));
 
elseif scale == 2
    
    row_num = 4;
    col_num = 5;
    x_0 = -1.6;
    y_0 = -0.4;
    for ij=1:row_num
        for jj=1:col_num
            genPosi = [x_0+0.8*(jj-1), y_0-0.8*(ij-1),z_pos];
            genPos = [genPos; genPosi];
        end
    end
    genPos = genPos + (-0.25 + (0.25-(-0.25)) .* rand(20,3));

elseif scale == 3

    row_num = 8;
    col_num = 10;
    x_0 = -3.6;
    y_0 = -0.4;
    for ij=1:row_num
        for jj=1:col_num
            genPosi = [x_0+0.8*(jj-1), y_0-0.8*(ij-1),z_pos];
            genPos = [genPos; genPosi];
        end
    end
    genPos = genPos + (-0.25 + (0.25-(-0.25)) .* rand(80,3));
   

end


end

