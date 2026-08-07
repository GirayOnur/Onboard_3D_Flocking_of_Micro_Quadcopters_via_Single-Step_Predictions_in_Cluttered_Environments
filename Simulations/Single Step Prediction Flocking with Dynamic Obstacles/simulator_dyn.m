%Animates trajectories of the robots together with the moving obstacles
figure;

side_view = 0; %1 -> side view, 0 -> top view

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%simulator:
dt_sim=0.000000001;

n_loop = sim_kk;
animate_data = sim_pos_list;

% quad graphical model:
Xr = [-0.04 -0.06 -0.02 -0.06 -0.04 0.0 0.04 0.06 0.02 0.06 0.04 0.0];
Yr = [-0.06 -0.04 0.0 0.04 0.06 0.02 0.06 0.04 0.0 -0.04 -0.06 -0.02];
Zr = [0 0 0 0 0 0 0 0 0 0 0 0];
Cr = zeros(1,12);

%set camera angles
if side_view
    az = 90;
    el = 0;
else
    az = 0;
    el = 90;
end

loop_index_ref = int16(linspace(1,3*quadNum-2,quadNum));

grid on
axis equal
set(gca,'XLim',[-2.5 2.5],'YLim',[-2.5 2.5],'ZLim',[0 1.5]) %axis limits
for i=1:(n_loop)
    quad_plot_list = [];
    x = animate_data(i,loop_index_ref);
    y = animate_data(i,(loop_index_ref + 1));
    z = animate_data(i,loop_index_ref + 2);
    %draw the robots at their current positions:
    for nn=1:quadNum
        quad_plot_list(end+1) = fill3(Xr+x(nn),Yr+y(nn),Zr+z(nn),Cr);
    end
    %draw the obstacles at their current positions, the same graphical model
    %is used since the obstacles are also quadcopters:
    for nn = 1:obsNum
        quad_plot_list(end+1) = fill3(...
            Xr + obs_pos_list_(nn,1,i),...
            Yr + obs_pos_list_(nn,2,i),...
            Zr + obs_pos_list_(nn,3,i),...
            Cr);
    end
    view(az,el)
    rotate3d on
    drawnow
    pause(dt_sim)
    [az,el] = view;
    %clear the drawn robots and obstacles before the next step
    for nn=1:length(quad_plot_list)
        delete(quad_plot_list(nn))
    end
end
