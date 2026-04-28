%plots and animates trajectories of the quadcopters

plot_traj = 1; %1 -> plot trajectories, 0 -> animate trajectories
side_view = 0; %1 -> side view, 0 -> top view
save_video = 0; %1 -> save the animation as a movie, 0 -> do not save the animation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%simulator:
dt_sim=0.000000001;
sim_time = sim_kk*0.05;
obs_mode = 1;

simulation_time = sim_time;

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
%set axis limits

%loop_index_ref = [1 4 7 10 13 16];
loop_index_ref = int16(linspace(1,3*quadNum-2,quadNum));


if plot_traj
    %plot obstacles:
    if obs_mode==-1
        plot_arena = plot3( [xr1 xr2 xr2 xr1 xr1], [yr1 yr1 yr2 yr2 yr1], [1 1 1 1 1] ,'Color','black');
        plot_arena.LineWidth = 1;
        hold on
    elseif obs_mode == 1
        for i=1:obsNum
            obsRad = obs_rad_list_real(i);
            [obsX,obsY,obsZ] = cylinder(obsRad);
            obsZ(2, :) = 10;
            surf(obsX + obs_pos_list(i,1) ,obsY + obs_pos_list(i,2),obsZ, 'FaceColor',[0.4660 0.6740 0.1880])
            hold on
            
            if ~side_view
                dashed_circle(obs_pos_list(i,1), obs_pos_list(i,2),obs_pos_list(i,3)+0.01, d_safety_obs, 'k')
            end
        end
        if side_view
            line([2,2],[0,8],[1.25,1.25],'LineStyle', '--' ,'Color', 'r', 'LineWidth', 0.8);
            line([2,2],[0,8],[0.25,0.25], 'LineStyle', '--' ,'Color', 'r', 'LineWidth', 0.8);
        end
    else
        for i=1:obsNum
            obsRad = obs_rad_list_real(i)-0.25;
            [obsX,obsY,obsZ] = cylinder(obsRad);
            surf(obsX + obs_pos_list(i,1) ,obsY + obs_pos_list(i,2),obsZ ,'FaceColor', [0.4660 0.6740 0.1880])
            hold on
            circle(obs_pos_list(i,1), obs_pos_list(i,2),1.0, obsRad+0.2, [0.4660 0.6740 0.1880]);
            circle(obs_pos_list(i,1), obs_pos_list(i,2),0.75, obsRad+0.25,'k');
        end
    end
    view(az,el)
    grid on
    axis equal
    set(gca,'XLim',[-3.5 3.5],'YLim',[0 8],'ZLim',[0 1.5]) %axis limits
    zticks([0,0.75,1.5])
    x_arr = animate_data(:,loop_index_ref);
    y_arr = animate_data(:,(loop_index_ref + 1));
    z_arr = animate_data(:,loop_index_ref + 2);  
    for nn=1:quadNum
        t = linspace(1,double(sim_time),double(sim_kk));
        x = x_arr(:,nn);
        y = y_arr(:,nn);
        z = z_arr(:,nn);
        
        tt = linspace(t(1),t(end));
        xx = interp1(t,x',tt,'spline');
        yy = interp1(t,y',tt,'spline');
        zz = interp1(t,z',tt,'spline');

        if real
            plot3(xx,yy,zz,'Color', [0.4940 0.1840 0.5560])
        else
            plot3(xx,yy,zz,'Color', [0 0.4470 0.7410])
        end
    end
    yline(7.5,'k--','LineWidth',1.2);
    set(gca,'fontsize', 22);    
    set(get(gca,'XLabel'),'String','$x [m]$','interpreter', 'latex','fontsize', 24)
    set(get(gca,'YLabel'),'String','$y [m]$','interpreter', 'latex','fontsize', 24)
    set(get(gca,'ZLabel'),'String','$z [m]$','interpreter', 'latex','fontsize', 24)

elseif save_video
    set(gcf, 'WindowState', 'maximized');
    pause(0.1)

    %plot obstacles:
    if obs_mode==-1
        plot_arena = plot3( [xr1 xr2 xr2 xr1 xr1], [yr1 yr1 yr2 yr2 yr1], [1 1 1 1 1] ,'Color','black');
        plot_arena.LineWidth = 3;
        hold on
        
    else
        for i=1:obsNum
            obsRad = obs_rad_list_real(i);
            [obsX,obsY,obsZ] = cylinder(obsRad);
            surf(obsX + obs_pos_list(i,1) ,obsY + obs_pos_list(i,2),obsZ)
            hold on
        end
    end
    axis equal
    set(gca,'XLim',[-7 7],'YLim',[0 14],'ZLim',[0 1.5]) %axis limits

    myVideo = VideoWriter('psm_movie'); %open video file
    myVideo.FrameRate = 1/dt;
    open(myVideo)

    for i=1:(n_loop)
        quad_plot_list = [];
        x = animate_data(i,loop_index_ref);
        y = animate_data(i,(loop_index_ref + 1));
        z = animate_data(i,loop_index_ref + 2);
        for nn=1:quadNum
             quad_plot_list(end+1) = fill3(Xr+x(nn),Yr+y(nn),Zr+z(nn),Cr);
        end
        view(az,el)
        rotate3d on
        %drawnow
        pause(dt_sim)
        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);
        [az,el] = view;
        for nn=1:quadNum
            delete(quad_plot_list(nn))
        end
    end
    close(myVideo)
else
    %plot obstacles:
    if obs_mode==-1
        plot_arena = plot3( [xr1 xr2 xr2 xr1 xr1], [yr1 yr1 yr2 yr2 yr1], [1 1 1 1 1] ,'Color','black');
        plot_arena.LineWidth = 3;
        hold on
                
    else
        for i=1:obsNum
            obsRad = obs_rad_list_real(i);
            [obsX,obsY,obsZ] = cylinder(obsRad);
            surf(obsX + obs_pos_list(i,1) ,obsY + obs_pos_list(i,2),obsZ)
            hold on
        end
    end
    grid on
    axis equal
    set(gca,'XLim',[-3.5 3.5],'YLim',[0 8],'ZLim',[0 1.5]) %axis limits

    for i=1:(n_loop)
        quad_plot_list = [];
        x = animate_data(i,loop_index_ref);
        y = animate_data(i,(loop_index_ref + 1));
        z = animate_data(i,loop_index_ref + 2);
        for nn=1:quadNum
             quad_plot_list(end+1) = fill3(Xr+x(nn),Yr+y(nn),Zr+z(nn),Cr);
        end
        view(az,el)
        rotate3d on
        drawnow
        pause(dt_sim)
        [az,el] = view;
        for nn=1:quadNum
            delete(quad_plot_list(nn))
        end
    end
end


function circles = circle(x,y,z,r,c)
    C = [x,y,z] ; 
    R = r;
    teta=0:0.01:2*pi ;
    x=C(1)+R*cos(teta);
    y=C(2)+R*sin(teta) ;
    z = C(3)+zeros(size(x)) ;
    patch(x,y,z,c,'LineStyle','none')
    circles = 1;
end

function dashed_circles = dashed_circle(x,y,z,r,c)
    C = [x,y,z];
    R = r;
    teta=0:0.01:2*pi ;
    x=C(1)+R*cos(teta);
    y=C(2)+R*sin(teta);
    z = C(3)+zeros(size(x));
    %plot(x,y)
    patch(x,y,z, 'EdgeColor', 'r', 'LineStyle','--','FaceColor','none','LineWidth',1.0)

    dashed_circles = 1;
end