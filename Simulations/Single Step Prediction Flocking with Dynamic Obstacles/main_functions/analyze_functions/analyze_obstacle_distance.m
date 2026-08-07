%plots the minimum obstacle distance metric for moving obstacles:
function [done,arrOd]=analyze_obstacle_distance(dt,posData,obsPosList,obsRadList,simStep,agentNum,obsNum,quadRad,dSafetyObs,obsRad)
    ign_kk = int16(0.5/dt); %number of initial steps ignored while averaging

    arrTime = linspace(0,dt*(simStep-1),simStep);
    arrOd = [];
    collDetected = 0;

    for i=1:simStep
        posArr = reshape(posData(i,:)',3,agentNum)';
        for n=1:agentNum
            quadPos = posArr(n,:);
            %obstacle positions are read at the current step since the obstacles move
            obsAgentDistArr = vecnorm(abs(obsPosList(:,:,i) - quadPos),2,2); %relative obstacle distances
            obsAgentMinDist = min(obsAgentDistArr); %closest obstacle distance of the robot
            if obsAgentMinDist < 0
                collDetected = 1;
            end
            arrOd(i,n) = obsAgentMinDist; %minimum obstacle distance of each robot at each step
        end
    end
    disp(mean(arrOd(ign_kk:end)))
    disp(std(arrOd(ign_kk:end)))

    hold on
    yline(dSafetyObs,'r--', 'LineWidth',1.2)
    yline(obsRad+quadRad,'k--', 'LineWidth',1.2)

    if collDetected == 1
        disp("Robot-obstacle collision is detected")
    end
    done = "Agent-obstacle safety test is done";

    %shade the band between the closest and the furthest obstacle distances:
    hold on, shade(arrTime,max(arrOd'),'w', arrTime,min(arrOd'),'w', 'FillType', [1 2; 2 1], 'FillColor',	[0.4940 0.1840 0.5560],'FillAlpha',0.2);
    hold on, plot(arrTime,mean(arrOd'),'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.2);

    set(gca,'YLim',[0 4.75]) %axis limits
    set(gca,'XLim',[0 12]) %axis limits
    set(gca,'fontsize', 22);
    set(get(gca,'XLabel'),'String','\boldmath$t [s]$','interpreter', 'latex','fontsize', 24)
    set(get(gca,'YLabel'),'String','\boldmath$m_{o} [m]$','interpreter', 'latex','fontsize', 24)
