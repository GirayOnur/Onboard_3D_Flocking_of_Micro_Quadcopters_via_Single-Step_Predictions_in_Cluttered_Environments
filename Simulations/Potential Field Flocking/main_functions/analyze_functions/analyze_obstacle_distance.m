%plots the obstacle distance metric:
function [done,arrOd]=analyze_obstacle_distance(dt,posData,obsPosList,obsRadList,simStep,agentNum,obsNum,quadRad,obsRad)
    ign_kk = int16(0.5/dt);

    arrTime = linspace(0,dt*(simStep-1),simStep);
    arrOd =  [];
    %obsquadBias = obsRadList; %obsRadList + quadRad;
    collDetected = 0;

    for i=1:simStep
        posArr = reshape(posData(i,:)',3,agentNum)';   
        OdSum = 0.0;
        obsAgentMinDistArr = [];
        for n=1:agentNum
            quadPos = posArr(n,:);
            obsAgentDistArr = vecnorm(abs(obsPosList(:,1:2) - quadPos(:,1:2)),2,2); % - obsquadBias;
            obsAgentMinDist = min(obsAgentDistArr);
            obsAgentMinDistArr(end+1) = obsAgentMinDist;
            if obsAgentMinDist < 0
                collDetected = 1;
            end
        end
        arrOd(end+1) = min(obsAgentMinDistArr);      

    end
    disp(mean(arrOd(ign_kk:end)))
    disp(std(arrOd(ign_kk:end)))
    plot(arrTime,arrOd,'Color', [0 0.4470 0.7410] , 'LineWidth', 1.2)
    hold on
    %set(gca,'XLim',[0 15.65]) %axis limits
    %yline(dSafetyObs,'r--', 'LineWidth',1.2)
    yline(obsRad+quadRad,'k--', 'LineWidth',1.2)
    set(gca,'YLim',[0 2]) %axis limits
    %set(gca,'XLim',[0 36.66]) %axis limits
    set(gca,'fontsize', 22);
    set(get(gca,'XLabel'),'String','\boldmath$t [s]$','interpreter', 'latex','fontsize', 24)
    set(get(gca,'YLabel'),'String','\boldmath$m_{o} [m]$','interpreter', 'latex','fontsize', 24)
    greyout = 0;
    if greyout
        xlim = get(gca,'XLim');
        ylim = get(gca,'YLim');
        xl1 = arrTime(end);
        xl2 = xlim(2);
        yl1 = ylim(1);
        yl2 = ylim(2);
        xBox = [xl1, xl1, xl2, xl2, xl1];
        yBox = [yl1, yl2, yl2, yl1, yl1];
        patch(xBox, yBox, 'black', 'FaceColor', 'black', 'FaceAlpha', 0.2,'LineStyle','none');
    end

    if collDetected == 1
        disp("Robot-obstacle collision is detected")
    end
    done = "Agent-obstacle safety test is done";