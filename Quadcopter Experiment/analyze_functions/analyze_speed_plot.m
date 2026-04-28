
function [done]=analyze_speed_plot(dt,velData,refVel,simStep,agentNum) %(dt,sim_vel_list,migVel,double(sim_kk),quadNum)
    arrTime = linspace(0,dt*(simStep-1),simStep);
    arrMax = [];
    arrEv =  [];
    arrMin = [];

    for i=1:simStep
        velArr = velData(i,:);
        Ev = sum(abs(velArr)) / (agentNum);
        velMax = max(velArr);
        velMin = min(velArr);

        arrMax(end+1) = velMax;
        arrEv(end+1) = Ev;
        arrMin(end+1) = velMin;     
    end

    shade(arrTime,arrMax, 'w', arrTime,arrMin, 'w' , 'FillType', [1 2; 2 1], 'FillColor',[0.8500 0.3250 0.0980],'FillAlpha',0.2);
    hold on
    plot(arrTime,arrEv, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2)
    set(gca,'XLim',[0 17.7]) %axis limits
    set(gca,'YLim',[0 2]) %axis limits
    set(gca,'fontsize', 22);
    set(get(gca,'XLabel'),'String','\boldmath$t [s]$','interpreter', 'latex','fontsize', 24)
    set(get(gca,'YLabel'),'String','\boldmath$|\!| v_i |\!| / |\!| v^{m} |\!|$','interpreter', 'latex','fontsize', 24)
    %legend('Max. speed','Avg. speed','Min. speed','','','')
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
    done = "Speed test is done";
