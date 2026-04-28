%plots the order metric:
function [done,arrOrder,arrAccuracy]=analyze_order_vel(dt,velData,simStep,agentNum,migVel,plotAccuracy)
    ign_kk = int16(0.5/dt);
    arrTime = linspace(0,dt*(simStep-1),simStep);
    
    arrOrder =  [];
    arrAccuracy = [];
    
    migAng = atan2(migVel(2),migVel(1));
    
    for i=1:simStep
        
        velArr = reshape(velData(i,:)',3,agentNum)';
        uArr = velArr./vecnorm(velArr,2,2);
        uSumArr = sum(uArr);
        orderStep = norm(uSumArr)/agentNum;
        arrOrder(end+1) = orderStep;
        uSumAng = atan2(uSumArr(2),uSumArr(1));
        accuracyStep = 1 - (1-orderStep*cos(uSumAng-migAng))/2;
        arrAccuracy(end+1) = accuracyStep;
    end    
    
    if plotAccuracy
        disp(mean(arrAccuracy))
        disp(std(arrAccuracy))  
        plot(arrTime,arrAccuracy,'Color', [0 0.4470 0.7410] , 'LineWidth', 1.2)
        ylim([0 1])
        %set(gca,'XLim',[0 17.7]) %axis limits
        set(gca,'fontsize', 22);
        set(get(gca,'XLabel'),'String','\boldmath$t [s]$','interpreter', 'latex','fontsize', 24)
        set(get(gca,'YLabel'),'String','\boldmath$\delta$','interpreter', 'latex','fontsize', 24)
    else
        disp(mean(arrOrder(ign_kk:end)))
        disp(std(arrOrder(ign_kk:end)))
        %plot(arrTime,arrOrder,'Color', [0 0.4470 0.7410] , 'LineWidth', 1)
        plot(arrTime,arrOrder, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.2) %normal 1.2, bold 3 (for zoom)
        ylim([0 1])
        %set(gca,'XLim',[0 10.6]) %axis limits
        %set(gca,'XLim',[0 36.66]) %axis limits
        set(gca,'fontsize', 22);
        set(get(gca,'XLabel'),'String','\boldmath$t [s]$','interpreter', 'latex','fontsize', 24)
        set(get(gca,'YLabel'),'String','\boldmath$\psi$','interpreter', 'latex','fontsize', 24)  
    end
    
    
    greyout = 0;
    if greyout
        xlim = get(gca,'XLim');
        ylimm = get(gca,'YLim');
        xl1 = arrTime(end);
        xl2 = xlim(2);
        yl1 = ylimm(1);
        yl2 = ylimm(2);
        xBox = [xl1, xl1, xl2, xl2, xl1];
        yBox = [yl1, yl2, yl2, yl1, yl1];
        patch(xBox, yBox, 'black', 'FaceColor', 'black', 'FaceAlpha', 0.1,'LineStyle','none');
    end
    done = "Order test is done";