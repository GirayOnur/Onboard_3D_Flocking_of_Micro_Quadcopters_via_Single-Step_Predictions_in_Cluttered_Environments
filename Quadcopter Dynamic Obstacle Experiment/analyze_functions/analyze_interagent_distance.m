%plots inter-robot distances:
function [done,arrEd,arrMin]=analyze_interagent_distance(dt,posData,refDist,simStep,agentNum,quadRad,senseRange,Mmax,dSafetyRob)
    ign_kk = int16(0.5/dt); %number of initial steps ignored while averaging
    arrTime = linspace(0,dt*(simStep-1),simStep);
    arrMax = [];
    arrEd =  [];
    arrMin = [];

    interDetected = 0;

    for i=1:simStep

        posArr = reshape(posData(i,:)',3,agentNum)';
        EdSum = 0.0;
        quadDistMinArr = [];
        quadDistMaxArr = [];
        totNumSep=0;

        for n=1:agentNum
            quadPos = posArr(n,:);
            interAgentDistArr = vecnorm((posArr(1:end~=n,:) - quadPos),2,2); %relative neighbor distances
            %consider the neighbors in sensing range
            interAgentDistArr = interAgentDistArr(interAgentDistArr<senseRange);
            interAgentDistArr = sort(interAgentDistArr);
            %consider closest Mmax neighbors due to topological selection
            if length(interAgentDistArr) > Mmax
                interAgentDistArr = interAgentDistArr(1:Mmax,:);
            end

            quadInRange = length(interAgentDistArr); %number of considered neighbors

            if quadInRange~=0
                quadDistMinArr(end+1) = min(interAgentDistArr);
                quadDistMaxArr(end+1) = max(interAgentDistArr);
                interAgentDistSum = sum(interAgentDistArr) / (quadInRange);
                EdSum = EdSum + interAgentDistSum;
            else
                %the robot has no neighbor left in its sensing range
                interDetected = 1;
                totNumSep = totNumSep + 1;
            end
        end

        quadDistMin = min(quadDistMinArr);
        quadDistMax = max(quadDistMaxArr);

        %normalize the distances with the reference inter-robot distance:
        arrMax(end+1) = quadDistMax/refDist;
        arrEd(end+1) = EdSum / (agentNum*refDist);
        arrMin(end+1) = quadDistMin/refDist;

    end


    disp(mean(arrEd(ign_kk:end)))
    disp(std(arrEd(ign_kk:end)))

    if interDetected
        disp('Separated agent detected.')
    end

    %shade the band between the closest and the furthest neighbor distances:
    shade(arrTime,arrMax,'w', arrTime,arrMin,'w', 'FillType', [1 2; 2 1], 'FillColor',	[0.4940 0.1840 0.5560],'FillAlpha',0.2);
    plot(arrTime,arrEd, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.2)
    hold on
    yline(dSafetyRob/refDist,'r--', 'LineWidth',1.2)
    yline(2*quadRad/refDist,'k--', 'LineWidth',1.2)
    set(gca,'YLim',[0 2]) %axis limits
    set(gca,'XLim',[0 12]) %axis limits
    set(gca,'fontsize', 22);
    set(get(gca,'XLabel'),'String','\boldmath$t [s]$','interpreter', 'latex','fontsize', 24)
    set(get(gca,'YLabel'),'String','\boldmath$d_{j}/d_{r}$','interpreter', 'latex','fontsize', 24)
    done = "Inter-agent distance test is done";
