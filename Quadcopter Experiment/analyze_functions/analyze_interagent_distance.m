
function [done,arrEd]=analyze_interagent_distance(dt,posData,refDist,simStep,agentNum,quadRad,senseRange,Mmax,dSafetyRob,real) %(dt,sim_pos_list,???,double(sim_kk),quadNum,quad_rad,sense_range)
    ign_kk = int16(0.5/dt);
    arrTime = linspace(0,dt*(simStep-1),simStep);
    arrMax = [];
    arrEd =  [];
    arrMin = [];
    
    %quadquadBias = 2*quadRad;
    
    %senseRange = senseRange+quadRad;
    
    interDetected = 0;

    for i=1:simStep
        
        posArr = reshape(posData(i,:)',3,agentNum)';
        EdSum = 0.0;
        quadDistMinArr = [];
        quadDistMaxArr = [];
        totNumSep=0;

        for n=1:agentNum
            quadPos = posArr(n,:);
            interAgentDistArr = vecnorm((posArr(1:end~=n,:) - quadPos),2,2); %- quadquadBias; % consider extracting quadquadBias, too
            interAgentDistArr = interAgentDistArr(interAgentDistArr<senseRange);
            interAgentDistArr = sort(interAgentDistArr);
            if length(interAgentDistArr) > Mmax
                interAgentDistArr = interAgentDistArr(1:Mmax,:);
            end
            
            quadInRange = length(interAgentDistArr);

            if quadInRange~=0
                interAgentDistArr = interAgentDistArr; %- quadquadBias;
                quadDistMinArr(end+1) = min(interAgentDistArr);
                quadDistMaxArr(end+1) = max(interAgentDistArr);
                interAgentDistSum = sum(interAgentDistArr) / (quadInRange);
                EdSum = EdSum + interAgentDistSum;
            else
                interDetected = 1;
                totNumSep = totNumSep + 1;
            end
        end
        
        quadDistMin = min(quadDistMinArr);
        quadDistMax = max(quadDistMaxArr);
        
        arrMax(end+1) = quadDistMax/refDist;
        arrEd(end+1) = EdSum / (agentNum*refDist);
        arrMin(end+1) = quadDistMin/refDist;
         
    end


    disp(mean(arrEd(ign_kk:end)))
    disp(std(arrEd(ign_kk:end)))
    
    if interDetected
        disp('Separated agent detected.')
    end
    if real
        shade(arrTime,arrMax, arrTime,arrMin, 'Color','none' , 'FillType', [1 2; 2 1], 'FillColor',[0.4940 0.1840 0.5560],'FillAlpha',0.3);
        hold on
        plot(arrTime,arrEd, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.2)
    else
        shade(arrTime,arrMax, arrTime,arrMin, 'Color', 'none', 'FillType', [1 2; 2 1], 'FillColor',[0 0.4470 0.7410],'FillAlpha',0.15);
        hold on
        h2a = plot(arrTime,arrEd, 'Color', [0 0.4470 0.7410], 'LineWidth', 1);
        h2a.Color(4)=0.5;          
    end
    yline(dSafetyRob/refDist,'r--', 'LineWidth',1.2)
    yline(2*quadRad/refDist,'k--', 'LineWidth',1.2)
    set(gca,'XLim',[0 12]) %axis limits
    set(gca,'YLim',[0 2]) %axis limits
    set(gca,'fontsize', 22);
    set(get(gca,'XLabel'),'String','\boldmath$t [s]$','interpreter', 'latex','fontsize', 24)
    set(get(gca,'YLabel'),'String','\boldmath$d_{j}/d_{r}$','interpreter', 'latex','fontsize', 24)
    %legend('Max. distance','Avg. distance','Min. distance','','','')
    greyout = 0;
    %disp(arrTime(end))
    if greyout
        xlim = get(gca,'XLim');
        ylimm = get(gca,'YLim');
        xl1 = arrTime(end);
        xl2 = xlim(2);
        yl1 = ylimm(1);
        yl2 = ylimm(2);
        xBox = [xl1, xl1, xl2, xl2, xl1];
        yBox = [yl1, yl2, yl2, yl1, yl1];
        patch(xBox, yBox, 'black', 'FaceColor', 'black', 'FaceAlpha', 0.2,'LineStyle','none');
    end
    done = "Inter-agent distance test is done";

