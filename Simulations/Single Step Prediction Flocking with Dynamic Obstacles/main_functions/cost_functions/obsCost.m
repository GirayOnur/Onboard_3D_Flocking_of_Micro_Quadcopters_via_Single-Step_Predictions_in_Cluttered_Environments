%calculates obstacle avoidance heuristic
function [obsHeurCost] = obsCost(obsRelMagList,obsApproachList,obsKK,sensRange,dSafety)

kCollObs = 1000; %collision safety gain
kObs = 10; %obstacle avoidance gain
pObs = 3;
d0 = 2.0; %obstacle influence threshold

obsHeurCost = 0.0;
for ddd=1:obsKK
    obsDist = obsRelMagList(ddd); %relative distance to an obstacle
    obsApproachTerm = obsApproachList(ddd)^pObs; %approach term
    if obsDist <= dSafety
        obsCostCoeff = (kCollObs*kObs*0.5*((1/obsDist - 1/d0)^2))/obsKK;
    elseif obsDist <= d0
        obsCostCoeff = (obsApproachTerm*kObs*0.5*((1/obsDist - 1/d0)^2))/obsKK;
    else
        obsCostCoeff = 0;
    end
    obsHeurCost =  obsHeurCost + obsCostCoeff;
end


end
