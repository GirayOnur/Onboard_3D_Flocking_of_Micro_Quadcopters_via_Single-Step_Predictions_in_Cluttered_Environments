%calculates inter-robot distance heuristic
function [quadHeurCost] = quadCost(quadRelMagList,quadKK,desDist,dSafety)

quadCostCoeff = 250; %inter-robot gain
quadHeurCost = 0.0;
kCollRob = 1000; %collision safety gain

for ddd=1:quadKK
    quadNeigDistRel = quadRelMagList(ddd); %relative distance to a neighbor   
    if quadNeigDistRel <= dSafety
        quadNeigCostCoeff = (kCollRob*quadCostCoeff*0.5*(quadNeigDistRel - desDist)^2)/quadKK;
    else
        quadNeigCostCoeff = (quadCostCoeff*0.5*(quadNeigDistRel - desDist)^2)/quadKK;
    end
    quadHeurCost = quadHeurCost + quadNeigCostCoeff;

end

end

