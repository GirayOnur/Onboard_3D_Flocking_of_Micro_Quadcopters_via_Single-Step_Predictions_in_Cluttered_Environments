%calculates migration heuristic
function [migHeurCost] = migCost(migAng,migVel,currentVel,currentHeadVec)
kVel = 2.0; %speed gain

kDir = 2.0; %direction gain

migVelVec = migVel*[cos(migAng) sin(migAng), 0]; %migration velocity

agentVelVec = currentVel*currentHeadVec; %velocity of the robot

migHeurCostDir = kDir*(1 - dot(migVelVec,agentVelVec)/(migVel*currentVel)); %direction heuristic

migHeurCostVel = kVel*abs(migVel - currentVel)/migVel; %speed heuristic

migHeurCost = migHeurCostDir + migHeurCostVel; %migration heuristic
end
