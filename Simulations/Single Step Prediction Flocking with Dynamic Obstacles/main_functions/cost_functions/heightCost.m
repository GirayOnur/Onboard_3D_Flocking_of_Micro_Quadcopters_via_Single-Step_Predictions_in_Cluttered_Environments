%calculates altitude heuristic
function [heightHeurCost] = heightCost(currentHeight,desHeight,dMargin)
% cost calculation for quad-quad collision avoidance and quad proximality

kMarginViolation = 100; %altitude gain

heightError = abs(currentHeight-desHeight); %altitude difference

if heightError <= dMargin
    heightHeurCost = 0;
else
    heightHeurCost = (kMarginViolation*0.5*((1/heightError - 1/dMargin)^2));
end

end

