function MC_PlotInterAgentDistance()
select = 'best'; % 'step' 'best'

switch select
    case 'step'

        stepNumArr = 1:10;
        Nbest = 2;

        McData = plotStep(stepNumArr, Nbest);

    case 'best'

        stepNum = 1;
        NbestArr = 1;

        McData = plotBest(stepNum, NbestArr);

end

% Compute Statistics
numSim = length(McData);

meanVals = zeros(1, numSim);
stdVals  = zeros(1, numSim);

for i = 1:numSim
    meanVals(i) = mean(McData(i).InterAgentDistanceData);
    stdVals(i)  = std(McData(i).InterAgentDistanceData);
end

% Create InterAgentDistanceed categorical labels (ρ₁ ... ρ₁₀)
labels = compose('\\rho_{%d}', 1:numSim);
x = categorical(labels, labels, 'Ordinal', true);

% Plot
figure;
hBar = bar(x, meanVals, 0.6);
hold on

% Bar color (SFM style)
hBar.FaceColor = [0.4940 0.1840 0.5560];

% Error bars
errorbar(1:numSim, meanVals, stdVals, '.k', 'LineWidth', 2)

% Styling
xlabel('Density','FontSize',22)
ylabel('\boldmath${\psi}$','FontSize',30,'Interpreter','latex')

set(gca,'linewidth',1,'FontSize',22)
% ylim([0 1.05])
grid on

end

function McData = plotStep(stepNumArr, Nbest)

% Collect Data
McData = struct();

for stepNum_i = 1:length(stepNumArr)
    stepNum = stepNumArr(stepNum_i);

    FileDir = [pwd, '\logs\stepNum', num2str(stepNum), '\Nbest', num2str(Nbest)];

    FileNames = dir(FileDir);
    FileNames = FileNames(3:end);

    InterAgentDistanceData = [];

    for i = 1:length(FileNames)
        data = load([FileDir, '\', FileNames(i).name]);

        [~, ~, ~, arrAd] = analyze_interagent_dist(data.dt, data.sim_pos_list, data.inter_agent_dist, double(data.sim_kk), data.quadNum, data.quad_rad_real, data.sense_range, data.quadK, data.d_safety_rob);

        % Remove NaNs
        validIdx = ~isnan(arrAd);
        arrAd = arrAd(validIdx);

        % Concatenate
        InterAgentDistanceData = [InterAgentDistanceData, arrAd];
    end

    McData(stepNum_i).InterAgentDistanceData = InterAgentDistanceData;
end

end

function McData = plotBest(stepNum, NbestArr)

% Collect Data
McData = struct();

for Nbest_i = 1:length(NbestArr)
    Nbest = NbestArr(Nbest_i);

    FileDir = [pwd, '\logs\stepNum', num2str(stepNum), '\Nbest', num2str(Nbest)];

    FileNames = dir(FileDir);
    FileNames = FileNames(3:end);

    InterAgentDistanceData = [];

    for i = 1:length(FileNames)
        data = load([FileDir, '\', FileNames(i).name]);

        [~, ~, ~, arrAd] = analyze_interagent_dist(data.dt, data.sim_pos_list, data.inter_agent_dist, double(data.sim_kk), data.quadNum, data.quad_rad_real, data.sense_range, data.quadK, data.d_safety_rob);

        % Remove NaNs
        validIdx = ~isnan(arrAd);
        arrAd = arrAd(validIdx);

        % Concatenate
        InterAgentDistanceData = [InterAgentDistanceData, arrAd];
    end

    McData(Nbest_i).InterAgentDistanceData = InterAgentDistanceData;
end

end

