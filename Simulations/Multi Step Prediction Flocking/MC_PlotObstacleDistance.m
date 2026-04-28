function MC_PlotObstacleDistance()

select = 'step'; % 'step' 'best'

switch select
    case 'step'

        stepNumArr = 1:10;
        Nbest = 7;

        McData = plotStep(stepNumArr, Nbest);

    case 'best'

        stepNum = 10;
        NbestArr = 1:7;

        McData = plotBest(stepNum, NbestArr);

end

% Compute Statistics
numSim = length(McData);

meanVals = zeros(1, numSim);
stdVals  = zeros(1, numSim);

for i = 1:numSim
    meanVals(i) = mean(McData(i).ObstacleDistanceData);
    stdVals(i)  = std(McData(i).ObstacleDistanceData);
end

% Create ObstacleDistanceed categorical labels (ρ₁ ... ρ₁₀)
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

%
figure;
boxplot([...
    McData(1).ObstacleDistanceData', ...
    McData(2).ObstacleDistanceData', ...
    McData(3).ObstacleDistanceData', ...
    McData(4).ObstacleDistanceData', ...
    McData(5).ObstacleDistanceData', ...
    McData(6).ObstacleDistanceData', ...
    McData(7).ObstacleDistanceData', ...
    McData(8).ObstacleDistanceData', ...
    McData(9).ObstacleDistanceData', ...
    McData(10).ObstacleDistanceData'],'Notch','on','Labels',{'1','2','3','4','5','6','7','8','9','10'});

end

function McData = plotStep(stepNumArr, Nbest)

% Collect Data
McData = struct();

for stepNum_i = 1:length(stepNumArr)
    stepNum = stepNumArr(stepNum_i);

    FileDir = [pwd, '\logs\stepNum', num2str(stepNum), '\Nbest', num2str(Nbest)];

    FileNames = dir(FileDir);
    FileNames = FileNames(3:end);

    ObstacleDistanceData = [];

    for i = 1:length(FileNames)
        data = load([FileDir, '\', FileNames(i).name]);

        [~, arrOd] = analyze_obstacle_dist(data.dt, data.sim_pos_list, data.obs_pos_list, double(data.sim_kk), data.quadNum);

        % Remove NaNs
        validIdx = ~isnan(arrOd);
        arrOd = arrOd(validIdx);
        arrOd = min(arrOd);

        % Concatenate
        ObstacleDistanceData = [ObstacleDistanceData, arrOd];
    end

    McData(stepNum_i).ObstacleDistanceData = ObstacleDistanceData;
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

    ObstacleDistanceData = [];

    for i = 1:length(FileNames)
        data = load([FileDir, '\', FileNames(i).name]);

        [~, arrOd] = analyze_obstacle_dist(data.dt, data.sim_pos_list, data.obs_pos_list, double(data.sim_kk), data.quadNum);

        % Remove NaNs
        validIdx = ~isnan(arrOd);
        arrOd = arrOd(validIdx);
        arrOd = min(arrOd);

        % Concatenate
        ObstacleDistanceData = [ObstacleDistanceData, arrOd];
    end

    McData(Nbest_i).ObstacleDistanceData = ObstacleDistanceData;
end

end

