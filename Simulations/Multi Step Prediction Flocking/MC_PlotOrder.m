function MC_PlotOrder()
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
    meanVals(i) = mean(McData(i).OrderData);
    stdVals(i)  = std(McData(i).OrderData);
end

% Create ordered categorical labels (ρ₁ ... ρ₁₀)
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
    McData(1).OrderData', ...
    McData(2).OrderData', ...
    McData(3).OrderData', ...
    McData(4).OrderData', ...
    McData(5).OrderData', ...
    McData(6).OrderData', ...
    McData(7).OrderData'],'Notch','on','Labels',{'1','2','3','4','5','6','7'});

end

function McData = plotStep(stepNumArr, Nbest)

% Collect Data
McData = struct();

for stepNum_i = 1:length(stepNumArr)
    stepNum = stepNumArr(stepNum_i);

    FileDir = [pwd, '\logs\stepNum', num2str(stepNum), '\Nbest', num2str(Nbest)];

    FileNames = dir(FileDir);
    FileNames = FileNames(3:end);

    OrderData = [];

    for i = 1:length(FileNames)
        data = load([FileDir, '\', FileNames(i).name]);

        [~, arrOrder] = analyze_order_v(data.dt, data.sim_vel_list, double(data.sim_kk), data.quadNum, data.migVel);

        % Remove NaNs
        validIdx = ~isnan(arrOrder);
        arrOrder = arrOrder(validIdx);

        % Concatenate
        OrderData = [OrderData, arrOrder];
    end

    McData(stepNum_i).OrderData = OrderData;
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

    OrderData = [];

    for i = 1:length(FileNames)
        data = load([FileDir, '\', FileNames(i).name]);

        [~, arrOrder] = analyze_order_v(data.dt, data.sim_vel_list, double(data.sim_kk), data.quadNum, data.migVel);

        % Remove NaNs
        validIdx = ~isnan(arrOrder);
        arrOrder = arrOrder(validIdx);

        % Concatenate
        OrderData = [OrderData, arrOrder];
    end

    McData(Nbest_i).OrderData = OrderData;
end

end

