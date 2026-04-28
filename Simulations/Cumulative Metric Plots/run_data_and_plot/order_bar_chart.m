
addPFMv = 1;

first1 = first1_data_PFM;
first1len = length (first1); 
first1method = 'SFM' + strings(1,first1len);
first1metric = 'Map 1' + strings(1,first1len);

first2 = first2_data_PFM;
first2len = length (first2); 
first2method = 'PFM' + strings(1,first2len);
first2metric = 'Map 1' + strings(1,first2len);


second1 = second1_data_PFM;
second1len = length (second1); 
second1method = 'SFM' + strings(1,second1len);
second1metric = 'Map 2' + strings(1,second1len);

second2 = second2_data_PFM;
second2len = length (second2); 
second2method = 'PFM' + strings(1,second2len);
second2metric = 'Map 2' + strings(1,second2len);


third1 = third1_data_PFM;
third1len = length (third1); 
third1method = 'SFM' + strings(1,third1len);
third1metric = 'Map 3' + strings(1,third1len);

third2 = third2_data_PFM;
third2len = length (third2); 
third2method = 'PFM' + strings(1,third2len);
third2metric = 'Map 3' + strings(1,third2len);

dataErrPlot1 = {first1,second1,third1};
dataErrPlot2 = {first2,second2,third2};


medianListTot = [0,0,0];
errListDownTot = [0,0,0];
errListUpTot = [0,0,0];

medianList = [];
errListDown = [];
errListUp = [];
for i=1:3


    Qval = mean(dataErrPlot2{i});
    stdQval = std(dataErrPlot2{i});
    medianList(end+1) = Qval;
    errListDown(end+1) = stdQval;
    errListUp(end+1) = stdQval;
end
medianListTot = vertcat(medianListTot,medianList);
errListDownTot = vertcat(errListDownTot,errListDown);
errListUpTot = vertcat(errListUpTot, errListUp);

medianList = [];
errListDown = [];
errListUp = [];
for i=1:3
    Qval = mean(dataErrPlot1{i});
    stdQval = std(dataErrPlot1{i});
    medianList(end+1) = Qval;
    errListDown(end+1) = stdQval;
    errListUp(end+1) = stdQval;
end
medianListTot = vertcat(medianListTot,medianList);
errListDownTot = vertcat(errListDownTot,errListDown);
errListUpTot = vertcat(errListUpTot, errListUp);

medianListTot = medianListTot(2:end,:);
errListDownTot = errListDownTot(2:end,:);
errListUpTot = errListUpTot(2:end,:);




x=categorical({'\rho_1';'\rho_2';'\rho_3'});
y=medianListTot';
errorplus=errListUpTot;
errorminus=errListDownTot;
figure;
bar(x,y);
hBar = bar(y, 0.8);
hBar(1).FaceColor = [0.4940 0.1840 0.5560];
hBar(2).FaceColor = [0.8500 0.3250 0.0980];

for k1 = 1:size(y,2)
    ctr(k1,:) = bsxfun(@plus, hBar(k1).XData, hBar(k1).XOffset');     
    ydt(k1,:) = hBar(k1).YData;
end
hold on
errorbar(ctr, ydt, errorplus, '.k', 'LineWidth', 2)
hold off
xlabel('Density','FontSize',2)
set(gca,'linewidth',1,'FontSize',22)
ylabel('\boldmath${\psi}$','FontSize',30,'Interpreter','latex' )

ylim([0 1.05]);
set(gca,'XTickLabel',x)
grid on
function stdErr = calcStdErr(x)
    stdErr = std(x);
    %stdErr = std(x)/sqrt(length(x));
end
