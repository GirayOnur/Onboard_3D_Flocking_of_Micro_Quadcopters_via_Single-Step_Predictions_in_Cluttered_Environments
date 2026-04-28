
addPFMv = 1;

first1 = first1_data_PFM;
first1len = length (first1); 
first1method = 'Scale 1' + strings(1,first1len);
first1metric = 'Map 1' + strings(1,first1len);

first2 = first2_data_PFM;
first2len = length (first2); 
first2method = 'Scale 2' + strings(1,first2len);
first2metric = 'Map 1' + strings(1,first2len);

first3 = first3_data_PFM;
first3len = length (first3); 
first3method = 'Scale 3' + strings(1,first3len);
first3metric = 'Map 1' + strings(1,first3len);



second1 = second1_data_PFM;
second1len = length (second1); 
second1method = 'Scale 1' + strings(1,second1len);
second1metric = 'Map 2' + strings(1,second1len);

second2 = second2_data_PFM;
second2len = length (second2); 
second2method = 'Scale 2' + strings(1,second2len);
second2metric = 'Map 2' + strings(1,second2len);

second3 = second3_data_PFM;
second3len = length (second3); 
second3method = 'Scale 3' + strings(1,second3len);
second3metric = 'Map 2' + strings(1,second3len);



third1 = third1_data_PFM;
third1len = length (third1); 
third1method = 'Scale 1' + strings(1,third1len);
third1metric = 'Map 3' + strings(1,third1len);

third2 = third2_data_PFM;
third2len = length (third2); 
third2method = 'Scale 2' + strings(1,third2len);
third2metric = 'Map 3' + strings(1,third2len);

third3 = third3_data_PFM;
third3len = length (third3); 
third3method = 'Scale 3' + strings(1,third3len);
third3metric = 'Map 3' + strings(1,third3len);

dataErrPlot1 = {first1,second1,third1};
dataErrPlot2 = {first2,second2,third2};
dataErrPlot3 = {first3,second3,third3};


medianListTot = [0,0,0];
errListDownTot = [0,0,0];
errListUpTot = [0,0,0];



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
    Qval = mean(dataErrPlot3{i});
    stdQval = std(dataErrPlot3{i});
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

medianListTable = round(reshape(medianListTot, 1, []),2);
medianListErr = round(reshape(errListUpTot, 1, []),2);

swarm_size = {'N=5'; 'N=20'; 'N=80'};
order_table1 = table(swarm_size,medianListTable(1:3)', medianListErr(1:3)', 'VariableNames',["Swarm Size","Mean", "Deviation"]);
order_table2 = table(swarm_size,medianListTable(4:6)', medianListErr(4:6)', 'VariableNames',["Swarm Size","Mean", "Deviation"]);
order_table3 = table(swarm_size,medianListTable(7:9)', medianListErr(7:9)', 'VariableNames',["Swarm Size","Mean", "Deviation"]);

disp("            Density = 0.06m^-2")
disp(order_table1)
disp("            Density 0.12m^-2")
disp(order_table2)
disp("            Density 0.20m^-2")
disp(order_table3)

