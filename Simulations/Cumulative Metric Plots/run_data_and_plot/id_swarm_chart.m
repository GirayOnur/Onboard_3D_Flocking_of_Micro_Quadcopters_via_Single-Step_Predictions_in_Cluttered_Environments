
addPFMv = 0;

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


dataarr = horzcat(first1, first2, second1, second2, third1, third2);
methodarr = horzcat(first1method, first2method, second1method, second2method, third1method, third2method);
envarr = horzcat(first1metric, first2metric, second1metric, second2metric, third1metric, third2metric);


datatable = table(methodarr', envarr', dataarr','VariableNames',{'Method' 'Environment' 'Data'});





s2 = swarmchart(1*ones(1,100),first2,30,[0.4940 0.1840 0.5560],'filled');
s2.XJitter = 'rand';
s2.XJitterWidth = 0.03;
hold on
s3 = swarmchart(1.1*ones(1,100),first1,30,[0.8500 0.3250 0.0980],'filled');
s3.XJitter = 'rand';
s3.XJitterWidth = 0.03;


s2 = swarmchart(2*ones(1,100),second2,30,[0.4940 0.1840 0.5560],'filled');
s2.XJitter = 'rand';
s2.XJitterWidth = 0.03;
s3 = swarmchart(2.1*ones(1,100),second1,30,[0.8500 0.3250 0.0980],'filled');
s3.XJitter = 'rand';
s3.XJitterWidth = 0.03;


s2 = swarmchart(3*ones(1,100),third2,30,[0.4940 0.1840 0.5560],'filled');
s2.XJitter = 'rand';
s2.XJitterWidth = 0.03;
s3 = swarmchart(3.1*ones(1,100),third1,30,[0.8500 0.3250 0.0980],'filled');
s3.XJitter = 'rand';
s3.XJitterWidth = 0.03;

line([0.75,3.25],[0.3,0.3],'Color','red','LineStyle','--', 'LineWidth', 1.5)

line([0.75,3.25],[0.2,0.2],'Color','black','LineStyle','--', 'LineWidth', 1.5)


ylim([0,0.6])

xticks([1 2 3])
xticklabels({'\rho_1','\rho_2','\rho_3'})
xlabel('Density','FontSize',2)
grid on
%{
envsOd = {'Map 1', 'Map 2', 'Map 3'};
datatable.Environment = categorical(datatable.Environment,envsOd);
%boxchart(datatable.Environment,datatable.Data,'GroupByColor',datatable.Method,'JitterOutliers','on');

swarmchart(datatable.Environment,datatable.Data,20,'c','filled')
%}
set(gca,'fontsize', 22);
ylabel('min\boldmath$(m_{r})\,[m]$','Interpreter','latex','fontsize', 30)


