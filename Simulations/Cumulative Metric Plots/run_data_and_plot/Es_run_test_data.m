clear
clc
t_s = 3.0;
kk_s = t_s*20;
kk_ss = t_s*100;

experiment_num = 99;

%% first environment


first1_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_data/S1f/S1f', int2str(i), '.mat');
    load(file_name)
    first1_data_PFM = horzcat(first1_data_PFM, speed_error_data_PFM(kk_ss:end));
end


first2_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_data/P1f/P1f', int2str(i), '.mat');
    load(file_name)
    first2_data_PFM = horzcat(first2_data_PFM, speed_error_data_PFM(kk_s:end));
end


first3_data_PFM = [];


%%second environment


second1_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_data/S2f/S2f', int2str(i), '.mat');
    load(file_name)
    second1_data_PFM = horzcat(second1_data_PFM, speed_error_data_PFM(kk_ss:end));
end


second2_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_data/P2f/P2f', int2str(i), '.mat');
    load(file_name)
    second2_data_PFM = horzcat(second2_data_PFM, speed_error_data_PFM(kk_s:end));
end


second3_data_PFM = [];



%%third environment

third1_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_data/S3f/S3f', int2str(i), '.mat');
    load(file_name)
    third1_data_PFM = horzcat(third1_data_PFM, speed_error_data_PFM(kk_ss:end));
end


third2_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_data/P3f/P3f', int2str(i), '.mat');
    load(file_name)
    third2_data_PFM = horzcat(third2_data_PFM, speed_error_data_PFM(kk_s:end));
end


third3_data_PFM = [];


