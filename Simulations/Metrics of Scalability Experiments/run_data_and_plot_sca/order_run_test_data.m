clear
clc
t_s = 3.0; % Transient period in seconds
k_s = t_s*20; % Transient time steps (\delta t = 0.05 -> 20 Hz)

experiment_num = 99;

%% first environment


first1_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/P1f/P1f', int2str(i), '.mat');
    load(file_name)
    first1_data_PFM = horzcat(first1_data_PFM, order_data_PFM(k_s:end));
end


first2_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/S1f/S1f', int2str(i), '.mat');
    load(file_name)
    first2_data_PFM = horzcat(first2_data_PFM, order_data_PFM(k_s:end));
end


first3_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/T1f/T1f', int2str(i), '.mat');
    load(file_name)
    first3_data_PFM = horzcat(first3_data_PFM, order_data_PFM(k_s:end));
end


%%second environment


second1_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/P2f/P2f', int2str(i), '.mat');
    load(file_name)
    second1_data_PFM = horzcat(second1_data_PFM, order_data_PFM(k_s:end));
end


second2_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/S2f/S2f', int2str(i), '.mat');
    load(file_name)
    second2_data_PFM = horzcat(second2_data_PFM, order_data_PFM(k_s:end));
end


second3_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/T2f/T2f', int2str(i), '.mat');
    load(file_name)
    second3_data_PFM = horzcat(second3_data_PFM, order_data_PFM(k_s:end));
end


%%third environment

third1_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/P3f/P3f', int2str(i), '.mat');
    load(file_name)
    third1_data_PFM = horzcat(third1_data_PFM, order_data_PFM(k_s:end));
end


third2_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/S3f/S3f', int2str(i), '.mat');
    load(file_name)
    third2_data_PFM = horzcat(third2_data_PFM, order_data_PFM(k_s:end));
end


third3_data_PFM = [];
for i=0:experiment_num
    file_name = strcat('sim_sca_data/T3f/T3f', int2str(i), '.mat');
    load(file_name)
    third3_data_PFM = horzcat(third3_data_PFM, order_data_PFM(k_s:end));
end


