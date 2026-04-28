%transforms vicon trajectory data to .mat file for plotting the
%trajectories and metric plots

clear
clc

filename = "vicon_logs\Quadcopter_Experiment_Trajectories_120.csv";
pos_log_vicon = readmatrix(filename);
dt = 0.05;
vicon_hz = 120;
time_scale = 120*dt;


pos_log_vicon = pos_log_vicon(6:end,3:17);

sim_kk = floor(length(pos_log_vicon)/time_scale);

pos_log_sim = [];

for i=1:sim_kk-1
    pos_log_sim = [pos_log_sim;pos_log_vicon(time_scale*i,:)];
end
pos_log_sim = pos_log_sim./1000;
save('pos_log_vicon','pos_log_sim')