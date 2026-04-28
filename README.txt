This folder includes simulation and harware experiments for the paper "X" submitted to IEEE Robotics and Automation Letters (RA-L).

"Quadcopter Experiment" folder includes VICON trajectories and codes for simulating the experiment and plotting the metrics.
Run "run_quadcopter_experiment.m" first, then run "simulation.m" to simulate the experiment or run "flocking_analyze_test.m" to plot 
metrics.

"Quadcopter Firmware Codes" folder includes flocking and hitl simulation firmware app layer 
codes for Crazyflie 2.1 quadcopters. To see how to use these codes to control 
Crazyflie quadcopters onboard see: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/

"Simulations" folder consists of:
--"Cumulative Metric Plots" folder which includes MATLAB scripts to plot cumulative metrics.
--"Metrics of Scalability Experiments" folder which includes MATLAB scripts to display metric values of scalability experiments.
--"Single Step Prediction Flocking" folder which includes MATLAB scripts that contains implementation of the Predictive Search flocking method, and scripts that plot/animate trajectories of the robots and plot the metric values of the simulation.
--"Multi Step Prediction Flocking" folder which includes MATLAB scripts containing the multi step version of the Predictive Search flocking method for comparison
--"Potential Field Flocking" folder which includes MATLAB scripts that contains implementation of the Potential Field flocking method, and scripts that plot/animate trajectories of the robots and plot the metric values of the simulation.


