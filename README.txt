This folder includes simulation and hardware experiments for the paper "Onboard 3D Flocking of Micro Quadcopters via Single-Step Predictions in Cluttered Environments".

"Quadcopter Experiment" folder includes VICON trajectories and codes for simulating the experiment and plotting the metrics.
Run "run_quadcopter_experiment.m" first, then run "simulation.m" to simulate the experiment or run "flocking_analyze_test.m" to plot 
metrics.

"Quadcopter Dynamic Obstacle Experiment" folder includes VICON trajectories and codes for plotting the metrics of the dynamic obstacle
experiment, in which two quadcopters cross the arena on linear paths perpendicular to the migration direction as moving obstacles. The
paths are generated as cubic Bezier curves with collinear control points. Run "plot_dynamic_robot_metrics.m" to plot
the metrics and the trajectories of the test reported in the paper.
"onboard_and_obstacle_controller_vicon.py" is the Crazyswarm script that starts the experiment and flies the obstacle
quadcopters along their trajectories.

"Quadcopter Firmware Codes" folder includes flocking and HITL simulation firmware app layer
codes for Crazyflie 2.1 quadcopters. To see how to use these codes to control 
Crazyflie quadcopters onboard see: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/

"Simulations" folder consists of:
--"Cumulative Metric Plots" folder which includes MATLAB scripts to plot cumulative metrics.
--"Metrics of Scalability Experiments" folder which includes MATLAB scripts to display metric values of scalability experiments.
--"Single Step Prediction Flocking" folder which includes MATLAB scripts that contains implementation of the Predictive Search flocking method, and scripts that plot/animate trajectories of the robots and plot the metric values of the simulation.
--"Single Step Prediction Flocking with Dynamic Obstacles" folder which includes MATLAB scripts that contains the version of the Predictive Search flocking method used in the dynamic obstacle experiments.
--"Multi Step Prediction Flocking" folder which includes MATLAB scripts containing the multi step version of the Predictive Search flocking method for comparison
--"Potential Field Flocking" folder which includes MATLAB scripts that contains implementation of the Potential Field flocking method, and scripts that plot/animate trajectories of the robots and plot the metric values of the simulation.

"Videos" folder includes the recordings of the static obstacle and the dynamic obstacle experiments.


