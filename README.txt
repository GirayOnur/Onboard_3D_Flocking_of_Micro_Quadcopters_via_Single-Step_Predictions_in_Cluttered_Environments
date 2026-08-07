This folder includes simulation and harware experiments for the paper "Onboard 3D Flocking of Micro Quadcopters via Single-Step Predictions in Cluttered Environments" submitted to IEEE Robotics and Automation Letters (RA-L).

"Quadcopter Experiment" folder includes VICON trajectories and codes for simulating the experiment and plotting the metrics.
Run "run_quadcopter_experiment.m" first, then run "simulation.m" to simulate the experiment or run "flocking_analyze_test.m" to plot 
metrics.

"Quadcopter Dynamic Obstacle Experiment" folder includes VICON trajectories and codes for plotting the metrics of the dynamic obstacle
experiment, in which two quadcopters are flown along cubic Bezier curves as moving obstacles. Run "plot_dynamic_robot_metrics.m" to plot
the metrics and the trajectories. The test to be plotted is selected with the "filename" variable, the "idx_start" and "idx_stop" variables
are tuned for Test 5. "onboard_and_obstacle_controller_vicon.py" is the Crazyswarm script that starts the experiment and flies the obstacle
quadcopters along their trajectories.

"Quadcopter Firmware Codes" folder includes flocking and hitl simulation firmware app layer
codes for Crazyflie 2.1 quadcopters. To see how to use these codes to control 
Crazyflie quadcopters onboard see: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/

"Simulations" folder consists of:
--"Cumulative Metric Plots" folder which includes MATLAB scripts to plot cumulative metrics.
--"Metrics of Scalability Experiments" folder which includes MATLAB scripts to display metric values of scalability experiments.
--"Single Step Prediction Flocking" folder which includes MATLAB scripts that contains implementation of the Predictive Search flocking method, and scripts that plot/animate trajectories of the robots and plot the metric values of the simulation.
--"Single Step Prediction Flocking with Dynamic Obstacles" folder which includes MATLAB scripts that contains the version of the Predictive Search flocking method used in the dynamic obstacle experiments, where the obstacles move along cubic Bezier curves. Run "single_step_predictive_search_method_dyn.m", it plots the metric values and the trajectories of the simulation and then animates the robots together with the moving obstacles.
--"Multi Step Prediction Flocking" folder which includes MATLAB scripts containing the multi step version of the Predictive Search flocking method for comparison
--"Potential Field Flocking" folder which includes MATLAB scripts that contains implementation of the Potential Field flocking method, and scripts that plot/animate trajectories of the robots and plot the metric values of the simulation.

"Videos" folder includes the recordings of the static obstacle and the dynamic obstacle experiments.


