#!/usr/bin/env python3

#Starts the dynamic obstacle experiment, the flocking robots run the onboard
#firmware while the obstacle quadcopters are flown along cubic Bezier curves

import numpy as np
from scipy.io import savemat
from pycrazyswarm import *
from datetime import datetime

def cubic_bezier_curve(t, P0, P1, P2, P3):
    """
    Generate a cubic Bezier curve that traces P0 and P3, running smoothly near control points P1 and P2.
    """
    B0 = (1 - t)**3
    B1 = 3 * t * (1 - t)**2
    B2 = 3 * t**2 * (1 - t)
    B3 = t**3
    return P0 * B0 + P1 * B1 + P2 * B2 + P3 * B3

def generate_bezier_points(P0, P1, P2, P3, num_points):
    """
    Sample the cubic Bezier curve at num_points equally spaced parameter values.
    """
    t_values = np.linspace(0, 1, num_points)
    points = [cubic_bezier_curve(t, P0, P1, P2, P3) for t in t_values]
    return points

if __name__ == "__main__":

    # Set up configuration
    sleepRate = 10 #Hz
    report_rate = 10 #steps

    # Enable/disable obstacle and agent operation
    obstacles_enabled = True
    agents_enabled = True

    agent_ids = [1,2,3,4,5] #flocking quadcopters
    obs_ids = [6,7] #obstacle quadcopters

    quadNum = len(agent_ids) #number of quadcopters
    obsNum = len(obs_ids) #number of obstacles

    target_height_values = [0.40,1.05] #obstacle altitudes in meters, the obstacles fly below and above the reference altitude
    takeoff_duration = 2.5 #seconds, from crazyflie_psm_*.c firmware code
    landing_height = 0.03 #meters, from crazyflie_psm_*.c firmware code
    landing_duration = 3.0 #seconds, from crazyflie_psm_*.c firmware code
    obstacle_navigation_iters = 160 #number of waypoints on each obstacle trajectory
    agent_finish_distance = 3.5 #y distance in meters travelled to terminate the loop
    obstacle_finish_distance = -2.0 #y distance in meters travelled to stop obstacle navigation
    obstacle_hover_pre_delay = 1.1 #seconds, from crazyflie_psm_*.c firmware code
    obstacle_hover_post_delay = 3.0 #seconds, from crazyflie_psm_*.c firmware code
    landing_delay = 10.0 #seconds

    # Generate obstacle trajectories
    bezier_control_points = [] #four control points per obstacle
    bezier_control_points.append(
        np.array([
            [-0.75,0],
            [-0.25,0],
            [0.25,0],
            [0.75,0]
        ])
    )

    bezier_control_points.append(
        np.array([
            [0.75,1],
            [0.25,1],
            [-0.25,1],
            [-0.75,1]
        ])
    )

    obstacle_trajectories = [np.asarray(generate_bezier_points(*bezier_control_points[i], obstacle_navigation_iters)) for i in range(len(bezier_control_points))]

    # Position memory arrays
    obs_pos_list_i = np.zeros((obsNum,3))
    quad_pos_list_i = np.zeros((quadNum,3))

    obs_pos_log = np.zeros((500,obsNum*3)) #obstacle position log history
    agent_pos_log = np.zeros((500,quadNum*3)) #robot position log history

    # Start crazyflie server
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Start experiment
    try:
        file_label = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
        if agents_enabled:
            for crazyflie_id in agent_ids:
                allcfs.crazyfliesById[crazyflie_id].setParam("fmodes/ufs",int(1)) #signal the onboard controller to start flocking
            print("Starting experiment.")

        # Initialize flags
        obstacles_navigating = True
        agents_navigating = True
        obstacle_hover_state = "pre-delay"
        time_iters = 0 #time = time_iters/sleepRate
        obs_waypoint_id = 0
        # Main loop
        while (agents_navigating and (time_iters<400)):
            # Access crazyflies
            ## Access agents
            if agents_enabled:
                for crazyflie_id in agent_ids:
                    # Check agent position
                    quad_pos_list_i[agent_ids.index(crazyflie_id)] = allcfs.crazyfliesById[crazyflie_id].position()
            ## Access obstacles
            if obstacles_enabled:
                for crazyflie_id in obs_ids: #obstacle indices must follow AFTER agent indices, modify if otherwise
                    # Check obstacle position
                    obs_pos_list_i[obs_ids.index(crazyflie_id)] = allcfs.crazyfliesById[crazyflie_id].position()

                # Obstacle finite state machine
                if obstacle_hover_state == "pre-delay":
                    obstacle_hover_state = "liftoff" if time_iters >= obstacle_hover_pre_delay*sleepRate else "pre-delay"
                if obstacle_hover_state == "liftoff":
                    for crazyflie_id in obs_ids:
                        allcfs.crazyfliesById[crazyflie_id].takeoff(target_height_values[obs_ids.index(crazyflie_id)],takeoff_duration)
                    obstacle_hover_state = "post-delay"
                if obstacle_hover_state == "post-delay":
                    obstacle_hover_state = "hovering" if time_iters >= (obstacle_hover_pre_delay + obstacle_hover_post_delay)*sleepRate else "post-delay"
                # Obstacle waypoint navigation
                if obstacle_hover_state == "hovering":
                    try:
                        for crazyflie_id in obs_ids:
                            destination = obstacle_trajectories[obs_ids.index(crazyflie_id)][obs_waypoint_id]
                            allcfs.crazyfliesById[crazyflie_id].cmdPosition(pos= [destination[0],destination[1],target_height_values[obs_ids.index(crazyflie_id)]])
                    except IndexError:
                        #the obstacle reached the end of its trajectory, hold the last waypoint
                        for crazyflie_id in obs_ids:
                            print(f"Trajectory ID: {obs_ids.index(crazyflie_id)}, Waypoint: {obs_waypoint_id}")
                            destination = obstacle_trajectories[obs_ids.index(crazyflie_id)-quadNum][-1]
                            allcfs.crazyfliesById[crazyflie_id].cmdPosition(pos= [destination[0],destination[1],target_height_values[obs_ids.index(crazyflie_id)]])

            # Pre-loop
            ## Flag checks and memory management
            agent_pos_log[time_iters,:] = quad_pos_list_i.flatten()
            obs_pos_log[time_iters,:] = obs_pos_list_i.flatten()
            agents_navigating = False if np.any(quad_pos_list_i[:,1] > agent_finish_distance) else True
            obstacles_navigating = False if np.any(obs_pos_list_i[:,1] < obstacle_finish_distance) else True

            ## Iteration and timing
            time_iters += 1
            obs_waypoint_id = obs_waypoint_id+1 if (obstacles_navigating and obstacle_hover_state == "hovering") else obs_waypoint_id
            timeHelper.sleepForRate(sleepRate)
            if time_iters % report_rate == 0:
                print(f"Current time step: {time_iters}, Bezier step: {obs_waypoint_id}, Agents up: {agents_navigating}, Obstacles Up: {obstacles_navigating}")

        # Landing
        ## Land obstacles by teleoperation
        print("Landing sequence initiated")
        if obstacles_enabled:
            for crazyflie_id in obs_ids:
                try:
                    allcfs.crazyfliesById[crazyflie_id].notifySetpointsStop()
                    allcfs.crazyfliesById[crazyflie_id].land(targetHeight= landing_height,duration= landing_duration)
                except rospy.service.ServiceException:
                    print(f"Obstacle {crazyflie_id} cannot land. Check mocap input.")
        ## Signal the onboard controller for the landing sequence
        if agents_enabled:
            for crazyflie_id in agent_ids:
                try:
                    allcfs.crazyfliesById[crazyflie_id].setParam("fmodes/ufs",int(4))
                except rospy.service.ServiceException:
                    print(f"Agent {crazyflie_id} cannot receive landing instructions.")
        timeHelper.sleep(landing_delay)

        print("Ending experiment.")

        savemat(f'obs_pos_log_{file_label}.mat', {"pos":obs_pos_log})
        savemat(f'agent_pos_log_{file_label}.mat', {"pos":agent_pos_log})

    except Exception as e:
        print(f"Emergency landing initiated: {e}")
        ## Land obstacles by teleoperation
        if obstacles_enabled:
            for crazyflie_id in obs_ids:
                try:
                    allcfs.crazyfliesById[crazyflie_id].land(landing_height,landing_duration)
                except rospy.service.ServiceException:
                    print(f"Obstacle {crazyflie_id} cannot land. Check mocap input.")
        ## Signal the onboard controller for the landing sequence
        if agents_enabled:
            for crazyflie_id in agent_ids:
                try:
                    allcfs.crazyfliesById[crazyflie_id].setParam("fmodes/ufs",int(4))
                except rospy.service.ServiceException:
                    print(f"Agent {crazyflie_id} cannot receive landing instructions.")
        timeHelper.sleep(landing_delay)

        print("Saving files.")
        savemat(f'obs_pos_log_{file_label}.mat', {"pos":obs_pos_log})
        savemat(f'agent_pos_log_{file_label}.mat', {"pos":agent_pos_log})
