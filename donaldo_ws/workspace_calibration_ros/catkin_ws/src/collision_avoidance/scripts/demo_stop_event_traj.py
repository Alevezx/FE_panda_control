#!/usr/bin/env python3
# coding=utf-8

import rospy
from copy import deepcopy
import numpy as np
import time
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionResult
from sensor_msgs.msg import JointState
from collision_avoidance.srv import *
import math
import roboticstoolbox as rtb
from spatialmath import SE3
import sys

# Parameters
traj_step = 0.01   # s
time_iter = 0.05   # s
toll_no_stop = 0.05   # rad
q_p_lim = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])    # rad/s
q_pp_lim = np.array([15, 7.5, 10, 12.5, 15, 20, 20])                            # rad/s^2

# Initialize variables
q_meas = []
t_exe = []
status = []
t_i = None
outfile_q = None
outfile_dist = None



def CallbackFollowJointTrajectoryResult(data, arg):
    arg.append(data.status.status)


def CallbackJointStates(data):
    global q_meas, outfile_q

    t = rospy.get_time()
    t_exe = t - t_i
 
    q_meas = data.position
    outfile_q.write(f"\t<point time='{t_exe}'>{q_meas[0]} {q_meas[1]} {q_meas[2]} {q_meas[3]} {q_meas[4]} {q_meas[5]} {q_meas[6]}</point>\n")


def CalcStopDurationClient(q_cur, q_p_cur, q_pp_cur):
    rospy.wait_for_service('calc_stop_duration')
    try:
        CalcStopDurationFunction = rospy.ServiceProxy('calc_stop_duration', CalcStopDuration)
        resp1 = CalcStopDurationFunction(q_cur, q_p_cur, q_pp_cur)
        return resp1.stop_duration
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def FlagStopClient(T_stop, q, q_p, q_pp):
    rospy.wait_for_service('flag_stop')
    try:
        FlagStopFunction = rospy.ServiceProxy('flag_stop', FlagStop)
        resp1 = FlagStopFunction(T_stop, q, q_p, q_pp)
        return resp1.flag_stop
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def TrajPoly5(qi, qi_p, qi_pp, qf, qf_p, qf_pp, traj_duration, time_step):
    coeff = np.zeros(6)

    coeff[0] = qi
    coeff[1] = qi_p
    coeff[2] = qi_pp / 2

    known_terms = np.array([qf - qi - qi_p * traj_duration - 0.5 * qi_pp * traj_duration**2, qf_p - qi_p - qi_pp * traj_duration, qf_pp - qi_pp])

    # Vandermonde matrix
    V = np.array([[traj_duration**3, traj_duration**4, traj_duration**5], [3 * traj_duration**2, 4 * traj_duration**3, 5 * traj_duration**4], [6 * traj_duration, 12 * traj_duration**2, 20 * traj_duration**3]])

    coeff_1 = np.linalg.solve(V, known_terms)

    coeff[3] = coeff_1[0]
    coeff[4] = coeff_1[1]
    coeff[5] = coeff_1[2]
    
    t = np.linspace(0, traj_duration, int(traj_duration / time_step + 1))

    # Position profile
    q = coeff[0] + coeff[1] * t + coeff[2] * t**2 + coeff[3] * t**3 + coeff[4] * t**4 + coeff[5] * t**5

    # Velocity profile
    q_p = coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t**2 + 4 * coeff[4] * t**3 + 5 * coeff[5] * t**4

    # Acceleration profile
    q_pp = 2 * coeff[2] + 6 * coeff[3] * t + 12 * coeff[4] * t**2 + 20 * coeff[5] * t**3

    traj = np.vstack((q, q_p, q_pp))
    
    return traj


def ExecuteTrajectory(q_traj, traj_duration, control_publisher):
    global q_meas, t_exe, status

    # Usefull variables
    flag_continue = 1   # flag to indicate that the final position isn't reached
    i = 0               # index of trajectory to final position

    traj_duration_new = round(traj_duration / traj_step) * traj_step    # current trajectory duration [s]
    added_time = 0                                                      # added time to precedent trajectory duration [s]

    q_c = deepcopy(q_traj[0])     # start position of current trajectory
    q_end = deepcopy(q_traj[1])   # final position of current trajectory

    q_p_lim = np.array([ 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
    q_pp_lim = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])

    status = []
  
    while(flag_continue):                
        # Initialization q, q_p and q_pp
        num_points = int(traj_duration_new / traj_step) + 1

        q = np.zeros((num_points, 7))
        q_p = np.zeros((num_points, 7))
        q_pp = np.zeros((num_points, 7)) 
    
        # Calculate complete trajectory
        for joint_num in range(7):
            traj_comp = TrajPoly5(q_c[joint_num], 0, 0, q_end[joint_num], 0, 0, traj_duration_new, traj_step)
            
            q[:, joint_num] = deepcopy(np.transpose(traj_comp[0, :]))
            q_p[:, joint_num] = deepcopy(np.transpose(traj_comp[1, :]))
            q_pp[:, joint_num] = deepcopy(np.transpose(traj_comp[2, :]))

            # Find out speed and acceleration limits to calculate added_time
            if not max(abs(traj_comp[1, :])) == 0.0:
                q_p_lim[joint_num] = (max(abs(traj_comp[1, :])))  
            
            if not max(abs(traj_comp[2, :])) == 0.0:
                q_pp_lim[joint_num] = (max(abs(traj_comp[2, :]))) 
                    
        # Write header message
        msg = FollowJointTrajectoryActionGoal()

        msg.header.stamp = rospy.Duration(0)
        msg.header.frame_id = ''

        msg.goal.trajectory.header.stamp = rospy.Duration(0)
        msg.goal.trajectory.header.frame_id = ''
        msg.goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']      
        
        # Write trajectory message
        point = JointTrajectoryPoint()
        point.positions = deepcopy(q_end)
        point.velocities = [0, 0, 0, 0, 0, 0, 0]
        point.accelerations = [0, 0, 0, 0, 0, 0, 0]
        point.effort = []
        point.time_from_start = rospy.Duration(traj_duration_new)
        msg.goal.trajectory.points.append(deepcopy(point))        
       
        # Status of movement on the trajectory
        status = []

        # Initialization subscriber node for /position_joint_trajectory_controller/follow_joint_trajectory/result
        sub_follow_joint_trajectory_result = rospy.Subscriber('/position_joint_trajectory_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, CallbackFollowJointTrajectoryResult, (status))
       
        # Move to configuration
        control_publisher.publish(msg)
        pub_time = rospy.get_time()

        # Wait the end of the trajectory or an error or a stop event
        stop_flag = 0

        if(len(q_meas) > 0):
            no_stop = np.all(abs(q_end - q_meas[-1]) < toll_no_stop)
        else:
            no_stop = 0

        rate = rospy.Rate(1/time_iter)

        while(len(status) == 0 and (not(stop_flag) or no_stop)): 
            # Calculate optimal stop duration
            traj_duration_prec = rospy.get_time() - pub_time + time_iter
            traj_index = int(traj_duration_prec / traj_step)
            if(traj_index < len(q)):
                q_c = deepcopy(q[traj_index])
                q_p_c = deepcopy(q_p[traj_index])
                q_pp_c = deepcopy(q_pp[traj_index])
              
                stop_duration = CalcStopDurationClient(deepcopy(q_c), deepcopy(q_p_c), deepcopy(q_pp_c)) 
            
            outfile_dist.write(f"\t<keypoint time='{t_exe}'>")
            outfile_dist.write(f"\t\t<point id='0'>{ee_pos[0]} {ee_pos[1]} {ee_pos[2]}</point>")
            outfile_dist.write(f"\t\t<point id='1'>{repul_final[0]} {repul_final[1]} {repul_final[2]}</point>")
            outfile_dist.write(f"\t</keypoint>")

            # Detect capsule collision
            if(FlagStopClient(stop_duration, q_c, q_p_c, q_pp_c)):
                stop_flag = 1
            
            if(len(q_meas) > 0):
                no_stop = np.all(abs(q_end - q_meas[-1]) < toll_no_stop)
            else:
                no_stop = 0
            
            rate.sleep()

        # If there was a stop event
        if(len(q_meas) > 0):
            do_stop = np.any(abs(q_end - q_meas[-1]) > toll_no_stop)
        else:
            do_stop = 1

        if (stop_flag and do_stop):
            print('\nIngaggio traiettoria di stop')

            # Write header message
            msg = FollowJointTrajectoryActionGoal()

            msg.header.stamp = rospy.Duration(0)
            msg.header.frame_id = ''

            msg.goal.trajectory.header.stamp = rospy.Duration(0)
            msg.goal.trajectory.header.frame_id = ''
            msg.goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

            # Write trajectory message
            point = JointTrajectoryPoint()
            point.positions = deepcopy(q_meas)
            point.velocities = [0, 0, 0, 0, 0, 0, 0]
            point.accelerations = [0, 0, 0, 0, 0, 0, 0]
            point.effort = []
            point.time_from_start = rospy.Duration(stop_duration)
            msg.goal.trajectory.points.append(deepcopy(point))

            # Move to stop configuration
            control_publisher.publish(msg)

            # Wait the end of the trajectory or an error
            while (status == [] or status == [2]):           
                FlagStopClient(stop_duration, q_c, q_p_c, q_pp_c)
                continue

            if(status[-1] == 3):
                print('Fine del movimento alla configurazione di stop')
            else:
                sys.exit('Errore nel movimento di tipo ' + str(status[-1]))
        else:
            # The final position is reached
            flag_continue = 0

            if(status[-1] == 3):
                print('Fine del movimento alla configurazione finale')
            else:
                sys.exit('Errore nel movimento di tipo ' + str(status[-1]))
        
        # Close subscribers
        sub_follow_joint_trajectory_result.unregister()

        # Update i
        i += 1

        # Update added_time 
        vect_t_vel = np.zeros((1, 7))
        np.divide(15 * (q_end - q_c), 8 * q_p_lim, vect_t_vel)
        t_vel = np.amax(abs(vect_t_vel))
   
        vect_t_acc = np.zeros((1, 7))
        np.divide(10 * (q_end - q_c), math.sqrt(3) * q_pp_lim, vect_t_acc)
        t_acc = np.amax(np.sqrt(abs(vect_t_acc)))
  
        a_time = max(t_vel, t_acc)

        if(a_time < 0.5):
            a_time = 0.5
        added_time = a_time
      
        # Update traj_duration
        traj_duration_new = round((added_time) / traj_step) * traj_step


def ExecuteTask(q_traj, traj_durations, control_publisher):
    global outfile_q, outfile_dist, t_i, status

    outfile_q = open("/home/panda/Documents/Data_Collision_Avoidance/demo_stop_event_control/q_robot.xml", 'w')
    outfile_q.write("<q>\n")

    outfile_dist = open("/home/panda/Documents/Data_Collision_Avoidance/demo_stop_event_control/dist_segment.xml", 'w')
    outfile_dist.write("<distance>\n")

    # Initialization subscriber node for /joint_states
    sub_joint_states = rospy.Subscriber('/joint_states', JointState, CallbackJointStates)

    t_i = rospy.get_time()
    traj_duration_prev = 0
    q_traj_prev = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    for i in range(len(traj_durations)):
        q_traj_pair = []
        q_traj_pair.append(deepcopy(q_traj_prev))
        q_traj_pair.append(deepcopy(q_traj[i]))
        ExecuteTrajectory(deepcopy(q_traj_pair), traj_durations[i] - traj_duration_prev, control_publisher)

        traj_duration_prev = traj_durations[i]
        q_traj_prev = q_traj[i]

    outfile_q.close()   

    # Close subscribers
    sub_joint_states.unregister()
              


if __name__ == '__main__':
    # Initialization node
    rospy.init_node('execute_trajectory')

    file = open("/home/panda/Desktop/demo/collision_avoidance/p_traj.txt", 'r')

    p_traj = []
    for line in file:
        v_s = line.split('\t')
        data = []
        for s in v_s:
            data.append(float(s))
        p_traj.append(data)
    
    file.close()

    file = open("/home/panda/Desktop/demo/collision_avoidance/t_traj.txt", 'r')

    t_traj = []
    for line in file:
        v_s = line.split('\t')
        data = []
        for s in v_s:
            t_traj.append(float(s))
    
    file.close()

    panda = rtb.models.DH.Panda()
    q_traj = np.zeros((len(p_traj), 7))
    q0 = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    for i in range(len(p_traj)):
        p = SE3(p_traj[i])*SE3.RPY([0.0, 0.0, math.pi], order='xyz')
 
        sol = panda.ikine_LM(p, q0=q0)                    
        q_traj[i, :] = np.matrix(sol.q)     

    # Initialization publisher node for /position_joint_trajectory_controller/follow_joint_trajectory/goal
    control_publisher = rospy.Publisher('/position_joint_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size = 10)

    # Needed to wait for the end of the node publication (publication isn't snapshot)
    time.sleep(0.4)

    print("This will move the robot to the home position")
    input("Press Enter to continue...")

    # Write header message
    msg = FollowJointTrajectoryActionGoal()

    msg.header.stamp = rospy.Duration(0)
    msg.header.frame_id = ''

    msg.goal.trajectory.header.stamp = rospy.Duration(0)
    msg.goal.trajectory.header.frame_id = ''
    msg.goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

    # Write trajectory message
    point = JointTrajectoryPoint()
    point.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    point.velocities = [0, 0, 0, 0, 0, 0, 0]
    point.accelerations = [0, 0, 0, 0, 0, 0, 0]
    point.effort = []
    point.time_from_start = rospy.Duration(5.0)
    msg.goal.trajectory.points.append(deepcopy(point))

    # Move to stop configuration
    control_publisher.publish(msg)
    time.sleep(5.0)
    print("Finished moving to initial joint configuration.")  

    print("WARNING: Collision thresholds are set to high values. ")
    print("Make sure you have the user stop at hand!")
    print("This will start the joint control task")
    input("Press Enter to continue...")

    t_i = rospy.get_time()
    
    ExecuteTask(deepcopy(np.array(q_traj)), deepcopy(np.array(t_traj)), control_publisher)

    print(f"Trajectory completed, {t_traj[-1]} seconds has passed, shutting down example")

