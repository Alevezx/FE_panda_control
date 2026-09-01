#!/usr/bin/env python3
# coding=utf-8

import cv2
import math
from math import cos, sin, pi
import numpy as np
from copy import deepcopy
import rospy
from collision_avoidance.srv import FlagStop, FlagStopResponse
import time
import sys
from bs4 import BeautifulSoup
import threading
import socket


skel_index = []
skel_index.append([0, 1])
skel_index.append([2, 3])
skel_index.append([3, 4])
skel_index.append([5, 6])
skel_index.append([6, 7])
skel_index.append([1, 8])
skel_index.append([9, 10])
skel_index.append([10, 11])
skel_index.append([12, 13])
skel_index.append([13, 14])	
MOD_vel = False
old_segment = []
t_start = 0.0
t_exe = 0.0
skel_list = []
robot_capsules = []   
rv = [0.085, 0.085, 0.06, 0.065]  # rv robot shell
T_reaction = 0.05  # reaction time, con safety controller a 200 Hz
csi = 0  # Zd+Zr; costante
max_human_speed = 1.6
max_robot_speed = 2
skeleton = []
t1 = None
do_once = True





def capsule_calculation(T_stop, q, q_p, q_pp, C_h, C_r):
    global rv, T_reaction, csi, max_human_speed, max_robot_speed, DH, old_segment

    Sh = max_human_speed*(T_reaction + T_stop)

    n_samples = T_stop*1000/ 5 # sample ogni 5 ms
        
    # ricalcolo la traiettoria di stop con n_samples punti:
    num_points = int( T_stop*1000/ n_samples) + 1 

    qi = np.zeros((num_points, 7))
    qi_p = np.zeros((num_points, 7))
    qi_pp = np.zeros((num_points, 7))

    q = np.array([q[0], q[1], q[2], q[3], q[4], q[5], q[6]])
    q_p = np.array([q_p[0], q_p[1], q_p[2], q_p[3], q_p[4], q_p[5], q_p[6]])
    q_pp = np.array([q_pp[0], q_pp[1], q_pp[2], q_pp[3], q_pp[4], q_pp[5], q_pp[6]])

    for k in range(7):
        traj = TrajPoly5(q[k], q_p[k], q_pp[k], q[k], 0, 0, T_stop, n_samples)
       
        qi[:, k] = deepcopy(np.transpose(traj[0, :]))
        qi_p[:, k] = deepcopy(np.transpose(traj[1, :]))
        qi_pp[:, k] = deepcopy(np.transpose(traj[2, :]))
    
    vel_max_1 = []
    vel_max_2 = []
    vel_max_3 = []
    vel_max_4 = []
    
    for m in range(len(qi)):  # per ogni step della traiettoria di stop calcolo la velocita' massima dei punti estremi di ogni capsula
        q_stop = np.matrix([[qi[m, 0]],
                            [qi[m, 1]],
                            [qi[m, 2]],
                            [qi[m, 3]],
                            [qi[m, 4]],
                            [qi[m, 5]],
                            [qi[m, 6]]])

        qp_stop = np.matrix([[qi_p[m, 0]],
                             [qi_p[m, 1]],
                             [qi_p[m, 2]],
                             [qi_p[m, 3]],
                             [qi_p[m, 4]],
                             [qi_p[m, 5]],
                             [qi_p[m, 6]]])

        vel_max = compute_max_vel_capsule(q_stop, qp_stop, DH, rv, C_h, C_r)
        vel_max_1.append(vel_max[0])
        vel_max_2.append(vel_max[1])
        vel_max_3.append(vel_max[2])
        vel_max_4.append(vel_max[3])
        
        dt_stop = np.linspace(0, T_stop, len(qi))

    Sr_1 = vel_max_1[0]*T_reaction
    Sr_2 = vel_max_2[0]*T_reaction
    Sr_3 = vel_max_3[0]*T_reaction
    Sr_4 = vel_max_4[0]*T_reaction 

    # integro sulla traiettoria di stop
    Ss_1 = trapz(dt_stop, vel_max_1)
    Ss_2 = trapz(dt_stop, vel_max_2)
    Ss_3 = trapz(dt_stop, vel_max_3)
    Ss_4 = trapz(dt_stop, vel_max_4)
    
    # calcolo i raggi delle capsule r_sw
    r_sw_1 = rv[0] + Sh + Sr_1 + Ss_1 + csi
    r_sw_2 = rv[1] + Sh + Sr_2 + Ss_2 + csi
    r_sw_3 = rv[2] + Sh + Sr_3 + Ss_3 + csi
    r_sw_4 = rv[3] + Sh + Sr_4 + Ss_4 + csi

    r_sw = [r_sw_1, r_sw_2, r_sw_3, r_sw_4]  # raggi delle 4 capsule del robot 
    
    return r_sw



def trapz(dt_stop, vel_max):
    Area = 0

    for i in range(1, len(dt_stop)):
        A = (dt_stop[i]-dt_stop[i-1])*(vel_max[i]+vel_max[i-1])/2
        
        Area = Area + A
        
    return Area



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
    
    t = np.linspace(0, traj_duration, int(traj_duration*1000 / time_step + 1))
    
    q = coeff[0] + coeff[1] * t + coeff[2] * t**2 + coeff[3] * t**3 + coeff[4] * t**4 + coeff[5] * t**5
    q_p = coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t**2 + 4 * coeff[4] * t**3 + 5 * coeff[5] * t**4
    q_pp = 2 * coeff[2] + 6 * coeff[3] * t + 12 * coeff[4] * t**2 + 20 * coeff[5] * t**3

    traj = np.vstack((q, q_p, q_pp))
    
    return traj
    


def compute_max_vel_capsule(q, qp, DH, rv, C_h, C_r):
    norm = np.linalg.norm
        
    C = C_h - C_r
    C_norm = norm(C)

    ################ 1-0
    A_1 = compute_capsule_end_point(DH, q, 1)
    B_1 = np.zeros(3)
        
    #calculate Jacobian for capsule 1-0 
    J = Jacobian(q[0], qp[0], DH[0, :], A_1)
    Jp = J[0:3, :]
    A_1_p = np.dot(Jp, qp[0])
    A_1_p = np.array([A_1_p[0, 0], A_1_p[1, 0], A_1_p[2, 0]])
                       
    Jo = J[3:6, :]
    omega_1 = np.dot(Jo, qp[0])
    omega_1 = np.array([omega_1[0, 0], omega_1[1, 0], omega_1[2, 0]])
    
    A_e1_p = A_1_p + np.cross(omega_1, ((A_1-B_1)/(norm(A_1-B_1))*rv[0]))
    B_e1_p = A_1_p + np.cross(omega_1, ((B_1-A_1)/(norm(B_1-A_1))*(norm(B_1-A_1) + rv[0])))
    
    vel_max_1_A = np.dot(A_e1_p, C)/C_norm
    vel_max_1_B = np.dot(B_e1_p, C)/C_norm
    
    vel_max_1 = max(vel_max_1_A, vel_max_1_B)
        
    if vel_max_1 < 0:
        vel_max_1 = 0  
        
    ################# 2-1
    A_2 = compute_capsule_end_point(DH, q, 3)
    B_2 = compute_capsule_end_point(DH, q, 2) 

    #calculate Jacobian for capsule 2-1 
    J = Jacobian(q[0:3], qp[0:3], DH[0:3, :], A_2)
    Jp = J[0:3, :]
    A_2_p = np.dot(Jp, qp[0:3])
    A_2_p = np.array([A_2_p[0, 0], A_2_p[1, 0], A_2_p[2, 0]])
    
    Jo = J[3:6, :]
    omega_2 = np.dot(Jo, qp[0:3])
    omega_2 = np.array([omega_2[0, 0], omega_2[1, 0], omega_2[2, 0]])
       
    A_e2_p = A_2_p + np.cross(omega_2, ((A_2-B_2)/(norm(A_2-B_2))*rv[1]))
    B_e2_p = A_2_p + np.cross(omega_2, ((B_2-A_2)/(norm(B_2-A_2))*(norm(B_2-A_2) + rv[1])))

    vel_max_2_A = np.dot(A_e2_p, C)/C_norm
    vel_max_2_B = np.dot(B_e2_p, C)/C_norm
    
    vel_max_2 = max(vel_max_2_A, vel_max_2_B)
    
    if vel_max_2 < 0:
        vel_max_2 = 0
        
    ################ 3-2
    A_3 = compute_capsule_end_point(DH, q, 5)
    B_3 = compute_capsule_end_point(DH, q, 4)
    
    #calculate Jacobian for capsule 3-2 
    J = Jacobian(q[0:5], qp[0:5], DH[0:5, :], A_3)
    Jp = J[0:3, :]
    A_3_p = np.dot(Jp, qp[0:5])
    A_3_p = np.array([A_3_p[0, 0], A_3_p[1, 0], A_3_p[2, 0]])
    
    Jo = J[3:6, :]
    omega_3 = np.dot(Jo, qp[0:5])
    omega_3 = np.array([omega_3[0, 0], omega_3[1, 0], omega_3[2, 0]])
    
    A_e3_p = A_3_p + np.cross(omega_3, ((A_3-B_3)/(norm(A_3-B_3))*rv[2]))
    B_e3_p = A_3_p + np.cross(omega_3, ((B_3-A_3)/(norm(B_3-A_3))*(norm(B_3-A_3) + rv[2])))

    vel_max_3_A = np.dot(A_e3_p, C)/C_norm
    vel_max_3_B = np.dot(B_e3_p, C)/C_norm
    
    vel_max_3 = max(vel_max_3_A, vel_max_3_B)
    
    if vel_max_3 < 0:
        vel_max_3 = 0
     
    ################ 5-4
    A_4 = compute_capsule_end_point(DH, q, 7)
    B_4 = compute_capsule_end_point(DH, q, 6)

    #calculate Jacobian for capsule 5-4 
    J = Jacobian(q[0:7], qp[0:7], DH[0:7, :], A_4)
    Jp = J[0:3, :]
    A_4_p = np.dot(Jp, qp[0:7])
    A_4_p = np.array([A_4_p[0, 0], A_4_p[1, 0], A_4_p[2, 0]])
    
    Jo = J[3:6, :]
    omega_4 = np.dot(Jo, qp[0:7])
    omega_4 = np.array([omega_4[0, 0], omega_4[1, 0], omega_4[2, 0]])
    
    A_e4_p = A_4_p + np.cross(omega_4, ((A_4-B_4)/(norm(A_4-B_4))*rv[3]))
    B_e4_p = A_4_p + np.cross(omega_4, ((B_4-A_4)/(norm(B_4-A_4))*(norm(B_4-A_4) + rv[3])))
    
    vel_max_4_A = np.dot(A_e4_p, C)/C_norm
    vel_max_4_B = np.dot(B_e4_p, C)/C_norm
    
    vel_max_4 = max(vel_max_4_A, vel_max_4_B)
    
    if vel_max_4 < 0:
        vel_max_4 = 0
   
    vel_max = []
    vel_max.append(vel_max_1)
    vel_max.append(vel_max_2)
    vel_max.append(vel_max_3)
    vel_max.append(vel_max_4)

    return vel_max
 


def Jacobian(q, dq, DH, A_k): 
    z0 = np.array([[0],
                   [0], 
                   [1]])

    p0 = np.array([[0],
                   [0],
                   [0], 
                   [1]])
        
    J = np.zeros((6, len(q)))
    
    R_k = np.identity(3)         
    T_k = np.identity(4)

    if len(q) == 1:        
        zj = z0 
        zj = np.array([zj[0, 0], zj[1, 0], zj[2, 0]])
        pj = p0
        pj = np.array([pj[0, 0], pj[1, 0], pj[2, 0]])
        p = A_k
                
        J_k = np.cross(zj, p - pj)
        J[0:3, 0] = np.array([J_k[0], J_k[1], J_k[2]])
        J[3:6, 0] = np.array([zj[0], zj[1], zj[2]])

    else:
        zj = z0 
        zj = np.array([zj[0, 0], zj[1, 0], zj[2, 0]])
        pj = p0
        pj = np.array([pj[0, 0], pj[1, 0], pj[2, 0]])
        p = A_k
        
        J_k = np.cross(zj, p - pj)
        J[0:3, 0] = np.array([J_k[0], J_k[1], J_k[2]])
        J[3:6, 0] = np.array([zj[0], zj[1], zj[2]])

        for i in range(len(q) - 1):             
            a = DH[i, 0]
            alpha = DH[i, 1]
            d = DH[i, 3]
            theta = q[i]+DH[i, 2]
                
            R = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha) ],  
                            [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha)], 
                            [0,          sin(alpha),             cos(alpha)            ]])

            T = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta)],
                            [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                            [0,          sin(alpha),             cos(alpha),             d           ],
                            [0,          0,                      0,                      1           ]])
                    
            R_k = deepcopy(np.dot(R_k, R))
            T_k = deepcopy(np.dot(T_k, T)) 
                
            zj = np.dot(R_k, z0) 
            zj = np.array([zj[0, 0], zj[1, 0], zj[2, 0]])
            pj = np.dot(T_k, p0)
            pj = np.array([pj[0, 0], pj[1, 0], pj[2, 0]])
            p = A_k
            
            J_k = np.cross(zj, p - pj)
            J[0:3, i + 1] = np.array([J_k[0], J_k[1], J_k[2]])
            J[3:6, i + 1] = np.array([zj[0], zj[1], zj[2]])
    
    return J   



def compute_robot_capsules(q):
    global DH

    A_1_r = compute_capsule_end_point(DH, q, 1)
    B_1_r = np.zeros(3)  
        
    A_2_r = compute_capsule_end_point(DH, q, 3)
    B_2_r = compute_capsule_end_point(DH, q, 2)
        
    A_3_r = compute_capsule_end_point(DH, q, 5)
    B_3_r = compute_capsule_end_point(DH, q, 4)
        
    A_4_r = compute_capsule_end_point(DH, q, 7)
    B_4_r = compute_capsule_end_point(DH, q, 6)
               
    robot_capsules = np.array([[A_1_r, B_1_r],
                               [A_2_r, B_2_r],
                               [A_3_r, B_3_r],
                               [A_4_r, B_4_r]]) 

    return(robot_capsules)



def compute_capsule_end_point(DH, q, k):
    T_k = np.identity(4)

    for j in range(k):        
        a = DH[j, 0]
        alpha = DH[j, 1]
        d = DH[j, 3]
        theta = q[j]+DH[j, 2]
              
        T = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta)],
                      [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                      [0,          sin(alpha),             cos(alpha),             d],
                      [0,          0,                      0,                      1]])
                
        T_k = deepcopy(np.dot(T_k, T))  
    
    A_k = np.array([T_k[0, 3], T_k[1, 3], T_k[2, 3]])

    return(A_k)



def distance_to_skeleton(skeleton, P2, Q2):                
    count = 0
    do_once = 1
    min_distance = []

    for i in range(len(skel_index)):
        ind_1 = skel_index[i][0]
        ind_2 = skel_index[i][1]

        P1 = np.array([float(skeleton[ind_1][0]), float(skeleton[ind_1][1]), float(skeleton[ind_1][2])])
        Q1 = np.array([float(skeleton[ind_2][0]), float(skeleton[ind_2][1]), float(skeleton[ind_2][2])])

        if not (any(np.isnan(P1)) == True or any(np.isnan(Q1)) == True):
            dist = distance_to_segment(P1, Q1, P2, Q2)

            distance = dist[0]
            C_h = dist[1]
            C_r = dist[2]
                
            if do_once:
                min_distance.append(deepcopy(distance))
                min_distance.append(deepcopy(C_h))
                min_distance.append(deepcopy(C_r))
                ind_h = count
            
                do_once = 0

            else:  
                if not min_distance == []:              
                    if distance < min_distance[0]:
                        min_distance[0] = deepcopy(distance)
                        min_distance[1] = deepcopy(C_h)
                        min_distance[2] = deepcopy(C_r)
                        ind_h = count

        count += 1

    return min_distance, ind_h



def distance_to_segment(P1, Q1, P2, Q2):   
    D1 = Q1-P1
    D2 = Q2-P2
    R = P1-P2
    a = np.inner(D1, D1)     
    b = np.inner(D1, D2) 
    c = np.inner(D1, R)
    e = np.inner(D2, D2)
    f = np.inner(D2, R)
    
    d = a*e-b*b
    
    s = (b*f-c*e)/d
    s = clamp((b*f-c*e)/d)
    
    t = (b*s+f)/e

    if t < 0.0:
        t = 0.0
        s = clamp(-c/a)
    
    elif t > 1.0:
        t = 1.0
        s = clamp((b-c)/a)
        
    C_h = P1+D1*s
    C_r = P2+D2*t            
    distance = math.sqrt(np.inner(C_h-C_r, C_h-C_r))
        
    return [distance, C_h, C_r]



def clamp(n):
    if n < 0:
        out = 0
    elif n > 1:
        out = 1
    else:
        out = n

    return out



def load_skeleton(): 
    global t_start, t_exe, skel_list, skeleton

    # load skeleton positions
    joint = []
    keypoint = []

    t_exe = time.time() - t_start
    while t_exe > 1000.0:
        t_exe = time.time() - t_start
        continue
        
    for key in skel_list:
        skel_time = float(key.get('time'))

        while skel_time >= t_exe:
            t_exe = time.time() - t_start
            continue

        keypoint = []
        for i in range(15):
            point = key.find('point', {'id':i})
            joint = []
            for jnt in point.text.split():
                joint.append(jnt)
            keypoint.append(joint)
        
        skeleton = keypoint



def receive_skeleton(): 
    global skeleton

    while True:
        with open("/home/panda/Documents/Data_Collision_Avoidance/skeleton_temp.txt", 'r') as temp:
            keypoint = []
            data = temp.read()
            for line in data.split("\n"):
                joint = []
                jnt = line.split("\t")
                if not jnt == ['']:
                    for i in range(1, 4):
                        joint.append(float(jnt[i]))

                    keypoint.append(joint)
        
            skeleton = keypoint
                    

    

def FlagStopServer(T_stop, q, q_p, q_pp): 
    global rv, robot_capsules, t_exe, skeleton, skel_index

    flag_stop = False

    # Calculates robot capsules
    robot_capsules = compute_robot_capsules(q)   

    #len_h = [0.21, 0.355, 0.305, 0.355, 0.305, 0.65, 0.5, 0.5, 0.5, 0.5]
    r_sw_h = [0.16, 0.05, 0.06, 0.05, 0.06, 0.15, 0.1, 0.08, 0.1, 0.08]

    
    if not skeleton == []:  
        min_distance = []
        min_segment = []
        do_once = 1

        for i in range(len(robot_capsules)):
            P2 = deepcopy(robot_capsules[i][0])
            Q2 = deepcopy(robot_capsules[i][1])
            
            dist, ind_h_temp = distance_to_skeleton(skeleton, P2, Q2)
            
            if not dist == []: 
                distance = dist[0]
                C_h = dist[1]
                C_r = dist[2]  

                if do_once:
                    min_distance.append(deepcopy(distance))
                    min_segment.append(deepcopy(C_h))
                    min_segment.append(deepcopy(C_r))
                    ind_h = ind_h_temp
                    ind_r = i

                    do_once = 0
                                
                else:
                    if distance < min_distance[0]:
                        min_distance[0] = deepcopy(distance)
                        min_segment[0] = deepcopy(C_h)
                        min_segment[1] = deepcopy(C_r)
                        ind_h = ind_h_temp
                        ind_r = i

        do_once = 1

        if not min_distance == []:         
            r_sw_r = capsule_calculation(T_stop, q, q_p, q_pp, min_segment[0], min_segment[1])

            if min_distance[0] <  r_sw_h[ind_h] + r_sw_r[ind_r]:
                flag_stop = True                          
            else:
                flag_stop = False

    print(min_distance[0])
    return(flag_stop)
              


def HandleFlagStop(req):
    global t_start, t1, do_once

    if do_once == True:
        t_start = time.time()
        do_once = False

    flag_stop = FlagStopServer(req.T_stop, req.q, req.q_p, req.q_pp)
    
    return FlagStopResponse(flag_stop)
    


def FlagStopServer1():
    rospy.init_node('flag_stop_server') 
    
    rospy.Service('flag_stop', FlagStop, HandleFlagStop)

    print("Ready.")
    
    rospy.spin()



if __name__ == "__main__":
    rospy.init_node('flag_stop_server') 
    virt = rospy.get_param("~virt")

    DH = np.array([[0,    -pi/2,  0, 0.333],
                   [0,    -pi/2, pi, 0],
                   [0.088, pi/2, pi, 0.316],
                   [0.088, pi/2, pi, 0],
                   [0,     pi/2, pi, 0.384],
                   [0.088, pi/2, 0,  0],
                   [0,     0,    0,  0.2]])
    
    if virt:
        with open("/home/panda/Documents/Data_Collision_Avoidance/skeleton_coords.xml", 'r') as skel:
            data = skel.read()
            skel_data = BeautifulSoup(data, "xml")
            skel_list = skel_data.find_all('keypoint')

        t1 = threading.Thread(target = load_skeleton)
        t1.start()
    else:
        t1 = threading.Thread(target = receive_skeleton)
        t1.start()
    
    FlagStopServer1()
    t1.join()



