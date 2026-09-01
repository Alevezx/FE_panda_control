#!/usr/bin/env python3
# coding=utf-8

import rospy
import numpy as np
import math
import time
from math import cos, sin, pi, atan2
from copy import deepcopy
from sensor_msgs.msg import JointState

q_meas = []

def CallbackJointStates(data):
    global q_meas
 
    q_meas = data.position
    


def compute_end_effector(DH, q):
    T_k = np.identity(4)

    for j in range(7):        
        a = DH[j, 0]
        alpha = DH[j, 1]
        d = DH[j, 3]
        theta = q[j]+DH[j, 2]
              
        T = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta)],
                      [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                      [0,          sin(alpha),             cos(alpha),             d],
                      [0,          0,                      0,                      1]])
                
        T_k = deepcopy(np.dot(T_k, T))  
    
    R = T_k   

    return R


if __name__ == "__main__":
    rospy.init_node('marker_pos_calculator')

    # Initialization subscriber node for /joint_states
    sub_joint_states = rospy.Subscriber('/joint_states', JointState, CallbackJointStates)

    q_multi = []

    time.sleep(1)

    if input("\n\nDo you want to update marker position?(y/n)\n") == "y":
        input("With the tip of the calibration tool, touch the corner n.1 of the marker and then press enter...")
        q_multi.append(q_meas)
        input("With the tip of the calibration tool, touch the corner n.2 of the marker and then press enter...")
        q_multi.append(q_meas)
        input("With the tip of the calibration tool, touch the corner n.3 of the marker and then press enter...")
        q_multi.append(q_meas)
        input("With the tip of the calibration tool, touch the corner n.4 of the marker and then press enter...")
        q_multi.append(q_meas)

        DH = np.array([[0,    -pi/2,  0, 0.333],
                    [0,    -pi/2, pi, 0],
                    [0.088, pi/2, pi, 0.316],
                    [0.088, pi/2, pi, 0],
                    [0,     pi/2, pi, 0.384],
                    [0.088, pi/2, 0,  0],
                    [0,     0,    0,  0.107]])

        p = []

        for i in range(len(q_multi)):
            q = deepcopy(q_multi[i])
            
            R = compute_end_effector(DH, q)

            A = np.array([[1, 0, 0, 0   ],
                        [0, 1, 0, 0   ],
                        [0, 0, 1, 0.05],
                        [0, 0, 0, 1   ]]) 

            E = np.dot(R, A)

            p.append([E[0][3], E[1][3], E[2][3]])

        A1 =[]
        A2 =[]
        A3 =[]
        A4 =[]

        for j in range(3):
            A1.append((p[0][j] + p[1][j])/2)
            A2.append((p[2][j] + p[3][j])/2)
            A3.append((p[0][j] + p[3][j])/2)
            A4.append((p[1][j] + p[2][j])/2)

        C1 = []
        C2 = []

        for j in range(3):
            C1.append((A1[j] + A2[j])/2)
            C2.append((A3[j] + A4[j])/2)

        C = []

        for j in range(3):
            C.append(-(C1[j] + C2[j])/2)   #inverto i segni per poter portare l'array come variabile marker_pos in base_rotation_matrix_subscriber.py 

        print("\n\nThis is the resulting relative position between the marker center point and the base of the robot\n")
        print(C)

        file = open("/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/traiettoria_rettilinea_setup_completo/marker_pos.txt", 'w')
        file.write(f"{C[0]}\t{C[1]}\t{C[2]}")
        file.close()

    else:
        quit()
       
    

