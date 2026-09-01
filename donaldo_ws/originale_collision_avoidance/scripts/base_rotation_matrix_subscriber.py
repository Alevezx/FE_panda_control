#!/usr/bin/env python3

"""
Il programma salva su documento di testo la matrice di rotazione per ottenere la terna base
Bisogna prima lanciare due nodi:
- roslaunch realsense2_camera rs_camera.launch (wrapper della telecamera)
- roslaunch aruco_ros single.launch (libreria aruco che sfrutta OpenCV per identificare il marker)
"""

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from math import cos, sin, pi, atan2
import rospy
from time import sleep
from geometry_msgs.msg import PoseStamped

marker_pos = []


def callback(data, arg):
    global marker_pos

    a = data.pose.position.x
    b = data.pose.position.y
    c = data.pose.position.z
    x_q = data.pose.orientation.x
    y_q = data.pose.orientation.y
    z_q = data.pose.orientation.z
    w_q = data.pose.orientation.w

    x = a
    y = c
    z = -b

    euler = quaternion_to_euler(w_q, x_q, y_q, z_q)

    R = Rx(-pi/2)*Rz(euler[0])*Ry(euler[1])*Rx(euler[2])*Rz(-pi/2)
    V = np.array([[0, 0, 0]])
    F = np.array([[x], [y], [z], [1]])
        
    R = np.concatenate((R, V), axis = 0)
    R = np.concatenate((R, F), axis = 1)

    K = np.array([[1, 0, 0, marker_pos[0]],
                  [0, 1, 0, marker_pos[1]],
                  [0, 0, 1, marker_pos[2]],
                  [0, 0, 0, 1            ]])

    R = R*K

    sleep(1)

    arg.append(R)
   
   

def quaternion_to_euler(w, x, y, z):    
    r21 = 2*(x*y + w*z)
    r11 = 2*(w**2 + x**2) - 1    
    r31 = 2*(x*z - w*y)
    r32 = 2*(y*z + w*x)
    r33 = 2*(w**2 + z**2) - 1

    X = (atan2(r21, r11))
    Y = (atan2(-r31, (r32**2 + r33**2)**0.5))
    Z = (atan2(r32, r33))
    
    return X, Y, Z
    

def Rx(theta):
    return np.matrix([[ 1, 0         , 0         ],
                      [ 0, cos(theta),-sin(theta)],
                      [ 0, sin(theta), cos(theta)]])
 
def Ry(theta):
    return np.matrix([[ cos(theta), 0, sin(theta)],
                      [ 0         , 1, 0         ],
                      [-sin(theta), 0, cos(theta)]])
 
def Rz(theta):
    return np.matrix([[ cos(theta), -sin(theta), 0 ],
                      [ sin(theta), cos(theta) , 0 ],
                      [ 0         , 0          , 1 ]])

      
def listener():
    global marker_pos
    
    rospy.init_node('listener', anonymous=True)

    arg = list()
            
    Reader = rospy.Subscriber('/aruco_single/pose', PoseStamped, callback, (arg))

    # dati inseriti da print di marker_pos_calculator
    file = open("/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/traiettoria_rettilinea_setup_completo/marker_pos.txt", 'r')
    temp = file.read().split("\t")

    marker_pos = [float(temp[0]), float(temp[1]), float(temp[2])]

    file.close()
    
    while True: 
        if not arg == []:         
            R = arg[0] 

            file = open('/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/traiettoria_rettilinea_setup_completo/rotation_matrix.txt', 'w')

            for i in range(4):
                
                file.write(str(R[i, 0]))
                file.write('\t')
                file.write(str(R[i, 1]))
                file.write('\t')
                file.write(str(R[i, 2]))
                file.write('\t')
                file.write(str(R[i, 3]))
                file.write('\t')
                file.write('\n')

            file.close()
            print(R)

            Reader.unsubscribe()

            quit()
                        
              
       
if __name__ == '__main__':
    
    listener()