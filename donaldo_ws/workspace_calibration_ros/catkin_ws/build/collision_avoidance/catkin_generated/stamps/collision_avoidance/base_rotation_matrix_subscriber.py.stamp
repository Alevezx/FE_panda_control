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

    # 1. Estrazione posa del marker vista dalla Camera (Topic ROS)
    a = data.pose.position.x
    b = data.pose.position.y
    c = data.pose.position.z
    x_q = data.pose.orientation.x
    y_q = data.pose.orientation.y
    z_q = data.pose.orientation.z
    w_q = data.pose.orientation.w

    # 2. Swap degli assi (Cambio di coordinate)
    # Converte dal frame ottico della camera (Z avanti, X destra, Y giù)
    # a un frame spaziale più convenzionale per il robot.
    x = a
    y = c
    z = -b
    


    euler = quaternion_to_euler(w_q, x_q, y_q, z_q)

    # 3. Costruzione della Matrice di Rotazione R (Marker -> Camera Adjusted)
    # Questa catena di rotazioni (Rx, Rz, Ry...) serve a riallineare gli assi
    # del marker rilevato affinché coincidano con la convenzione desiderata
    # per il calcolo finale T_base_camera.
    #R = Rx(-pi/2)*Rz(euler[0])*Ry(euler[1])*Rx(euler[2])*Rz(-pi/2)
    R = Rz(euler[0])*Ry(euler[1])*Rx(euler[2])*Rx(pi/2)

    # Aggiunta della traslazione alla matrice di rotazione per farla diventare 4x4
    V = np.array([[0, 0, 0]])
    F = np.array([[x], [y], [z], [1]])
        
    R = np.concatenate((R, V), axis = 0)
    R = np.concatenate((R, F), axis = 1)

    # 4. Costruzione della Matrice K (Base -> Marker)
    # Contiene la traslazione nota del marker rispetto alla base del robot.
    # NOTA: Assume che la rotazione tra Base e Marker sia Identità (nessuna rotazione relativa).
    K = np.array([[1, 0, 0, marker_pos[0]],
                  [0, 1, 0, marker_pos[1]],
                  [0, 0, 1, marker_pos[2]],
                  [0, 0, 0, 1            ]])

    # 5. Calcolo Finale: T_base_camera = T_marker_camera * T_base_marker
    # Nota: R è np.matrix, K è np.array. L'operatore '*' qui esegue il prodotto matriciale.
    # R = R*K
    R = np.dot(R, K)

    sleep(1)

    arg.append(R)
   
   

def quaternion_to_euler(w, x, y, z):    
    # Conversione da Quaternioni ad angoli di Eulero (Roll, Pitch, Yaw)
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

    # Lista mutabile per salvare il risultato della callback
    arg = list()
            
    Reader = rospy.Subscriber('/aruco_single/pose', PoseStamped, callback, (arg))

    # dati inseriti da print di marker_pos_calculator
    file = open("/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/pick_and_place_finale/marker_pos.txt", 'r')
    temp = file.read().split("\t")

    marker_pos = [float(temp[0]), float(temp[1]), float(temp[2])]

    file.close()
    
    while True: 
        # Appena la callback popola 'arg' con una matrice calcolata, salva ed esce
        if not arg == []:         
            R = arg[0] 

            file = open('/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/pick_and_place_finale/rotation_matrix_test.txt', 'w')

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

            # Reader.unsubscribe()
            # CORREZIONE:
            Reader.unregister()

            quit()
                        
              
       
if __name__ == '__main__':
    
    listener()