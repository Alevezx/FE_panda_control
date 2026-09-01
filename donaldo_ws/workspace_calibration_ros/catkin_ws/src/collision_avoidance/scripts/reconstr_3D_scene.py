#!/usr/bin/env python3
# coding=utf-8

import csv
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import numpy as np
import math
import xmltodict
import time
import threading
import roboticstoolbox as rtb
import cv2
import rospy



t_exe = 0
t_start = 0
q = np.zeros((1,7))
skeleton = np.zeros((15,3))
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
s = 0.1



panda = rtb.models.DH.Panda()




def print_3D_plot(rot_mat):
    global ax, skeleton, q, skel_index

    fig = plt.figure(0, (6.4, 4.8), 200)
    ax = fig.add_subplot(111, projection='3d')
    ax.cla()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-0.75, 1.25)
    ax.view_init(10,-15)


    # plot skeleton
    for i in range(len(skel_index)):
        ind_1 = skel_index[i][0]
        ind_2 = skel_index[i][1]

        joint_1 = skeleton[ind_1, 0:3]
        joint_2 = skeleton[ind_2, 0:3]

        if not (any(np.isnan(joint_1)) or any(np.isnan(joint_2))):
            ax.plot([joint_1[0], joint_2[0]], [joint_1[1], joint_2[1]], [joint_1[2], joint_2[2]], 'b')
    
    # plot panda
    jnt = panda.fkine_all(q)
    for i in range(len(jnt) - 1):
        ax.plot([jnt.t[i][0], jnt.t[i+1][0]], [jnt.t[i][1], jnt.t[i+1][1]], [jnt.t[i][2], jnt.t[i+1][2]], 'r')
        ax.text(jnt.t[i][0], jnt.t[i][1], jnt.t[i][2], f"{i}")


    ax.plot([rot_mat[0][3], rot_mat[0][3]+rot_mat[0][1]*0.1], [rot_mat[1][3], rot_mat[1][3]+rot_mat[1][1]*0.1], [rot_mat[2][3], rot_mat[2][3]+rot_mat[2][1]*0.1], 'g')

    plt.pause(0.1)



def load_skeleton():
    global skeleton
    
    # load skeleton
    with open("/home/panda/Documents/Data_Collision_Avoidance/skeleton_coords.xml") as fd:
        skel = xmltodict.parse(fd.read())
        for keypoints in skel["skeleton"]["keypoint"]:
            if float(keypoints["@time"]) <= t_exe:
                continue
            for points in keypoints["point"]:
                pnt = points["#text"].split()
                skeleton[int(points["@id"]), 0:3] = [float(pnt[0]), float(pnt[1]), float(pnt[2])]

            break
    
            

def load_configuration():
    global q, t_exe

    # load robot configuration
    #with open("/home/panda/Documents/Data_Collision_Avoidance/demo_stop_event_control/q_robot.xml") as fd2:
    #    rob = xmltodict.parse(fd2.read())
    #    for points in rob["q"]["point"]:
    #        if float(points["@time"]) <= t_exe:
    #            continue
    #        
    #        point_temp = points["#text"].split()
    #        q = [float(point_temp[0]), float(point_temp[1]), float(point_temp[2]), float(point_temp[3]), float(point_temp[4]), float(point_temp[5]), float(point_temp[6])]
    #
    #        break

    q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]



def write_output(result):
    global fig

    fig.canvas.draw()
    f_arr = np.array(fig.canvas.renderer._renderer)
    f_arr = cv2.resize(f_arr,(1920, 1080))
    bgr = cv2.cvtColor(f_arr, cv2.COLOR_RGBA2BGR)
    result.write(bgr)





if __name__ == '__main__': 
    rospy.init_node('3D_scene')    
    #result = cv2.VideoWriter("/home/panda/Documents/Data_Collision_Avoidance/output.avi", cv2.VideoWriter_fourcc(*'MJPG'), 10, (1920, 1080)) 

    file = open("/home/panda/Documents/Data_Collision_Avoidance/rotation_matrix.txt", 'r')

    do_once = True
    for line in file:
        array = line.split('\t')
        a = []            

        for i in range(4):
            a.append(float(array[i]))
        P = np.array([a])

        if do_once == True:
            R_base2cam = P
            do_once = False
        else:
            R_base2cam = np.concatenate((R_base2cam, P), axis = 0)

    R_cam2base = np.linalg.inv(R_base2cam)

    print("Cam-to-robot base rotation matrix")
    print(R_cam2base)
 
    try:
        for t_exe in range(0, 30, 2):
            load_skeleton()
            load_configuration()
            print_3D_plot(R_cam2base)
    except:
        file.close()


