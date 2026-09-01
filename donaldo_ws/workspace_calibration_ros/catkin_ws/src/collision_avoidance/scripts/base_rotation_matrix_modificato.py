#!/usr/bin/env python3

"""
Il programma calcola e salva la trasformazione omogenea dalla Base Robot alla Telecamera.
Catena cinematica: Base <--- Marker <--- Camera
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans # Usiamo la libreria standard di ROS per le trasformazioni

marker_pos_base_frame = []

def get_matrix_from_pose(p, q):
    """ Converte posizione e quaternione in matrice 4x4 """
    # Traslazione
    trans_mat = tf_trans.translation_matrix([p.x, p.y, p.z])
    # Rotazione (da quaternione)
    rot_mat = tf_trans.quaternion_matrix([q.x, q.y, q.z, q.w])
    
    # Moltiplicazione: Traslazione * Rotazione
    # Nota: tf_trans crea matrici 4x4 piene
    T = np.dot(trans_mat, rot_mat)
    return T

def callback(data):
    global marker_pos_base_frame
    
    # 1. Otteniamo la trasformazione Marker rispetto alla Camera (C_T_M)
    # Questa arriva direttamente dal topic di aruco
    C_T_M = get_matrix_from_pose(data.pose.position, data.pose.orientation)
    
    # 2. Costruiamo la trasformazione Base rispetto al Marker (M_T_B)
    # Sappiamo che l'orientamento è identico (Matrice Identità)
    # marker_pos_base_frame è il vettore posizione della Base visto dal Marker
    M_T_B = np.identity(4)
    M_T_B[0, 3] = marker_pos_base_frame[0]
    M_T_B[1, 3] = marker_pos_base_frame[1]
    M_T_B[2, 3] = marker_pos_base_frame[2]
    
    # 3. Calcoliamo la Base rispetto alla Camera (C_T_B)
    # Formula: C_T_B = C_T_M * M_T_B
    C_T_B = np.dot(C_T_M, M_T_B)
    
    # 4. Calcoliamo la Camera rispetto alla Base (B_T_C)
    # È l'inversa di C_T_B
    B_T_C = np.linalg.inv(C_T_B)

    # Output e Salvataggio
    print("Matrice calcolata (Camera rispetto a Base Robot):")
    print(B_T_C)
    
    #file_path = '/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/traiettoria_rettilinea_setup_completo/rotation_matrix.txt'
    file_path = '/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/05_pick_and_place_v2/rotation_matrix.txt'
    
    # Salvataggio in formato leggibile (tab separated)
    with open(file_path, 'w') as file:
        for i in range(4):
            line = f"{B_T_C[i,0]:.6f}\t{B_T_C[i,1]:.6f}\t{B_T_C[i,2]:.6f}\t{B_T_C[i,3]:.6f}\n"
            file.write(line)
            
    print(f"Salvato in {file_path}")
    
    # Chiudiamo il nodo dopo aver salvato una volta
    rospy.signal_shutdown("Calibrazione completata.")

def listener():
    global marker_pos_base_frame
    
    rospy.init_node('base_rotation_matrix_listener', anonymous=True)

    # Lettura file posizione marker
    try:
        #with open("/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/traiettoria_rettilinea_setup_completo/marker_pos.txt", 'r') as file:
        with open("/home/lab/donaldo_ws/progetto_tesi_magistrale/applicazione_controllo_ammettenza/05_pick_and_place_v2/marker_pos.txt", 'r') as file:
            temp = file.read().strip().split("\t")
            # Assicuriamoci di leggere float
            marker_pos_base_frame = [float(x) for x in temp if x]
            print(f"Posizione Base rispetto a Marker letta: {marker_pos_base_frame}")
    except Exception as e:
        rospy.LogError(f"Errore lettura file marker_pos: {e}")
        return

    # Subscriber
    rospy.Subscriber('/aruco_single/pose', PoseStamped, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()