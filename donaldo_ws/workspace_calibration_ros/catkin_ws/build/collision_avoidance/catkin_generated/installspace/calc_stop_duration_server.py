#!/usr/bin/env python3
# coding=utf-8

"""
Ottimizzatore con TrajPoly5 e altri vincoli (tutti meno posizione e derivate della coppia)
"""

import rospy
import casadi as ca
from def_proj.srv import CalcStopDuration, CalcStopDurationResponse
from copy import deepcopy
import urdf2casadi.urdfparser as u2c
import time

# Settings
lbx = 0.01
ubx = 0.4
p1 = 1e6
p2 = 1e5
n = 10
path_to_urdf = "/home/panda/franka_emika_ws/src/collision_avoidance/urdf/panda.urdf"
root = "panda_link0"
tip = "panda_link8"
gravity_u2c = [0, 0, -9.81]                                                         # m/s^2                           
q_min_lim = ca.DM([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])  # rad
q_max_lim = ca.DM([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])        # rad
q_p_lim = ca.DM([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])           # rad/s
q_pp_lim = ca.DM([15, 7.5, 10, 12.5, 15, 20, 20])                                   # rad/s^2
q_ppp_lim = ca.DM([7500, 3750, 5000, 6250, 7500, 10000, 10000])                     # rad/s^3
tau_lim = ca.DM([87, 87, 87, 87, 12, 12, 12])                                       # Nm
tau_p_lim = ca.DM([1000, 1000, 1000, 1000, 1000, 1000, 1000])                       # Nm/s
toll = 0.9

sol = None
stop_duration_prec = ubx



def Cost(x):
    global stop_duration_prec

    return x * p1 + ca.fabs(x - stop_duration_prec) * p2



def TrajPoly5Symb(qi, qi_p, qi_pp, qf, qf_p, qf_pp, x, n):
    coeff = ca.SX.zeros(6,1)

    coeff[0] = qi
    coeff[1] = qi_p
    coeff[2] = qi_pp / 2

    known_terms = ca.SX.zeros(3,1)
    known_terms[0] = qf - qi - qi_p * x - 0.5 * qi_pp * x**2
    known_terms[1] = qf_p - qi_p - qi_pp * x
    known_terms[2] = qf_pp - qi_pp

    V = ca.SX.zeros(3, 3)
    V[0,0] = x**3
    V[0,1] = x**4
    V[0,2] = x**5
    V[1,0] = 3 * x**2
    V[1,1] = 4 * x**3
    V[1,2] = 5 * x**4
    V[2,0] = 6 * x
    V[2,1] = 12 * x**2
    V[2,2] = 20 * x**3

    coeff_1 = ca.solve(V, known_terms)

    coeff[3] = coeff_1[0]
    coeff[4] = coeff_1[1]
    coeff[5] = coeff_1[2]

    t = ca.SX.zeros(n + 1, 1)
    for i in range(n + 1):
        t[i] = i * x / float(n)

    q = coeff[0] + coeff[1] * t + coeff[2] * t**2 + coeff[3] * t**3 + coeff[4] * t**4 + coeff[5] * t**5
    q_p = coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t**2 + 4 * coeff[4] * t**3 + 5 * coeff[5] * t**4
    q_pp = 2 * coeff[2] + 6 * coeff[3] * t + 12 * coeff[4] * t**2 + 20 * coeff[5] * t**3

    return ca.horzcat(q, q_p, q_pp) 



def Constraints(x, qi, qi_p, qi_pp):
    Q = ca.SX.zeros(n + 1, 7)
    Q_p = ca.SX.zeros(n + 1, 7)
    Q_pp = ca.SX.zeros(n + 1, 7)

    for i in range(7):
        TRAJ = TrajPoly5Symb(qi[i], qi_p[i], qi_pp[i] ,qi[i], 0, 0, x, n)
        Q[:,i] = deepcopy(TRAJ[:,0])
        Q_p[:,i] = deepcopy(TRAJ[:,1])
        Q_pp[:,i] = deepcopy(TRAJ[:,2])

    panda = u2c.URDFparser()
    panda.from_file(path_to_urdf)

    id_sym = panda.get_inverse_dynamics_rnea(root, tip, gravity_u2c)

    Q_ppp = ca.SX.zeros(n + 1, 7)
    tau = ca.SX.zeros(n + 1, 7)
    tau_p  = ca.SX.zeros(n + 1, 7)

    tau[0,:] = id_sym(Q[0,:], Q_p[0,:], Q_pp[0,:])

    for i in range(1, n + 1):
        Q_ppp[i,:] = (Q_pp[i,:] - Q_pp[i - 1,:]) / (x / float(n))
        tau[i,:] = id_sym(Q[i,:], Q_p[i,:], Q_pp[i,:])
        tau_p[i,:] = (tau[i,:] - tau[i - 1,:]) / (x / float(n))

    q_min = ca.SX.zeros(7)
    q_max = ca.SX.zeros(7)
    q_p_max = ca.SX.zeros(7)
    q_pp_max = ca.SX.zeros(7)
    q_ppp_max = ca.SX.zeros(7)
    tau_max = ca.SX.zeros(7)
    tau_p_max = ca.SX.zeros(7)
    for i in range(7):
        q_min[i] = ca.mmin(Q[:,i])
        q_max[i] = ca.mmax(Q[:,i])
        q_p_max[i] = ca.mmax(ca.fabs(Q_p[:,i]))
        q_pp_max[i] = ca.mmax(ca.fabs(Q_pp[:,i]))
        q_ppp_max[i] = ca.mmax(ca.fabs(Q_ppp[:,i]))
        tau_max[i] = ca.mmax(ca.fabs(tau[:,i]))
        tau_p_max[i] = ca.mmax(ca.fabs(tau_p[:,i]))

    c01 = q_min_lim * toll - q_min
    c02 = q_max - q_max_lim * toll
    c1 = q_p_max - q_p_lim * toll
    c2 = q_pp_max - q_pp_lim * toll
    c3 = q_ppp_max - q_ppp_lim * toll
    c4 = tau_max - tau_lim * toll
    c5 = tau_p_max - tau_p_lim * toll

    return ca.vertcat(c01, c02, c1, c2, c3, c4, c5)



def CreateSolver():
    x = ca.SX.sym('x')
    p = ca.SX.sym('p', 3, 7)

    nlp = {'x':x, 'p':p, 'f':Cost(x), 'g':Constraints(x, p[0,:], p[1,:], p[2,:])}
    opts = {'ipopt':{'print_level':0, 'sb':'yes', 'max_iter':10}, 'print_time':0}

    sol = ca.nlpsol('sol', 'ipopt', nlp, opts)

    return sol



def HandleStopDurationOtt(req):
    global stop_duration_prec

    p = ca.DM([req.q, req.q_p, req.q_pp])
    r = sol(x0 = stop_duration_prec, p = p, lbx = lbx, ubx = ubx, ubg = 0)
    
    stop_duration = r['x'].__float__()    
    stop_duration_prec = stop_duration

    return CalcStopDurationResponse(stop_duration)



def CalcStopDurationOttServer():
    global sol

    rospy.init_node('calc_stop_duration_server')

    sol = CreateSolver()

    s = rospy.Service('calc_stop_duration', CalcStopDuration, HandleStopDurationOtt)

    print("Ready.")

    rospy.spin()



if __name__ == "__main__":
    CalcStopDurationOttServer()