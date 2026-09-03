#ifndef HEADER_CAPSULESTATICHE_H
#define HEADER_CAPSULESTATICHE_H

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

// Struttura geometrica della capsula (segmento + raggio)
struct CapsuleGeo {
    Eigen::Vector3d p_start;
    Eigen::Vector3d p_end;
    double radius;
};

// Struttura parametri sicurezza (STATICHE)
struct SafetyParams {
    double r_inner;         // Raggio interno (Hard Safety / STOP)
    double r_outer;         // Raggio esterno (Soft Safety / Repulsione)
    double T_stop;          // Tempo per eseguire la frenata dolce
    // La tabella DH può servire se in futuro vorrai calcolare i centri delle capsule
    // sui link, per ora la lasciamo.
    Eigen::MatrixXd DH;     
};

// Struttura per restituire i risultati del calcolo distanza segmenti
struct DistanceResult {
    double distance;                // Distanza minima scalare
    Eigen::Vector3d closest_point_1; // Punto più vicino sul primo segmento (Robot)
    Eigen::Vector3d closest_point_2; // Punto più vicino sul secondo segmento (Ostacolo)
};

void save_log_to_csv(const std::vector<franka::RobotState>& log_data, const std::string& filename);

Eigen::MatrixXd generateQuinticCoefficients(double t0, double tf, const std::vector<double>& pos0, const std::vector<double>& posf);

// Calcola i coefficienti per un polinomio di 5° grado (Pos, Vel, Acc iniziali e finali)
// Fondamentale per la traiettoria di stop
Eigen::MatrixXd compute_traj_poly5_coeffs(double qi, double qi_p, double qi_pp, 
                                          double qf, double qf_p, double qf_pp, 
                                          double duration);

void trajectory_updater(Eigen::Matrix<double,3,1>& nextTrajPosition, double t, double t_start,
                            const Eigen::MatrixXd& coeffsX, const Eigen::MatrixXd& coeffsY, const Eigen::MatrixXd& coeffsZ);

// --- CALCOLO DISTANZA TRA SEGMENTI ---
DistanceResult distance_to_segment(Eigen::Vector3d P1, Eigen::Vector3d Q1, 
                                   Eigen::Vector3d P2, Eigen::Vector3d Q2);

#endif // HEADER_CAPSULESTATICHE_H