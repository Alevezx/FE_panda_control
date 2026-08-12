#pragma once

#include <Eigen/Dense>
#include <franka/model.h>
#include <franka/robot.h>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <casadi/casadi.hpp>
#include "examples_common.h"

// Pinocchio includes
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

// constants
// Robot capsule radii
const std::array<double, 4> RV = {0.085, 0.085, 0.06, 0.065};
// Human segment safety radii
const std::array<double, 10> R_SW_H = {0.16, 0.05, 0.06, 0.05,
                                        0.06, 0.15, 0.1, 0.08, 0.1, 0.08};
// Skeleton segment connectivity
const std::vector<std::array<int,2>> SKEL_INDEX = {
    {0,1},{2,3},{3,4},{5,6},{6,7},
    {1,8},{9,10},{10,11},{12,13},{13,14}
};
const double T_reaction;              // Tempo reazione safety controller (es. 0.005s per il paper)
const double v_human_max;             // 1.6 m/s (ISO/TS 15066)
const double csi;                     // Errore sensori

const Eigen::Matrix<double, 7, 1> tau_max; // Limiti coppia Franka
const Eigen::Matrix<double, 7, 1> tau_rate_max; // Limiti derivata coppia (Nm/s)

// Pesi ottimizzazione CasADi
double w0 = 1e6;    // Peso minimizzazione t_stop
double w1 = 1e5;    // Peso continuità (penalizza discontinuità)


enum class ExecutionState {
    RUNNING,
    STOPPING,
    PAUSED
};

struct Capsule {
    Eigen::Vector3d start;
    Eigen::Vector3d end;
    double radius;
};

struct DistanceResult {
    double distance;
    Eigen::Vector3d closest_robot;
    Eigen::Vector3d closest_human;
    int ind_h;
};

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Poly5Matrix = Eigen::Matrix<double, 6, 7>;
using Skeleton = std::vector<std::array<double, 3>>;

// =======================================================
// SHARED LOCK-FREE STATE
// =======================================================
// Flags
std::atomic<bool> stop_flag{false};
std::atomic<bool> program_running{true};

// Data from Control Thread -> Collision Thread
std::array<std::atomic<double>, 7> current_q;
std::array<std::atomic<double>, 7> current_dq;
std::array<std::atomic<double>, 7> current_ddq;

// Data from Collision Thread -> Control Thread
std::atomic<double> optimized_stop_duration{0.0};


// == Utility functions ==
Vector6d compute_poly5_coeff(const double q0, const double qf, const double v0, const double vf, const double a0, const double af, double T);
void save_log_to_csv(const std::vector<franka::RobotState>& log_data, const std::string& filename);