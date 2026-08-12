#pragma once
#include <array>
#include <vector>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "shared_state.h"

// ── Types ─────────────────────────────────────────────────────────────

// A capsule is defined by two 3D endpoints
struct Capsule {
    Eigen::Vector3d A;  // first endpoint
    Eigen::Vector3d B;  // second endpoint
    double          radius;
};

// Result of a distance query
struct DistanceResult {
    double          distance;
    Eigen::Vector3d C_h;  // closest point on human segment
    Eigen::Vector3d C_r;  // closest point on robot capsule
};

// ── Constants (mirrors Python globals) ────────────────────────────────
// Robot capsule radii
const std::array<double, 4> RV         = {0.085, 0.085, 0.06, 0.065};
// Human segment safety radii
const std::array<double, 10> R_SW_H    = {0.16, 0.05, 0.06, 0.05,
                                           0.06, 0.15, 0.1, 0.08, 0.1, 0.08};
// Skeleton segment connectivity
const std::vector<std::array<int,2>> SKEL_INDEX = {
    {0,1},{2,3},{3,4},{5,6},{6,7},
    {1,8},{9,10},{10,11},{12,13},{13,14}
};

const double T_REACTION      = 0.05;   // seconds
const double CSI             = 0.0;    // Zd + Zr constant
const double MAX_HUMAN_SPEED = 1.6;    // m/s

// ── Function declarations ──────────────────────────────────────────────

// Non-symbolic 5th degree polynomial trajectory (mirrors TrajPoly5 in Python)
// Returns matrix of size [num_points x 3] where columns are [q, q_p, q_pp]
Eigen::MatrixXd trajPoly5Numeric(
    double qi, double qi_p, double qi_pp,
    double qf, double qf_p, double qf_pp,
    double duration, int num_points);

// Compute the 4 robot capsule endpoints from joint config using Pinocchio FK
std::array<Capsule, 4> computeRobotCapsules(
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    const Eigen::VectorXd&  q);

// Compute max velocity of capsule endpoints toward human
std::array<double, 4> computeMaxVelCapsule(
    const pinocchio::Model&    model,
    pinocchio::Data&           data,
    const Eigen::VectorXd&     q,
    const Eigen::VectorXd&     q_p,
    const std::array<Capsule, 4>& capsules,
    const Eigen::Vector3d&     C_h,
    const Eigen::Vector3d&     C_r);

// Compute robot safety radii for each capsule along braking trajectory
std::array<double, 4> capsuleCalculation(
    double                  T_stop,
    const Eigen::VectorXd&  q,
    const Eigen::VectorXd&  q_p,
    const Eigen::VectorXd&  q_pp,
    const Eigen::Vector3d&  C_h,
    const Eigen::Vector3d&  C_r,
    const pinocchio::Model& model,
    pinocchio::Data&        data);

// Main stop decision function
bool flagStop(
    double                  T_stop,
    const Eigen::VectorXd&  q,
    const Eigen::VectorXd&  q_p,
    const Eigen::VectorXd&  q_pp,
    const Skeleton&         skeleton,
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    DistanceResult*         out_closest = nullptr);