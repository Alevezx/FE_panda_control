#include "header_stop_event.h"
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/rnea.hpp>

Vector6d compute_poly5_coeff(double q0, double qf, double v0, double vf, double a0, double af, double T) {
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    Vector6d coeffs;

    // Set columns 0, 1, and 2 directly from initial conditions
    coeffs(0) = q0;
    coeffs(1) = v0;
    coeffs(2) = 0.5 * a0;

    Eigen::Vector3d known_terms;
    known_terms(0) = qf - q0 - v0*T - 0.5*a0*T2;
    known_terms(1) = vf - v0 - a0*T;
    known_terms(2) = af - a0;

    // Vandermonde matrix
    Eigen::Matrix3d V;
    V(0,0) = T3;    V(0,1) = T4;    V(0,2) = T5;
    V(1,0) = 3*T2;  V(1,1) = 4*T3;  V(1,2) = 5*T4;
    V(2,0) = 6*T;   V(2,1) = 12*T2; V(2,2) = 20*T3;

    Eigen::Vector3d result = V.lu().solve(known_terms);
    coeffs(3) = result(0);
    coeffs(4) = result(1);
    coeffs(5) = result(2);

    return coeffs;
}

void save_log_to_csv(const std::vector<franka::RobotState>& log_data, const std::string& filename) {
    if (log_data.empty()) return;
    
    std::ofstream log_file(filename);
    if (!log_file.is_open()) {
        std::cerr << "ERRORE: Impossibile creare " << filename << std::endl;
        return;
    }
    
    log_file << "time,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7\n";
    
    double time = 0.0;
    for (const auto& state : log_data) {
        log_file << time;
        for(double val : state.q) log_file << "," << val;
        for(double val : state.dq) log_file << "," << val;
        log_file << "\n";
        time += 0.001;
    }
    
    std::cout << "Log salvato: " << filename << std::endl;
}