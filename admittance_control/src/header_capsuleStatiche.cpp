#include "header_capsuleStatiche.h"

// ============================================================================
// LOGGING E UTILITY
// ============================================================================

void save_log_to_csv(const std::vector<franka::RobotState>& log_data, const std::string& filename) {
    if (log_data.empty()) {
        std::cout << "Nessun dato da salvare." << std::endl;
        return;
    }
    std::ofstream log_file(filename);
    if (!log_file.is_open()) {
        std::cerr << "ERRORE: Impossibile creare il file di log: " << filename << std::endl;
        return;
    }
    log_file << "time,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,ddq_d1,ddq_d2,ddq_d3,ddq_d4,ddq_d5,ddq_d6,ddq_d7,O_T_EE_1,O_T_EE_2,O_T_EE_3,O_T_EE_4,O_T_EE_5,O_T_EE_6,O_T_EE_7,O_T_EE_8,O_T_EE_9,O_T_EE_10,O_T_EE_11,O_T_EE_12,O_T_EE_13,O_T_EE_14,O_T_EE_15,O_T_EE_16,tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,tau_J7,tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,tau_ext7,O_F_ext_x,O_F_ext_y,O_F_ext_z,O_T_ext_x,O_T_ext_y,O_T_ext_z\n";

    double time = 0.0;
    for (const auto& state : log_data) {
        log_file << time;
        for(double val : state.q) log_file << "," << val;
        for(double val : state.dq) log_file << "," << val;
        for(double val : state.ddq_d) log_file << "," << val;
        for(double val : state.O_T_EE) log_file << "," << val;
        for(double val : state.tau_J) log_file << "," << val;
        for(double val : state.tau_ext_hat_filtered) log_file << "," << val;
        for(double val : state.O_F_ext_hat_K) log_file << "," << val;
        log_file << "\n";
        time += 0.001; 
    }
    std::cout << "Dati di log salvati correttamente in " << filename << std::endl;
}

Eigen::MatrixXd generateQuinticCoefficients(double t0, double tf, const std::vector<double>& pos0, const std::vector<double>& posf){
  Eigen::MatrixXd X(6,6);
  X << 1, t0, std::pow(t0,2),   std::pow(t0,3),    std::pow(t0,4),    std::pow(t0,5),
       0,  1,           2*t0, 3*std::pow(t0,2),  4*std::pow(t0,3),  5*std::pow(t0,4),
       0,  0,              2,             6*t0, 12*std::pow(t0,2), 20*std::pow(t0,3),
       1, tf, std::pow(tf,2),   std::pow(tf,3),    std::pow(tf,4),    std::pow(tf,5),
       0,  1,           2*tf, 3*std::pow(tf,2),  4*std::pow(tf,3),  5*std::pow(tf,4),
       0,  0,              2,             6*tf, 12*std::pow(tf,2), 20*std::pow(tf,3);

  Eigen::MatrixXd B(6,1);
  B << pos0[0], pos0[1], pos0[2], posf[0], posf[1], posf[2];
  return (X.inverse() * B);
}

// Coefficienti polinomio 5° grado (Generico: Start -> End con vel/acc custom)
Eigen::MatrixXd compute_traj_poly5_coeffs(double qi, double qi_p, double qi_pp, 
                                          double qf, double qf_p, double qf_pp, 
                                          double duration) {
    Eigen::Vector3d coeff_known; 
    coeff_known(0) = qi;
    coeff_known(1) = qi_p;
    coeff_known(2) = qi_pp / 2.0;

    Eigen::Vector3d known_terms;
    known_terms(0) = qf - qi - qi_p * duration - 0.5 * qi_pp * std::pow(duration, 2);
    known_terms(1) = qf_p - qi_p - qi_pp * duration;
    known_terms(2) = qf_pp - qi_pp;

    Eigen::Matrix3d V;
    V << std::pow(duration, 3),    std::pow(duration, 4),    std::pow(duration, 5),
         3 * std::pow(duration, 2), 4 * std::pow(duration, 3), 5 * std::pow(duration, 4),
         6 * duration,              12 * std::pow(duration, 2), 20 * std::pow(duration, 3);

    Eigen::Vector3d coeff_unknown = V.colPivHouseholderQr().solve(known_terms);
    
    Eigen::MatrixXd all_coeffs(6, 1);
    all_coeffs << coeff_known, coeff_unknown;
    return all_coeffs;
}

void trajectory_updater(Eigen::Matrix<double,3,1>& nextTrajPosition, double t, double t_start,
                            const Eigen::MatrixXd& coeffsX, const Eigen::MatrixXd& coeffsY, const Eigen::MatrixXd& coeffsZ){
  double dt = t - t_start;
  if (dt < 0) dt = 0;
  
  nextTrajPosition[0] = coeffsX(0) + coeffsX(1)*dt + coeffsX(2)*std::pow(dt,2) + coeffsX(3)*std::pow(dt,3) + coeffsX(4)*std::pow(dt,4) + coeffsX(5)*std::pow(dt,5);
  nextTrajPosition[1] = coeffsY(0) + coeffsY(1)*dt + coeffsY(2)*std::pow(dt,2) + coeffsY(3)*std::pow(dt,3) + coeffsY(4)*std::pow(dt,4) + coeffsY(5)*std::pow(dt,5);
  nextTrajPosition[2] = coeffsZ(0) + coeffsZ(1)*dt + coeffsZ(2)*std::pow(dt,2) + coeffsZ(3)*std::pow(dt,3) + coeffsZ(4)*std::pow(dt,4) + coeffsZ(5)*std::pow(dt,5);
}

// ============================================================================
// CALCOLO DISTANZA TRA SEGMENTI
// ============================================================================

static double clamp_val(double n) {
    return std::max(0.0, std::min(1.0, n));
}

DistanceResult distance_to_segment(Eigen::Vector3d P1, Eigen::Vector3d Q1, 
                                   Eigen::Vector3d P2, Eigen::Vector3d Q2) {
    
    Eigen::Vector3d D1 = Q1 - P1;
    Eigen::Vector3d D2 = Q2 - P2;
    Eigen::Vector3d R  = P1 - P2;

    double a = D1.dot(D1);
    double b = D1.dot(D2);
    double c = D1.dot(R);
    double e = D2.dot(D2);
    double f = D2.dot(R);

    if (a <= 1e-9 && e <= 1e-9) return { (P1 - P2).norm(), P1, P2 };
    if (a <= 1e-9) {
        double t = clamp_val(-f / std::max(e, 1e-9));
        Eigen::Vector3d closest_on_2 = P2 + t * D2;
        return { (P1 - closest_on_2).norm(), P1, closest_on_2 };
    }
    if (e <= 1e-9) {
        double s = clamp_val(-c / std::max(a, 1e-9));
        Eigen::Vector3d closest_on_1 = P1 + s * D1;
        return { (closest_on_1 - P2).norm(), closest_on_1, P2 };
    }

    double d = a * e - b * b;
    if (std::abs(d) < 1e-9) d = 1e-9; 

    double s = clamp_val((b * f - c * e) / d);
    double t = (b * s + f) / e;

    if (t < 0.0) { t = 0.0; s = clamp_val(-c / a); }
    else if (t > 1.0) { t = 1.0; s = clamp_val((b - c) / a); }

    Eigen::Vector3d C_h = P1 + D1 * s;
    Eigen::Vector3d C_r = P2 + D2 * t;

    return {(C_h - C_r).norm(), C_h, C_r};
}