#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

Eigen::VectorXd compute_friction(Eigen::VectorXd dq) {
    Eigen::VectorXd tau_friction = Eigen::VectorXd::Zero(7);

    // con i coefficienti secondo Gaz, equazione 25
    std::array<double, 7> phi1 = {5.4615e-01, 0.87224, 6.4068e-01, 1.2794e+00, 8.3904e-01, 3.0301e-01, 5.6489e-01}; // N*m
    std::array<double, 7> phi2 = {5.1181, 9.0657e+00, 1.0136e+01, 5.5903e+00, 8.3469e+00, 1.7133e+01, 1.0336e+01}; // s/rad
    std::array<double, 7> phi3 = {3.9533e-02, 2.5882e-02, -4.6070e-02, 3.6194e-02, 2.6226e-02, -2.1047e-02, 3.5526e-03}; // rad/s
    
    for (int j = 0; j < 7; j++) {
        double e = -phi2[j]*(dq(j)+phi3[j]);
        double den1 = 1 + std::exp(e);
        double den2 = 1 + std::exp(-phi2[j]*phi3[j]);

        tau_friction(j) = phi1[j] / den1 - phi1[j] / den2;
    }

    // con i coefficienti secondo Scalera_2023, equazione 24 di Gaz
    /*std::array<double, 7> beta_v = {0.0853, 0.8687, 0.0597, 0.1877, 0.0896, 0.0172, 0.0499};
    std::array<double, 7> beta_c = {0.1506, 3.1571, 0.2381, 0.3726, 0.2950, 0.1281, 0.2133};
    std::array<double, 7> offset = {0, 0, 0, 0, 0, 0, 0};
    // offset calcolato come media delle differenze
    //std::array<double, 7> offset = {-0.0001, -5.5576, -0.4328, 3.6848, 0.1650, 0.2438, 0.0564};*/
    

    /*// coefficienti presi da Gaz per equazione 24
    std::array<double, 7> beta_v = {0.0665, 0.1987, 0.0399, 0.2257, 0.1023, -0.0132, 0.0638};
    std::array<double, 7> beta_c = {0.2450, 0.1523, 0.1827, 0.3591, 0.2669, 0.1658, 0.2109};
    std::array<double, 7> offset = {-0.1073, -0.1566, -0.0686, -0.2522, 0.0045, 0.0910, -0.0127};

    for (int j = 0; j < 7; j++) {
        double sign = std::copysign(1.0, dq(j)); // discontinuità aspre
        double eps = 0.5; // discontinuità smussate
        //double sign = std::tanh(dq(j) / eps);
        tau_friction(j) = beta_v[j] * dq(j) + beta_c[j] * sign + offset[j];
    }*/
    return tau_friction;
}


std::vector<std::vector<double>> readCSV(const std::string& path) {
    std::vector<std::vector<double>> rows;
    std::ifstream file(path);
    std::string line;

    while (std::getline(file, line)) {
        //std::cout << line;
        if (line.empty() || line[0] == '#') continue; // skip comments/blanks
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ' '))
            row.push_back(std::stod(cell));
        rows.push_back(row);
    }
    return rows;
}

int main() {
    const std::string urdf_path = "../urdf/panda_arm.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    std::cout << "Model loaded: " << model.nq << " DOF\n";

    std::ofstream out("../results/torques_out.txt");
    out << std::fixed;

    std::ofstream out_fric("../results/torques_with_friction_out_Gaz.txt");
    out_fric << std::fixed;

    std::ofstream out_fric_only("../results/friction_out.txt");
    out_fric_only << std::fixed;

    auto q_rows   = readCSV("../traj/q.txt");
    auto dq_rows  = readCSV("../traj/q_p.txt");
    auto ddq_rows = readCSV("../traj/q_pp.txt");

    std::cout << q_rows.size() << " " << dq_rows.size() << " " << ddq_rows.size() << "\n";

    assert(q_rows.size() == dq_rows.size() && 
        q_rows.size() == ddq_rows.size());

    for (size_t i = 0; i < q_rows.size(); ++i) {
        Eigen::VectorXd q   = Eigen::Map<Eigen::VectorXd>(q_rows[i].data(),   7);
        Eigen::VectorXd dq  = Eigen::Map<Eigen::VectorXd>(dq_rows[i].data(),  7);
        Eigen::VectorXd ddq = Eigen::Map<Eigen::VectorXd>(ddq_rows[i].data(), 7);

        Eigen::VectorXd tau = pinocchio::rnea(model, data, q, dq, ddq);


        pinocchio::computeGeneralizedGravity(model, data, q);
        //std::cout << data.g.transpose() << "\n";
        for (int j = 0; j < tau.size(); ++j)
            out << tau[j] << (j < tau.size()-1 ? "," : "\n");

        Eigen::VectorXd tau_friction = compute_friction(dq);

        for (int j = 0; j < tau_friction.size(); ++j)
            out_fric_only << tau_friction[j] << (j < tau_friction.size()-1 ? "," : "\n");

        tau = tau + tau_friction;
        for (int j = 0; j < tau.size(); ++j)
            out_fric << tau[j] << (j < tau.size()-1 ? "," : "\n");
    }
    out.close();
    out_fric.close();
    out_fric_only.close();

    return 0;
}