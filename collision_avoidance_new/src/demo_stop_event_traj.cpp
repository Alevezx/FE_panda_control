#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <atomic>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <Eigen/Dense>

// Pinocchio headers
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math/rpy.hpp>
#include "pinocchio/parsers/urdf.hpp"

#include "examples_common.h"
#include "header_stop_event.h"
#include "flag_stop.h"
#include "calc_stop_duration.h"

#define ROBOT_IP "172.16.0.2"

// start configuration
inline constexpr std::array<double, 7> Q_HOME = {
    -0.3180,
    0.4110,
    -0.2590,
    -1.5090,
    0.1200,
    1.9870,
    0.2620
};

void collision_checking_loop(const pinocchio::Model& pin_model) {
    pinocchio::Data pin_data(pin_model);
    StopDurationOptimizer optimizer(pin_model);

    while (program_running.load()) {
        // 1. read latest robot state
        std::array<double, 7> q_read, dq_read, ddq_read;
        for (int j = 0; j < 7; ++j) {
            q_read[j] = current_q[j].load();
            dq_read[j] = current_dq[j].load();
            ddq_read[j] = current_ddq[j].load();
        }

        // Convert to Eigen
        Eigen::VectorXd q    = Eigen::Map<Eigen::VectorXd>(q_read.data(),   7);
        Eigen::VectorXd q_p  = Eigen::Map<Eigen::VectorXd>(dq_read.data(),  7);
        Eigen::VectorXd q_pp = Eigen::Map<Eigen::VectorXd>(ddq_read.data(), 7);

        // 2. read skeleton data
        Skeleton skeleton;
        {
            std::lock_guard<std::mutex> lock(skeleton_mutex);
            skeleton = shared_skeleton;
        }

        // 3. optimize stop time with casadi
        if (!stop_flag.load() && !skeleton.empty()) {
            double T_opt = optimizer.solve(q, q_p, q_pp);
            optimized_stop_duration.store(T_opt);
            
            // 4. check if collision is possible
            //compute capsules and check
            DistanceResult closest;
            bool stop = flagStop(T_opt, q_read, dq_read, ddq_read, skeleton, pin_model, pin_data, &closest);
            stop_flag.store(stop);
        }

        // run slower than main loop to save CPU time
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    tau_max << 87, 87, 87, 87, 12, 12, 12;
    tau_rate_max << 1000, 1000, 1000, 1000, 1000, 1000, 1000;

    std::vector<franka::RobotState> log_data;
    log_data.reserve(200000);

    // ---------------------------------------------------------
    // 1. Read p_traj.txt into a 2D vector (vector of vectors)
    // ---------------------------------------------------------
    std::ifstream p_file("../traj/traiettoria.txt");
    std::vector<std::vector<double>> p_traj;

    if (p_file.is_open()) {
        std::string line;
        while (std::getline(p_file, line)) {
            std::vector<double> data;
            std::stringstream ss(line);
            std::string s;
            
            // Split the line by tab '\t'
            while (std::getline(ss, s, '\t')) {
                if (!s.empty()) { // prevent empty strings from throwing errors
                    data.push_back(std::stod(s));
                }
            }
            p_traj.push_back(data);
        }
        p_file.close();
    } else {
        std::cerr << "Failed to open p_traj.txt" << std::endl;
    }

    // ---------------------------------------------------------
    // 2. Read t_traj.txt into a 1D vector
    // ---------------------------------------------------------
    std::ifstream t_file("../traj/t.txt");
    std::vector<double> t_traj;

    if (t_file.is_open()) {
        std::string line;
        while (std::getline(t_file, line)) {
            std::stringstream ss(line);
            std::string s;
            
            // Split the line by tab '\t'
            while (std::getline(ss, s, '\t')) {
                if (!s.empty()) {
                    t_traj.push_back(std::stod(s));
                }
            }
        }
        t_file.close();
    } else {
        std::cerr << "Failed to open t_traj.txt" << std::endl;
    }

    pinocchio::Model pin_model;
    // Safely attempt to load the URDF
    try {
        pinocchio::urdf::buildModel("../urdf/panda.urdf", pin_model);
    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: Failed to load URDF." << std::endl;
        std::cerr << "Exception details: " << e.what() << std::endl;
        return -1;
    }
    if (pin_model.nv == 0) {
        std::cerr << "CRITICAL ERROR: URDF loaded, but the model has 0 degrees of freedom." << std::endl;
        return -1;
    }

    const int EE_ID = pin_model.getFrameId("panda_link8"); 
    pinocchio::Data data(pin_model);
    Eigen::VectorXd q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(Q_HOME.data());

    std::vector<Eigen::VectorXd> q_traj;
    q_traj.reserve(p_traj.size());

    //Levenberg-Marquardt (Damped Least Squares) Parameters
    const double eps = 1e-4;        // Convergence threshold
    const int IT_MAX = 500;         // Max iterations
    const double damp = 1e-4;       // Damping parameter (lambda)
    const double step_size = 0.5;   // Step scaling to avoid overshooting

    Eigen::MatrixXd J(6, pin_model.nv);

    // Iterate through the trajectory
    for (size_t i = 0; i < p_traj.size(); ++i) {
        // Construct the desired SE3 pose
        pinocchio::SE3 oMdes;
        // Assuming p_traj holds [x, y, z] translations
        oMdes.translation() << p_traj[i][0], p_traj[i][1], p_traj[i][2]; 
        
        // Multiply by RPY(0, 0, pi) like in Python
        oMdes.rotation() = pinocchio::rpy::rpyToMatrix(0.0, 0.0, M_PI);

        // Seed current point with q0 or previous trajectory point
        Eigen::VectorXd q = (i == 0) ? q0 : q_traj.back(); 

        bool converged = false;

        // IK Optimization Loop (Levenberg-Marquardt)
        for (int iter = 0; iter < IT_MAX; ++iter) {
            // 1. Update frame placements
            pinocchio::framesForwardKinematics(pin_model, data, q);

            // 2. FIXED: Transformation from CURRENT frame to DESIRED frame
            const pinocchio::SE3 iMd = data.oMf[EE_ID].actInv(oMdes);
            const Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

            if (err.norm() < eps) {
                converged = true;
                break;
            }

            // 3. Compute local frame Jacobian
            pinocchio::computeFrameJacobian(pin_model, data, q, EE_ID, pinocchio::LOCAL, J);

            // 4. Solve Damped Least Squares: \Delta q = J^T * (J * J^T + \lambda * I)^-1 * err
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += damp;
            
            Eigen::VectorXd v = J.transpose() * JJt.ldlt().solve(err);

            // 5. Integrate update with step size
            q = pinocchio::integrate(pin_model, q, v * step_size);

            // 6. FIXED: Clamp joint positions to robot's URDF physical limits
            q = q.cwiseMax(pin_model.lowerPositionLimit).cwiseMin(pin_model.upperPositionLimit);
        }

        if (!converged) {
            std::cerr << "Warning: IK did not converge for step " << i << std::endl;
        }
        // Store the result
        q_traj.push_back(q);
    }

    std::cout << "Inverse Kinematics is done\nNow the robot will move to the start position\n";
    std::cout << "Press Enter to contunue\n";
    std::cin.get();
    franka::Robot robot(ROBOT_IP);
    setDefaultBehavior(robot);
    robot.setLoad(0.0, {0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0});
    franka::Model model = robot.loadModel();

    try {
        bool initial_state_set = false;
        std::array<double, 7>initial_q;

        double duration = 5.0;
        double time = 0.0;
        // Moving to initial position
        robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
            time += period.toSec();

            if (!initial_state_set) {
                initial_q = state.q_d;
                initial_state_set = true;
            } 

            //smoothstep
            double s = time / duration;
            s = std::min(s, 1.0);
            double alpha = s * s * (3.0 - 2.0 * s);
            std::array<double, 7> q_cmd;
            for (int j = 0; j < 7; ++j) {
                q_cmd[j] = initial_q[j] + alpha * (q0(j) - initial_q[j]);

            }

            if (time >= duration) {
                return franka::MotionFinished(franka::JointPositions(q_cmd));
            }
            return franka::JointPositions(q_cmd);
        });
    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception caught in initial movement: " << e.what() << "\n";
        return -1;
    }

    std::cout << "Finished moving to initial joint configuration\n";
    std::cout << "Make sure you have the user stop at hand\n";
    std::cout << "Press Enter to start the task\n";
    std::cin.get();

    
    
    try {
        // start collision checking + stop duration optimizer thread
        std::thread collision_thread(collision_checking_loop);

        ExecutionState state = ExecutionState::RUNNING;
        double segment_time = 0.0;
        double stop_time = 0.0;
        double stop_duration = 0.4;
        int current_segment = 0;
        double segment_duration = t_traj[current_segment];
        Vector7d q_traj_prev = q0;
        Vector7d q_traj_next = q_traj[current_segment];
        Poly5Matrix coeffs, stop_coeffs;
        for (int j = 0; j < 7; ++j) {
            coeffs.col(j) = compute_poly5_coeff(q_traj_prev[j], q_traj_next[j], 0.0, 0.0, 0.0, 0.0, segment_duration);
        }
        
        // Hard 1kHz loop
        robot.control ([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
            std::array<double, 7> q_cmd;
            if (log_data.size() < log_data.capacity()) log_data.push_back(robot_state);
            
            // Share current robot state with the collision thread
            for (int i = 0; i < 7; ++i) {
                current_q[i].store(robot_state.q_d[i]);
                current_dq[i].store(robot_state.dq_d[i]);
                current_ddq[i].store(robot_state.ddq_d[i]);
            }

            // check for stop trigger
            if (state == ExecutionState::RUNNING && stop_flag.load()) {
                // load stop duration from optimizer
                stop_duration = optimized_stop_duration.load();

                // get initial robot state
                double t = segment_time; double t2 = t * t; double t3 = t2 * t;
                for (int j = 0; j < 7; ++j) {
                    double qi = robot_state.q_d[j];
                    double vi = robot_state.dq_d[j];
                    double ai = robot_state.ddq_d[j];

                    stop_coeffs.col(j) = compute_poly5_coeff(qi, qi, vi, 0.0, ai, 0.0, stop_duration);
                }
                stop_time = 0.0;
                state = ExecutionState::STOPPING;
            }

            // == normal running phase ==
            if (state == ExecutionState::RUNNING) {
                segment_time += period.toSec();
                double t = segment_time; double t2 = t*t; double t3 = t2*t;
                double t4 = t3*t; double t5 = t4*t;
                for (int j = 0; j < 7; ++j) {
                    q_cmd[j] = coeffs(0,j) + coeffs(1,j) * t + coeffs(2,j) * t2 +
                            coeffs(3,j) * t3 + coeffs(4,j) * t4 + coeffs(5,j) * t5;
                }
                
                // switch segment if waypoint is reached
                if (segment_time >= segment_duration) {
                    current_segment++;
                    if (current_segment >= q_traj.size())
                    {
                        return franka::MotionFinished(franka::JointPositions(q_cmd));
                    }
                    segment_duration = t_traj[current_segment] - t_traj[current_segment-1];
                    q_traj_prev = q_traj_next;
                    q_traj_next = q_traj[current_segment];
                    segment_time = 0.0;

                    // update matrix
                    for (int j = 0; j < 7; ++j) {
                        coeffs.col(j) = compute_poly5_coeff(q_traj_prev[j], q_traj_next[j], 0.0, 0.0, 0.0, 0.0, segment_duration);
                    }
                }
                return franka::JointPositions(q_cmd); 
            } 
            else if (state == ExecutionState::STOPPING) {
                // == stopping phase ==
                stop_time += period.toSec();
                double t = stop_time; double t2 = t*t; double t3 = t2*t;
                double t4 = t3*t; double t5 = t4*t;
                for (int j = 0; j < 7; ++j) {
                    q_cmd[j] = coeffs(0,j) + coeffs(1,j) * t + coeffs(2,j) * t2 +
                            coeffs(3,j) * t3 + coeffs(4,j) * t4 + coeffs(5,j) * t5;
                }
                if (stop_time >= stop_duration) {
                    state = ExecutionState::PAUSED;
                }
                return franka::JointPositions(q_cmd);
            }
        });
            
    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception in main loop: " << e.what() << "\n";
        save_log_to_csv(log_data, "../logs/log_error_franka.csv");
    } catch (const std::exception& e) {
        std::cerr << "Exception during main loop: " << e.what() << "\n";
        save_log_to_csv(log_data, "../logs/log_error_std.csv");
    }
    
    std::cout << "Trajectory completed, " << t_traj.back() << " seconds have passed, shutting down\n";

    // save log and clean up before exiting
    save_log_to_csv(log_data, "../logs/log_success.csv");
    program_running.store(false);
    if (collision_thread.joinable()) {
        collision_thread.join();
    }
    return 0;
}
