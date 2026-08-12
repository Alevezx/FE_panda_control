
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <thread>
#include <chrono>

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>

#include "examples_common.h"
#include "ik_solver.h"
#include "flag_stop.h"
#include "shared_state.h"
#include "calc_stop_duration.h"
#include "skeleton_reader.h"

#define SIMULATE

#define ROBOT_IP "172.16.0.2"

void collisionCheckerThread(const pinocchio::Model& model) {
    pinocchio::Data data(model);
    StopDurationOptimizer optimizer(model);  // builds solver once here

    while (!trajectory_done.load()) {
        // Read current robot state
        std::array<double, 7> q_arr, qp_arr, qpp_arr;
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex);
            q_arr   = shared_robot_state.q;
            qp_arr  = shared_robot_state.q_p;
            qpp_arr = shared_robot_state.q_pp;
        }

        // Convert to Eigen
        Eigen::VectorXd q   = Eigen::Map<Eigen::VectorXd>(q_arr.data(),   7);
        Eigen::VectorXd q_p = Eigen::Map<Eigen::VectorXd>(qp_arr.data(),  7);
        Eigen::VectorXd q_pp= Eigen::Map<Eigen::VectorXd>(qpp_arr.data(), 7);

        // Run optimization
        double stop_duration = optimizer.solve(q, q_p, q_pp);
        shared_stop_duration.store(stop_duration);

        // Read skeleton
        Skeleton skeleton;
        {
            std::lock_guard<std::mutex> lock(skeleton_mutex);
            skeleton = shared_skeleton;
        }


        // Stop decision 
        if (!skeleton.empty()) {
            bool should_stop = flagStop(stop_duration, q, q_p, q_pp,
                                        skeleton, model, data);
            stop_flag.store(should_stop);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void executeTrajectory(
    const Eigen::VectorXd&  q_start,
    const Eigen::VectorXd&  q_end,
    double                  traj_duration,
    franka::Robot&          robot,
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    //StopDurationOptimizer&  optimizer,
    std::ofstream&          outfile_q) {
    
    const double TRAJ_STEP = 0.01;
    const double TOLL_NO_STOP = 0.05; 
    double traj_duration_new = std::round(traj_duration / TRAJ_STEP) * TRAJ_STEP;
    bool flag_continue = true;

    Eigen::VectorXd q_c = q_start;

    Eigen::VectorXd q_p_lim = Eigen::VectorXd::Constant(7, 0.05);
    Eigen::VectorXd q_pp_lim = Eigen::VectorXd::Constant(7, 0.05);

    while (flag_continue) {
        int num_points = static_cast<int>(traj_duration / TRAJ_STEP) + 1;

        Eigen::MatrixXd Q(num_points, 7);
        Eigen::MatrixXd Q_p(num_points, 7);
        Eigen::MatrixXd Q_pp(num_points, 7);

        for (int j = 0; j < 7; j++) {
            // compute trajectory and limits

            Eigen::MatrixXd traj = trajPoly5Numeric(q_c(j), 0.0, 0.0,
                                                    q_end(j), 0.0, 0.0,
                                                    traj_duration_new, num_points);
            Q.col(j)    = traj.col(0);
            Q_p.col(j)  = traj.col(1);
            Q_pp.col(j) = traj.col(2);

            double max_vel = Q_p.col(j).cwiseAbs().maxCoeff();
            double max_acc = Q_pp.col(j).cwiseAbs().maxCoeff();
            if (max_vel > 0.0) q_p_lim(j)  = max_vel;
            if (max_acc > 0.0) q_pp_lim(j) = max_acc;
        }

        // execute trajectory using libfranka
        bool stop_flag_local = false;
        double stop_duration = 0.0;
        Eigen::VectorXd q_c_local  = q_c;
        Eigen::VectorXd q_p_local  = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd q_pp_local = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd q_meas     = q_c;

        auto t_start = std::chrono::steady_clock::now();

        robot.control([&](const franka::RobotState& state,
                           franka::Duration period) -> franka::JointPositions
        {
            double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();

            // ── Update shared robot state for collision checker ───────
            {
                std::lock_guard<std::mutex> lock(robot_state_mutex);
                for (int j = 0; j < 7; ++j) {
                    shared_robot_state.q[j]   = state.q[j];
                    shared_robot_state.q_p[j] = state.dq[j];
                    // finite difference for q_pp
                    shared_robot_state.q_pp[j] = state.ddq_d[j];
                }
            }

            // ── Store measured position ───────────────────────────────
            for (int j = 0; j < 7; ++j)
                q_meas(j) = state.q[j];

            // ── Get current trajectory index ──────────────────────────
            int idx = static_cast<int>(t / TRAJ_STEP);
            idx = std::min(idx, num_points - 1);

            q_c_local  = Q.row(idx).transpose();
            q_p_local  = Q_p.row(idx).transpose();
            q_pp_local = Q_pp.row(idx).transpose();

            // ── Check stop flag from collision checker thread ─────────
            if (stop_flag.load())
            {
                stop_flag_local = true;
                // Read stop duration from optimizer
                //stop_duration = optimizer.solve(q_c_local, q_p_local, q_pp_local);
                stop_duration = shared_stop_duration.load();
                return franka::MotionFinished(
                    franka::JointPositions(state.q_d));
            }

            // ── Log joint positions ───────────────────────────────────
            outfile_q << "\t<keypoint time='" << t << "'>\n";
            for (int j = 0; j < 7; ++j)
                outfile_q << "\t\t<point id='" << j << "'>"
                          << state.q[j] << "</point>\n";
            outfile_q << "\t</keypoint>\n";

            // ── Check if trajectory is complete ───────────────────────
            if (t >= traj_duration_new)
                return franka::MotionFinished(
                    franka::JointPositions(state.q_d));

            return franka::JointPositions(
                std::array<double,7>{Q(idx,0), Q(idx,1), Q(idx,2),
                                     Q(idx,3), Q(idx,4), Q(idx,5), Q(idx,6)});
        });

        // stop event
        bool do_stop = (q_end - q_meas).cwiseAbs().maxCoeff() > TOLL_NO_STOP;

        if (stop_flag_local && do_stop) {
            std::cout << "\nIngaggio traiettoria di stop\n";

            robot.control([&](const franka::RobotState& state,
                               franka::Duration period) -> franka::JointPositions
            {
                static double t = 0.0;
                t += period.toSec();

                int idx = static_cast<int>(t / TRAJ_STEP);

                // Braking trajectory: q_meas → q_meas with zero vel/acc
                Eigen::MatrixXd stop_traj = trajPoly5Numeric(
                    q_meas(0), q_p_local(0), q_pp_local(0),
                    q_meas(0), 0.0, 0.0,
                    stop_duration, static_cast<int>(stop_duration/TRAJ_STEP)+1);

                if (t >= stop_duration)
                    return franka::MotionFinished(
                        franka::JointPositions(state.q_d));

                std::array<double, 7> q_cmd;
                for (int j = 0; j < 7; ++j) {
                    Eigen::MatrixXd tj = trajPoly5Numeric(
                        q_meas(j), q_p_local(j), q_pp_local(j),
                        q_meas(j), 0.0, 0.0,
                        stop_duration,
                        static_cast<int>(stop_duration/TRAJ_STEP)+1);
                    int i = std::min(idx, (int)tj.rows()-1);
                    q_cmd[j] = tj(i, 0);
                }

                return franka::JointPositions(q_cmd);
            });

            // ── Recalculate duration for retry ────────────────────────
            Eigen::VectorXd vect_t_vel(7), vect_t_acc(7);
            for (int j = 0; j < 7; ++j) {
                vect_t_vel(j) = 15.0 * std::abs(q_end(j) - q_c_local(j))
                              / (8.0 * q_p_lim(j));
                vect_t_acc(j) = std::sqrt(10.0 * std::abs(q_end(j) - q_c_local(j))
                              / (std::sqrt(3.0) * q_pp_lim(j)));
            }

            double t_vel    = vect_t_vel.maxCoeff();
            double t_acc    = vect_t_acc.maxCoeff();
            double added_time = std::max({t_vel, t_acc, 0.5});

            traj_duration_new = std::round(added_time / TRAJ_STEP) * TRAJ_STEP;
            q_c = q_meas;  // restart from where we stopped

            // Reset stop flag for retry
            stop_flag.store(false);
            // Wait for obstacle to clear before retrying
            /*std::cout << "Waiting for obstacle to clear...\n";
            while (stop_flag.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::cout << "Obstacle cleared, retrying trajectory\n";*/
        }
        else
        {
            // ── Robot reached target ──────────────────────────────────
            flag_continue = false;
            std::cout << "Fine del movimento all configurazione finale\n";
        }
    }
}


void executeTask(
    std::vector<Eigen::VectorXd>& q_traj,
    std::vector<double>& traj_durations,
    franka::Robot& robot,
    const pinocchio::Model& model) {

    // --- apri file per logging ---
    std::string pathname = "../logs/";
    std::ofstream outfile_q(pathname + "q_robot.xml");
    std::ofstream outfile_dist(pathname + "dist_segment.xml");

    outfile_q << "<q>\n";
    outfile_dist << "<distance>\n";

    // --- thread per scheletro e collision check ---
    // usa loadSkeletonXML per virtual mode
    // usa receiveSkeletonLive per real time mode
    std::thread skeleton_thread(loadSkeletonXML,
    "../skeleton/skeleton_coords.xml");

    std::thread collision_thread(collisionCheckerThread,
    std::cref(model));

    //StopDurationOptimizer optimizer(model);
    pinocchio::Data data(model);

    double traj_duration_prev = 0.0;
    Eigen::VectorXd q_traj_prev(7);
    q_traj_prev << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;

    // esegui la traiettoria secondo i tempi in traj_duration
    for (int i = 0; i < (int)traj_durations.size(); i++) {
        double segment_duration = traj_durations[i] - traj_duration_prev;
        executeTrajectory(q_traj_prev, q_traj[i], segment_duration,
                          robot, model, data,
                          outfile_q);

        traj_duration_prev = traj_durations[i];
        q_traj_prev = q_traj[i];
    }

    trajectory_done.store(true);

    skeleton_thread.join();
    collision_thread.join();

    outfile_q << "</q>\n";
    outfile_dist << "</distance>\n";
    outfile_q.close();
    outfile_dist.close();
}

int main(int argc, char** argv) {    
    // Caricamento del modello URDF del robot Panda utilizzando Pinocchio per calcoli cinematici avanzati
    pinocchio::Model model;
    pinocchio::urdf::buildModel("../urdf/panda.urdf", model);
    pinocchio::Data data(model);
    const pinocchio::FrameIndex EE_ID = model.getFrameId("panda_link7");


    // traiettoria nello spazio 3D
    //std::string filename = "/home/panda/Desktop/demo/collision_avoidance/p_traj.txt";
    std::string filename = "../traj/traiettoria.txt";
    std::ifstream filein(filename);
    if (!filein.is_open()) {
        std::cout << "Impossibile aprire il file di traiettoria" << std::endl;
        return 1;
    }

    std::vector<Eigen::Vector3d> p_traj = {};  // vettore di waypoint nello spazio 3D
    std::string line;

    while (getline(filein, line)) {
        std::stringstream ss(line);
        std::string temp;
        std::vector<double> a = {};

        while (getline(ss, temp, ',')) {
            a.push_back(std::stod(temp));
        }
        
        p_traj.push_back(Eigen::Vector3d(a[0], a[1], a[2]));
    }
    filein.close();
    
    // vettore dei tempi
    //filename = "/home/panda/Desktop/demo/collision_avoidance/t_traj.txt";
    filename = "../traj/t.txt";
    std::ifstream filein_t(filename);
    if (!filein_t.is_open()) {
        std::cout << "Impossibile aprire il file dei tempi" << std::endl;
        std::cout << filename << std::endl;
        return 1;
    }
    std::vector<double> t_traj = {};
    while (getline(filein_t, line)) {
        t_traj.push_back(std::stod(line));
    }
    filein_t.close();
    
    // configurazione iniziale
    Eigen::VectorXd q0(7);
    q0 << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
    
    // risoluzione cinematica inversa
    const int N = p_traj.size();
    std::vector<Eigen::VectorXd> q_traj(N);
    for (int i = 0; i < N; ++i) {
        pinocchio::SE3 target = buildTarget(p_traj[i]);

        IKResult result = ikine_LM(model, data, target, EE_ID, q0);
        
        if (!result.converged) {
            std::cerr << "Warning: IK did not converge at point  " 
                      << i << " (residual: " << result.residual << ")\n";
        }
        q_traj[i] = result.q;
    }
    
    // movimento del robot
    std::cout << "This will move the robot to the home position\n";
    std::cout << "Press Enter to continue...\n";
    std::cin.get();
    try {
        franka::Robot robot(ROBOT_IP);
        setDefaultBehavior(robot);

        std::vector<double> q_home = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
        double move_duration = 5.0; // seconds
        double time = 0.0;

        robot.control([&](const franka::RobotState& state,
                           franka::Duration period) -> franka::JointPositions{
            time += period.toSec();

            // Smooth step interpolation (zero velocity at start and end)
            double s = time / move_duration;
            s = std::min(s, 1.0);
            double alpha = s * s * (3.0 - 2.0 * s);  // smoothstep

            std::array<double, 7> q_cmd;
            for (int j = 0; j < 7; ++j)
                q_cmd[j] = state.q_d[j] + alpha * (q_home[j] - state.q_d[j]);

            if (time > move_duration) {
                return franka::MotionFinished(franka::JointPositions(q_cmd));
            }
            return franka::JointPositions(q_cmd);
        });

        std::cout << "Finished moving to initial joint configuration\n";
        std::cout << "WARNING: Collision thresholds are set to high values.\n";
        std::cout << "Make sure you have the user stop at hand!\n";
        std::cout << "This will start the joint control task\n";
        std::cout << "Press Enter to continue...\n";
        std::cin.get();

        executeTask(q_traj, t_traj, robot, model);

        std::cout << "Trajectory completed, " << t_traj.back() 
                << " seconds have passed, shutting down example\n";

    } catch (const franka::Exception& ex){
        std::cerr << "Franka exception: " << ex.what() << "\n";
        trajectory_done.store(true);
        return -1;
    }

    return 0;
}