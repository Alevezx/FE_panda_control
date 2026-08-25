#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <experimental/filesystem>
#include <thread>
#include <chrono>

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>

#include "examples_common.h"
#include "ik_solver.h"
#include "flag_stop.h"
#include "shared_state.h"
#include "calc_stop_duration.h"
#include "skeleton_reader.h"

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


#define ROBOT_IP "172.16.0.2"

void collisionCheckerThread(const pinocchio::Model& model, std::ofstream& outfile_dist) {
    pinocchio::Data data(model);
    StopDurationOptimizer optimizer(model);  // builds solver once here

    auto t_i = std::chrono::steady_clock::now();  // task start, for logging

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

        // check staleness
        bool stale = skeleton_stale.load();
\
        // Stop decision 
        if (!skeleton.empty()) {
            DistanceResult closest;
            bool should_stop = flagStop(stop_duration, q, q_p, q_pp,
                                        skeleton, model, data, &closest);
            stop_flag.store(should_stop || stale);
            shared_rh_distance.store(closest.distance);

            //  Log the closest robot/human points, mirroring dist_segment.xml 
            double t_exe = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_i).count();
 
            outfile_dist << "\t<keypoint time='" << t_exe << "'>\n";
            outfile_dist << "\t\t<stop_duration>" << stop_duration << "</stop_duration>\n";
            outfile_dist << "\t\t<min_distance>" << closest.distance << "</min_distance>";
            outfile_dist << "\t</keypoint>\n";

        } else if (stale)
        {
            stop_flag.store(true);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void executeTrajectory (
    const Eigen::VectorXd&  q_start,
    const Eigen::VectorXd&  q_end,
    double                  traj_duration,
    franka::Robot&          robot,  
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    std::ofstream&          outfile_q,
    std::ofstream&          outfile_dq) {
    
    const double TRAJ_STEP = 0.001;  // matches franka's fixed 1kHz control period
    const double TOLL_NO_STOP = 0.05; 
    double traj_duration_new = std::round(traj_duration / TRAJ_STEP) * TRAJ_STEP;
    bool flag_continue = true;

    franka::Model franka_model = robot.loadModel();

    Eigen::VectorXd q_c = q_start;

    Eigen::VectorXd q_p_lim = Eigen::VectorXd::Constant(7, 0.05);
    Eigen::VectorXd q_pp_lim = Eigen::VectorXd::Constant(7, 0.05);

    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 20.0, 20.0, 7.0}};

    enum State {RUNNING, STOPPING};     // logica a stati del movimento
    State current_state = RUNNING;

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
 
        bool stop_flag_local = false;
        double stop_duration = 0.4;
        Eigen::VectorXd q_c_local  = q_c;
        Eigen::VectorXd q_p_local  = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd q_pp_local = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd q_meas     = q_c;

        double t = 0.0;  // accumulated from period.toSec()
        int loop_counter = 0;
        
        // braking variables
        bool is_braking = false;
        double t_brake = 0.0;
        int stop_num_points = 0;
        int stop_loop_counter = 0;
        
        Eigen::VectorXd q_0_brake = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd v_0_brake = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd a_0_brake = Eigen::VectorXd::Zero(7);
        Eigen::MatrixXd Q_stop, Q_p_stop, Q_pp_stop;

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        // execute trajectory using libfranka
        try {
            robot.control([&](const franka::RobotState& state,
                            franka::Duration period) -> franka::Torques
            {
                // control needs to just impose the trajectory, be it normal or stopping
                // outside the control loop we will compute recovering time if the robot is paused
                // this will help meet franka's 1kHz hard limit

                // Update shared robot state for collision checker 
                {
                    std::lock_guard<std::mutex> lock(robot_state_mutex);
                    for (int j = 0; j < 7; ++j) {
                        shared_robot_state.q[j]   = state.q[j];
                        shared_robot_state.q_p[j] = state.dq[j];
                        // finite difference for q_pp
                        shared_robot_state.q_pp[j] = state.ddq_d[j];
                    }
                }

                // Fetch dynamic properties from franka model
                std::array<double, 7> gravity_array = franka_model.gravity(state);
                std::array<double, 7> coriolis_array = franka_model.coriolis(state);
                std::array<double, 49> mass_array = franka_model.mass(state);
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> mass(mass.data());
                
                // Read measured state
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_m(state.q.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_m(state.dq.data());

                q_meas = q_m;

                // stop condition check
                if (current_state == RUNNING && stop_flag.load()) {
                    stop_flag_local = stop_flag.load();
                    
                    stop_duration = shared_stop_duration.load();
                    
                    q_0_brake = q_m;
                    v_0_brake = dq_m;
                    a_0_brake = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.ddq_d.data());

                    stop_num_points = static_cast<int>(stop_duration / TRAJ_STEP) + 1;
                    // compute poly5 for stopping
                    Q_stop.resize(stop_num_points, 7);
                    Q_p_stop.resize(stop_num_points, 7);
                    Q_pp_stop.resize(stop_num_points, 7);
                    for (int j = 0; j < 7; ++j) {
                        Eigen::MatrixXd tj = trajPoly5Numeric(
                            q_0_brake(j), v_0_brake(j), a_0_brake(j),
                            q_0_brake(j), 0.0, 0.0,
                            stop_duration, stop_num_points);
                        Q_stop.col(j) = tj.col(0);
                        Q_p_stop.col(j) = tj.col(1);
                        Q_pp_stop.col(j) = tj.col(2);
                    }

                    t_brake = 0.0;
                    stop_loop_counter = 0;
                    current_state = STOPPING;
                }

                // Desired position & velocity placeholders
                Eigen::Matrix<double, 7, 1> q_d;
                Eigen::Matrix<double, 7, 1> dq_d;
                Eigen::Matrix<double, 7, 1> ddq_d;

                // compute target values
                if (current_state == STOPPING) {
                    t_brake += period.toSec();
                    if (t_brake >= stop_duration) {
                        // gravity compensation torque
                        //std::array<double, 7> tau_g = franka_model.gravity(state);
                        return franka::MotionFinished(franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    }

                    int stop_idx = static_cast<int>(t_brake / TRAJ_STEP);
                    stop_idx = std::min(stop_idx, stop_num_points - 1);
                    //stop_loop_counter++;

                    q_d = Q_stop.row(stop_idx).transpose();
                    dq_d = Q_p_stop.row(stop_idx).transpose();
                    ddq_d = Q_pp_stop.row(stop_idx).transpose();
                } 
                else if (current_state == RUNNING) {
                    // Normal execution
                    t += period.toSec();
                    //  Get current trajectory index 
                    int idx = static_cast<int>(t / TRAJ_STEP);
                    idx = std::min(idx, num_points - 1);
                    //loop_counter++;
                    
                    //  Check if trajectory is complete 
                    if (t >= traj_duration_new) {
                        //std::array<double, 7> tau_g = franka_model.gravity(state);
                        return franka::MotionFinished(
                            franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    }

                    q_d = Q.row(idx).transpose();
                    dq_d = Q_p.row(idx).transpose();
                    ddq_d = Q_pp.row(idx).transpose();
                }

                //  Log joint positions 
                outfile_q << "\t<keypoint time='" << t << "'>\n";
                for (int j = 0; j < 7; ++j)
                    outfile_q << "\t\t<point id='" << j << "'>"
                            << state.q[j] << "</point>\n";
                outfile_q << "\t</keypoint>\n";

                //  Log joint velocities 
                outfile_dq << "\t<keypoint time='" << t << "'>\n";
                for (int j = 0; j < 7; ++j)
                    outfile_dq << "\t\t<point id='" << j << "'>"
                            << state.dq[j] << "</point>\n";
                outfile_dq << "\t</keypoint>\n";

                // compute torques
                Eigen::Matrix<double, 7, 1> tau_d, tau_fb;
                Eigen::Matrix<double, 7, 1> tau_ff = /*mass * ddq_d +*/ coriolis;

                // PD controller
                for (size_t i = 0; i < 7; ++i) {
                    tau_fb(i) = k_gains[i]*(q_d(i)-q_m(i)) + 
                                d_gains[i]*(dq_d(i)-dq_m(i));
                }

                tau_d = tau_ff + tau_fb;
                std::array<double, 7> tau_cmd;
                Eigen::VectorXd::Map(&tau_cmd[0], 7) = tau_d;
                std::array<double, 7> tau_d_rate_limited =
                    franka::limitRate(franka::kMaxTorqueRate, tau_cmd, state.tau_J_d);
                return tau_d_rate_limited;
                
                
                //return franka::Torques(tau_cmd);
            });
        } catch (const franka::Exception& e) {
                std::cerr << "executeTrajectory: robot fault during main motion: " << e.what() << "\n";
                robot.automaticErrorRecovery();
                throw;
        }

        // stop event
        bool do_stop = (q_end - q_meas).cwiseAbs().maxCoeff() > TOLL_NO_STOP;

        if (stop_flag_local) {
            std::cout << "\t Traiettoria di stop completata\n";

            //  Recalculate duration for retry 
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

            // wait for obstacle to move
            while (shared_rh_distance.load() < 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // Reset stop flag for retry
            stop_flag.store(false);
            current_state = RUNNING;
        } else
        {
            //  Robot reached target 
            flag_continue = false;
            std::cout << "Fine del movimento alla configurazione finale\n";
        }
    }
}


void executeTask(
    std::vector<Eigen::VectorXd>& q_traj,
    std::vector<double>& traj_durations,
    franka::Robot& robot,
    const pinocchio::Model& model) {

    // apri file per logging
    std::string pathname = "../logs/";
    std::ofstream outfile_q(pathname + "q_robot.xml");
    std::ofstream outfile_dist(pathname + "dist_segment.xml");
    std::ofstream outfile_dq(pathname + "dq_robot.xml");

    outfile_q << "<q>\n";
    outfile_dist << "<distance>\n";
    outfile_dq << "<dq>\n";
    std::cout << "Outfile initiated\n";

    // thread per scheletro e collision check
    // usa loadSkeletonXML per virtual mode
    // usa receiveSkeletonLive per real time mode
    std::thread skeleton_thread(receiveSkeletonMerged, "localhost", 10, "MERGED");

    std::thread collision_thread(collisionCheckerThread,
    std::cref(model), std::ref(outfile_dist));

    //StopDurationOptimizer optimizer(model);
    pinocchio::Data data(model);

    double traj_duration_prev = 0.0;
    Eigen::VectorXd q_traj_prev = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(Q_HOME.data());

    auto task_start = std::chrono::steady_clock::now();  // real wall-clock, diagnostic only

    // esegui la traiettoria secondo i tempi in traj_duration
    try {
        for (int i = 0; i < (int)traj_durations.size(); i++) {
            double segment_duration = traj_durations[i] - traj_duration_prev;
            executeTrajectory(q_traj_prev, q_traj[i], segment_duration,
                            robot, model, data,
                            outfile_q, outfile_dq);

            traj_duration_prev = traj_durations[i];
            q_traj_prev = q_traj[i];
        }
    } catch (...) {
        // A robot fault propagated up from executeTrajectory. Still need to
        // stop the skeleton/collision threads and close the log files
        // properly before this task ends otherwise a real fault leaks
        // threads and leaves truncated/unflushed log files behind, on top
        // of the actual robot error.
        std::cerr << "executeTask: aborting due to robot fault, cleaning up\n";
        trajectory_done.store(true);
        skeleton_thread.join();
        collision_thread.join();
        outfile_q << "</q>\n";
        outfile_dist << "</distance>\n";
        outfile_dq << "</dq>\n";
        outfile_q.close();
        outfile_dist.close();
        outfile_dq.close();
        throw;  // let the caller (main) know the task did not complete

    }

    double real_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - task_start).count();
    std::cout << "Nominal trajectory duration: " << traj_durations.back()
              << "s, real wall-clock elapsed: " << real_elapsed << "s\n";

    trajectory_done.store(true);

    skeleton_thread.join();
    collision_thread.join();

    outfile_q << "</q>\n";
    outfile_dist << "</distance>\n";
    outfile_dq << "</dq>\n";
    outfile_q.close();
    outfile_dist.close();
    outfile_dq.close();
}

int main(int argc, char** argv) {    
    // Caricamento del modello URDF del robot Panda utilizzando Pinocchio per calcoli cinematici avanzati
    pinocchio::Model model;
    pinocchio::urdf::buildModel("../urdf/panda.urdf", model);
    pinocchio::Data data(model);
    const pinocchio::FrameIndex EE_ID = model.getFrameId("panda_link8");


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
    Eigen::VectorXd q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(Q_HOME.data());
    
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
        q0 = result.q;
    }
    
    // movimento del robot
    std::cout << "This will move the robot to the home position!\n";
    std::cout << "Press Enter to continue...\n";
    std::cin.get();
    try {
        franka::Robot robot(ROBOT_IP);
        setDefaultBehavior(robot);

        std::vector<double> q_home(Q_HOME.begin(), Q_HOME.end());
        double move_duration = 5.0; // seconds
        double time = 0.0;
        std::array<double, 7> q_start;
        bool initial_state = true;

        robot.control([&](const franka::RobotState& state,
                           franka::Duration period) -> franka::JointPositions{
            time += period.toSec();

            if (initial_state) {
                q_start = state.q_d;
                initial_state = false;
            }

            // Smooth step interpolation (zero velocity at start and end)
            double s = time / move_duration;
            s = std::min(s, 1.0);
            double alpha = s * s * (3.0 - 2.0 * s);  // smoothstep

            std::array<double, 7> q_cmd;
            for (int j = 0; j < 7; ++j)
                q_cmd[j] = q_start[j] + alpha * (q_home[j] - q_start[j]);

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
    } catch (const std::exception& ex) {
        // Anything else unexpected (not a robot fault specifically)
        // still exit cleanly with a clear message rather than letting it
        // terminate the process unhandled.
        std::cerr << "Unexpected error: " << ex.what() << "\n";
        trajectory_done.store(true);
        return -1;
    }

    return 0;
}