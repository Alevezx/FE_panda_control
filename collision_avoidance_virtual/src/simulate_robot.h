#pragma once
#include <array>
#include <functional>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <stdexcept>

// start configuration
inline constexpr std::array<double, 7> Q_HOME = {
    1.9761036775835699,
    -1.0967421899063394,
    -2.17676965855803,
    -1.5385201110158262,
    -0.9831011303153109,
    2.126928828637027,
    -2.21494675444473
};


// Mirror libfranka types
namespace franka_sim {
    struct Duration {
        double toSec() const {return 0.001;}  
        // return a fixed 1kHz
    };

    struct RobotState {
        std::array<double, 7> q     = Q_HOME;
        std::array<double, 7> dq    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, 7> ddq_d = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, 7> q_d   = Q_HOME;
    };

    struct JointPositions {
        std::array<double, 7> q;
        bool motion_finished = false;

        JointPositions(std::array <double, 7> q) : q(q) {}
    };

    struct Exception : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };

    inline JointPositions MotionFinished(JointPositions jp) {
        jp.motion_finished = true;
        return jp;
    }

    class Robot {
        public:
        Robot(const std::string& ip) {
            std::cout << "[SIM] Robot connected at " << ip << "\n";
        }

        void setDefaultBehavior() {
            std::cout << "[SIM] Default behavor set\n";
        }

        // Mirrors franka::Robot::automaticErrorRecovery(). Real libfranka
        // requires this to be called after a reflex/fault before control()
        // can be used again. The simulator never actually faults on its
        // own, but this exists so error-handling code written against it
        // (catch -> log -> recover -> stop safely) is exactly what you'd
        // write against the real robot, with nothing to change at the call
        // site when switching over.
        void automaticErrorRecovery() {
            std::cout << "[SIM] automaticErrorRecovery() called\n";
        }

        // Mirror robot.control(), runs callback at 1kHz
        void control(std::function<JointPositions(const RobotState&, Duration)> callback) {
            Duration period;

            while (true) {
                JointPositions cmd = callback(state_, period);

                // Update simulated state, assume robot perfectly tracks commands
                for (int j = 0; j < 7; ++j) {
                    state_.dq[j]  = (cmd.q[j] - state_.q[j]) / period.toSec();
                    state_.ddq_d[j] = (state_.dq[j] - prev_dq_[j]) / period.toSec();
                    prev_dq_[j] = state_.dq[j];
                    state_.q[j] = cmd.q[j];
                    state_.q_d[j] = cmd.q[j];
                }

                if (cmd.motion_finished) break;

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        private:
        RobotState state_;
        std::array<double, 7> prev_dq_ = {0,0,0,0,0,0,0};
    };
}