#include <iostream>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include "calc_stop_duration.h"

int main() {
    // ── 1. Load model ─────────────────────────────────────────────────
    pinocchio::Model model;
    pinocchio::urdf::buildModel("../urdf/panda.urdf", model);
    std::cout << "Model loaded, nq = " << model.nq << "\n";

    // ── 2. Create optimizer ───────────────────────────────────────────
    StopDurationOptimizer optimizer(model);
    std::cout << "Optimizer created\n";

    // ── 3. Test case 1: robot at rest ─────────────────────────────────
    // If the robot is already still, stop duration should be near LBX (0.01)
    Eigen::VectorXd q(7),  q_p(7), q_pp(7);
    q   << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;  // home position
    q_p  = Eigen::VectorXd::Zero(7);
    q_pp = Eigen::VectorXd::Zero(7);

    double t_stop = optimizer.solve(q, q_p, q_pp);
    std::cout << "Test 1 (at rest):       t_stop = " << t_stop << "s"
              << " (expected: near 0.01)\n";

    // ── 4. Test case 2: robot moving fast ─────────────────────────────
    // High velocity should require a longer stop duration
    q_p << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    q_pp = Eigen::VectorXd::Zero(7);

    t_stop = optimizer.solve(q, q_p, q_pp);
    std::cout << "Test 2 (moving fast):   t_stop = " << t_stop << "s"
              << " (expected: > test 1)\n";

    // ── 5. Test case 3: robot at velocity limit ────────────────────────
    // At max velocity the stop duration should be near UBX (0.4)
    q_p << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

    t_stop = optimizer.solve(q, q_p, q_pp);
    std::cout << "Test 3 (at vel limit):  t_stop = " << t_stop << "s"
              << " (expected: near 0.4)\n";
    
    // ── 6. Test warm start ────────────────────────────────────────────
    // Calling solve() multiple times should converge faster each time
    // (you can measure this with a timer)
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10; ++i)
        optimizer.solve(q, q_p, q_pp);
    auto t1 = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "10 solve() calls took " << ms << "ms"
              << " (" << ms/10.0 << "ms avg)\n";
    std::cout << "(must be well under 10ms avg for real-time use)\n";

    return 0;
}