#include "ik_solver.h"

// --- helper function ---
pinocchio::SE3 buildTarget(const Eigen::Vector3d& pos)
{
    // aggiunge la rotazione verso il basso all'obiettivo
    Eigen::Matrix3d R_yaw_pi;
    R_yaw_pi << -1,  0, 0,
                 0, -1, 0,
                 0,  0, 1;

    return pinocchio::SE3(R_yaw_pi, pos);
}

IKResult ikine_LM(
    const pinocchio::Model&  model,
    pinocchio::Data&         data,
    const pinocchio::SE3&    target,          // desired EE pose
    const pinocchio::FrameIndex frame_id,     // EE frame
    const Eigen::VectorXd&   q0               // start configuration
)
{
    double  tol        = 1e-6;
    int     max_iter   = 1000;
    double  lambda0    = 1e-3;                // initial damping
    const int NQ = model.nq;
    Eigen::VectorXd q = q0;

    double lambda = lambda0;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // ── FK + frame update ──
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacement(model, data, frame_id);

        // ── Pose error in se(3) via log map ──
        const pinocchio::SE3 M_current = data.oMf[frame_id];
        const pinocchio::SE3 M_err     = M_current.inverse() * target; // local error
        const pinocchio::Motion xi     = pinocchio::log6(M_err);       // 6D twist error

        Eigen::Matrix<double,6,1> err_vec = xi.toVector();             // [v; w]

        // ── Convergence check ──
        if (err_vec.norm() < tol) {
            return { q, true, err_vec.norm() };
        }

        // ── Jacobian in local frame ──
        pinocchio::computeFrameJacobian(
            model, data, q, frame_id,
            pinocchio::LOCAL,   // same convention as the log6 error above
            data.J);            // 6 x NQ

        Eigen::MatrixXd J = data.J; // 6 x NQ

        // ── Damped Least Squares (Levenberg-Marquardt) step ──
        // Δq = (JᵀJ + λI)⁻¹ Jᵀ e
        Eigen::MatrixXd JtJ = J.transpose() * J;                       // NQ x NQ
        Eigen::VectorXd dq  = (JtJ + lambda * Eigen::MatrixXd::Identity(NQ, NQ))
                              .ldlt()
                              .solve(J.transpose() * err_vec);

        Eigen::VectorXd q_new = pinocchio::integrate(model, q, dq);

        // ── Clamp to joint limits ──
        q_new = q_new.cwiseMax(model.lowerPositionLimit)
                     .cwiseMin(model.upperPositionLimit);

        // ── LM damping update ──
        pinocchio::forwardKinematics(model, data, q_new);
        pinocchio::updateFramePlacement(model, data, frame_id);
        double new_err = (pinocchio::log6(data.oMf[frame_id].inverse() * target))
                         .toVector().norm();

        if (new_err < err_vec.norm())
            lambda *= 0.1;   // good step → reduce damping (more like Gauss-Newton)
        else
            lambda *= 10.0;  // bad step → increase damping (more like gradient descent)

        q = q_new;
    }

    // Return best effort if not converged
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, frame_id);
    double final_err = (pinocchio::log6(data.oMf[frame_id].inverse() * target))
                       .toVector().norm();

    return { q, false, final_err };
}