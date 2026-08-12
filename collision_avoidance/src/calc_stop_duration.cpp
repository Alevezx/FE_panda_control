#include <pinocchio/autodiff/casadi.hpp>
#include "calc_stop_duration.h"
#include <pinocchio/algorithm/rnea.hpp>

StopDurationOptimizer::StopDurationOptimizer(const pinocchio::Model& model)
    : stop_duration_prev_(UBX)  // mirrors: stop_duration_prec = ubx
{
    // ── Cast Pinocchio model to CasADi scalar type ──────────────────
    cmodel_ = model.cast<casadi::SX>();

    // ── Joint limits (mirrors the ca.DM([...]) in Python) ───────────
    q_min_lim_ = casadi::DM({-2.8973, -1.7628, -2.8973, -3.0718,
                              -2.8973, -0.0175, -2.8973});
    q_max_lim_ = casadi::DM({ 2.8973,  1.7628,  2.8973, -0.0698,
                               2.8973,  3.7525,  2.8973});
    q_p_lim_   = casadi::DM({ 2.1750,  2.1750,  2.1750,  2.1750,
                               2.6100,  2.6100,  2.6100});
    q_pp_lim_  = casadi::DM({15.0,  7.5, 10.0, 12.5, 15.0, 20.0, 20.0});
    q_ppp_lim_ = casadi::DM({7500.0, 3750.0, 5000.0, 6250.0,
                              7500.0, 10000.0, 10000.0});
    tau_lim_   = casadi::DM({87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0});
    tau_p_lim_ = casadi::DM({1000.0, 1000.0, 1000.0, 1000.0,
                              1000.0, 1000.0, 1000.0});

    // ── Build solver once ────────────────────────────────────────────
    solver_ = createSolver();
}

Poly5Traj StopDurationOptimizer::trajPoly5(
    casadi::SX qi,    casadi::SX qi_p,  casadi::SX qi_pp,
    casadi::SX qf,    casadi::SX qf_p,  casadi::SX qf_pp,
    casadi::SX x)
{
    // ── First 3 coefficients are trivial ────────────────────────────
    casadi::SX c0 = qi;
    casadi::SX c1 = qi_p;
    casadi::SX c2 = qi_pp / 2.0;

    // ── Solve for last 3 coefficients ───────────────────────────────
    // Mirrors: known_terms and V matrix in Python
    casadi::SX rhs = casadi::SX::zeros(3, 1);
    rhs(0) = qf    - qi    - qi_p  * x - 0.5 * qi_pp * x*x;
    rhs(1) = qf_p  - qi_p  - qi_pp * x;
    rhs(2) = qf_pp - qi_pp;

    casadi::SX V = casadi::SX::zeros(3, 3);
    V(0,0) = x*x*x;       V(0,1) = x*x*x*x;        V(0,2) = x*x*x*x*x;
    V(1,0) = 3*x*x;       V(1,1) = 4*x*x*x;         V(1,2) = 5*x*x*x*x;
    V(2,0) = 6*x;         V(2,1) = 12*x*x;           V(2,2) = 20*x*x*x;

    casadi::SX c345 = casadi::SX::solve(V, rhs);
    casadi::SX c3 = c345(0);
    casadi::SX c4 = c345(1);
    casadi::SX c5 = c345(2);

    // ── Sample trajectory at N+1 points ─────────────────────────────
    casadi::SX q_out   = casadi::SX::zeros(N+1, 1);
    casadi::SX qp_out  = casadi::SX::zeros(N+1, 1);
    casadi::SX qpp_out = casadi::SX::zeros(N+1, 1);

    for (int i = 0; i <= N; ++i)
    {
        casadi::SX t = i * x / (double)N;

        q_out(i)   = c0 + c1*t + c2*t*t + c3*t*t*t
                        + c4*t*t*t*t + c5*t*t*t*t*t;

        qp_out(i)  = c1 + 2*c2*t + 3*c3*t*t
                        + 4*c4*t*t*t + 5*c5*t*t*t*t;

        qpp_out(i) = 2*c2 + 6*c3*t + 12*c4*t*t + 20*c5*t*t*t;
    }

    return {q_out, qp_out, qpp_out};
}

casadi::SX StopDurationOptimizer::buildConstraints(casadi::SX x, casadi::SX p)
{
    // p is 3x7: p[0,:] = q, p[1,:] = q_p, p[2,:] = q_pp

    // ── 1. Build trajectory for each joint ──────────────────────────
    casadi::SX Q    = casadi::SX::zeros(N+1, 7);
    casadi::SX Q_p  = casadi::SX::zeros(N+1, 7);
    casadi::SX Q_pp = casadi::SX::zeros(N+1, 7);

    for (int i = 0; i < 7; ++i)
    {
        Poly5Traj traj = trajPoly5(
            p(0,i), p(1,i), p(2,i),  // qi, qi_p, qi_pp
            p(0,i), 0.0,    0.0,      // qf=qi, qf_p=0, qf_pp=0
            x);

        Q(casadi::Slice(), i)    = traj.q;
        Q_p(casadi::Slice(), i)  = traj.q_p;
        Q_pp(casadi::Slice(), i) = traj.q_pp;
    }

    // ── 2. Compute inverse dynamics via Pinocchio RNEA ───────────────
    // Build symbolic RNEA function once
    pinocchio::DataTpl<casadi::SX> cdata(cmodel_);

    casadi::SX q_sym   = casadi::SX::sym("q",   7);
    casadi::SX qp_sym  = casadi::SX::sym("qp",  7);
    casadi::SX qpp_sym = casadi::SX::sym("qpp", 7);

    // Gravity vector
    cmodel_.gravity.linear(pinocchio::ForceTpl<casadi::SX>::Vector3(0, 0, -9.81));
    
    // Use the correct Eigen vector type for CasADi scalars
    typedef pinocchio::ModelTpl<casadi::SX>::ConfigVectorType ConfigVector;
    typedef pinocchio::ModelTpl<casadi::SX>::TangentVectorType TangentVector;

    // Symbolic joint variables as proper Eigen vectors
    ConfigVector  q_eig(7);
    TangentVector qp_eig(7);
    TangentVector qpp_eig(7);

    for (int i = 0; i < 7; ++i) {
        q_eig[i]   = q_sym(i);
        qp_eig[i]  = qp_sym(i);
        qpp_eig[i] = qpp_sym(i);
    }

    pinocchio::rnea(cmodel_, cdata, q_eig, qp_eig, qpp_eig);

    // Wrap result back into casadi::SX
    casadi::SX tau_result = casadi::SX::zeros(7);
    for (int i = 0; i < 7; ++i)
        tau_result(i) = cdata.tau[i];

    casadi::Function rnea_fn("rnea",
        {q_sym, qp_sym, qpp_sym},
        {tau_result});

    // ── 3. Evaluate dynamics at each sample point ────────────────────
    casadi::SX Q_ppp = casadi::SX::zeros(N+1, 7);
    casadi::SX TAU   = casadi::SX::zeros(N+1, 7);
    casadi::SX TAU_p = casadi::SX::zeros(N+1, 7);

    // First sample (no finite difference possible yet)
    casadi::SXVector tau0 = rnea_fn(casadi::SXVector{Q(0, casadi::Slice()).T(),
                                     Q_p(0, casadi::Slice()).T(),
                                     Q_pp(0, casadi::Slice()).T()});
    TAU(0, casadi::Slice()) = tau0[0].T();

    for (int i = 1; i <= N; ++i)
    {
        casadi::SX qi_row   = Q(i,   casadi::Slice()).T();
        casadi::SX qpi_row  = Q_p(i,  casadi::Slice()).T();
        casadi::SX qppi_row = Q_pp(i, casadi::Slice()).T();

        // Jerk via finite difference: mirrors Q_ppp[i,:] = (Q_pp[i,:] - Q_pp[i-1,:]) / dt
        casadi::SX dt = x / (double)N;
        Q_ppp(i, casadi::Slice()) = (Q_pp(i,   casadi::Slice())
                                   - Q_pp(i-1, casadi::Slice())) / dt;

        // Torque via RNEA
        casadi::SXVector tau_i = rnea_fn(casadi::SXVector{qi_row, qpi_row, qppi_row});
        TAU(i, casadi::Slice()) = tau_i[0].T();

        // Torque rate via finite difference
        TAU_p(i, casadi::Slice()) = (TAU(i,   casadi::Slice())
                                   - TAU(i-1, casadi::Slice())) / dt;
    }

    // ── 4. Extract max/min over trajectory ──────────────────────────
    casadi::SX q_min   = casadi::SX::zeros(7);
    casadi::SX q_max   = casadi::SX::zeros(7);
    casadi::SX qp_max  = casadi::SX::zeros(7);
    casadi::SX qpp_max = casadi::SX::zeros(7);
    casadi::SX qppp_max= casadi::SX::zeros(7);
    casadi::SX tau_max = casadi::SX::zeros(7);
    casadi::SX taup_max= casadi::SX::zeros(7);

    for (int i = 0; i < 7; ++i)
    {
        casadi::SX col_q    = Q(casadi::Slice(),    i);
        casadi::SX col_qp   = Q_p(casadi::Slice(),  i);
        casadi::SX col_qpp  = Q_pp(casadi::Slice(), i);
        casadi::SX col_qppp = Q_ppp(casadi::Slice(),i);
        casadi::SX col_tau  = TAU(casadi::Slice(),  i);
        casadi::SX col_taup = TAU_p(casadi::Slice(),i);

        q_min(i)    = casadi::SX::mmin(col_q);
        q_max(i)    = casadi::SX::mmax(col_q);
        qp_max(i)   = casadi::SX::mmax(casadi::SX::abs(col_qp));
        qpp_max(i)  = casadi::SX::mmax(casadi::SX::abs(col_qpp));
        qppp_max(i) = casadi::SX::mmax(casadi::SX::abs(col_qppp));
        tau_max(i)  = casadi::SX::mmax(casadi::SX::abs(col_tau));
        taup_max(i) = casadi::SX::mmax(casadi::SX::abs(col_taup));
    }

    // ── 5. Build constraint vector (all must be <= 0) ────────────────
    // Mirrors: c01, c02, c1, c2, c3, c4, c5 in Python
    casadi::SX c01 = q_min_lim_ * TOLL - q_min;   // q >= q_min_lim * toll
    casadi::SX c02 = q_max      - q_max_lim_ * TOLL; // q <= q_max_lim * toll
    casadi::SX c1  = qp_max     - q_p_lim_   * TOLL;
    casadi::SX c2  = qpp_max    - q_pp_lim_  * TOLL;
    casadi::SX c3  = qppp_max   - q_ppp_lim_ * TOLL;
    casadi::SX c4  = tau_max    - tau_lim_   * TOLL;
    casadi::SX c5  = taup_max   - tau_p_lim_ * TOLL;

    return casadi::SX::vertcat({c01, c02, c1, c2, c3, c4, c5});
}

casadi::Function StopDurationOptimizer::createSolver()
{
    // Optimization variable: stop duration
    casadi::SX x = casadi::SX::sym("x");

    // Parameters: 3x7 matrix [q; q_p; q_pp]
    casadi::SX p = casadi::SX::sym("p", 3, 7);

    // Cost and constraints
    casadi::SX cost = x * P1 + casadi::SX::abs(x - stop_duration_prev_) * P2;
    casadi::SX constraints = buildConstraints(x, p);

    // NLP
    casadi::SXDict nlp = {{"x", x}, {"p", p}, {"f", cost}, {"g", constraints}};

    casadi::Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["ipopt.sb"]          = "yes";
    opts["ipopt.max_iter"]    = 10;
    opts["print_time"]        = 0;

    return casadi::nlpsol("solver", "ipopt", nlp, opts);
}

double StopDurationOptimizer::solve(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_p,
    const Eigen::VectorXd& q_pp)
{
    // Pack [q; q_p; q_pp] into a 3x7 DM matrix
    // mirrors: p = ca.DM([req.q, req.q_p, req.q_pp])
    std::vector<double> p_vec;
    for (int i = 0; i < 7; ++i) p_vec.push_back(q[i]);
    for (int i = 0; i < 7; ++i) p_vec.push_back(q_p[i]);
    for (int i = 0; i < 7; ++i) p_vec.push_back(q_pp[i]);

    casadi::DM p_dm = casadi::DM::reshape(casadi::DM(p_vec), 3, 7);

    // Number of constraints: 7 per constraint type, 7 types = 49
    int n_g = 7 * 7;
    
    casadi::DMDict args = {
        {"x0",  casadi::DM(stop_duration_prev_)},  // warm start
        {"p",   p_dm},
        {"lbx", casadi::DM(LBX)},
        {"ubx", casadi::DM(UBX)},
        {"lbg", -casadi::DM::inf(n_g)},            // no lower bound on constraints
        {"ubg", casadi::DM::zeros(n_g)}             // all constraints <= 0
    };

    casadi::DMDict result = solver_(args);
    //std::cout << "Active constraints g: " << result.at("g") << "\n";

    double stop_duration = static_cast<double>(result.at("x"));
    stop_duration_prev_  = stop_duration;  // update warm start

    return stop_duration;
}