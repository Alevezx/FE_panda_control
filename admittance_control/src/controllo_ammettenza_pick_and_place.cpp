#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include <map> // Aggiunto per gestire i waypoint
#include <iomanip>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <chrono>

#include "skeleton_reader.h"
#include "examples_common.h"
#include "header_capsuleStatiche.h"
#include "shared_state.h"


// Funzione di interpolazione sigmoidale per transizioni morbide (range [0, 1])
static double smoothstep(double edge0, double edge1, double x) {
  if (std::abs(edge1 - edge0) < 1e-12) return (x <= edge0) ? 1.0 : 0.0;
  double t = (x - edge0) / (edge1 - edge0);
  t = std::max(0.0, std::min(t, 1.0));
  return t * t * (3.0 - 2.0 * t);
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>\n";
    return -1;
  }

  std::thread skeleton_thread(receiveSkeletonMerged, "localhost", 10, "MERGED",
                                 "../logs/skeleton_received.txt");

  // ========================================================================
  // CONFIGURAZIONE TRAIETTORIA E WAYPOINT
  // ========================================================================
  
  // Definizione dei waypoint cartesiani (coordinate [x, y, z] in metri)
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.push_back(Eigen::Vector3d(0.3, 0.5, 0.5));  // Index 0: A (Home/Start) (0.3, 0.5, 0.5)
  waypoints.push_back(Eigen::Vector3d(0.3, 0.5, 0.2));  // Index 1: B (0.3, 0.5, 0.2)
  waypoints.push_back(Eigen::Vector3d(0.5, 0.4, 0.5));  // Index 2: C (0.5, 0.0, 0.5)
  waypoints.push_back(Eigen::Vector3d(0.5, 0.4, 0.2));  // Index 3: D (0.5, 0.0, 0.2)
  waypoints.push_back(Eigen::Vector3d(0.5, -0.3, 0.5)); // Index 4: E (0.3, -0.5, 0.5)
  waypoints.push_back(Eigen::Vector3d(0.5, -0.3, 0.2)); // Index 5: F (0.3, -0.5, 0.2)
  waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.4));  // Index 6: G (Transizione sicura) (0.5, 0.0, 0.4)

  // Definizione della sequenza di movimento (Pattern: A->B->A->C->D->C->E->F->E)
  //std::vector<int> pattern = {5, 4, 2, 3, 2, 4};
  std::vector<int> pattern = {4, 2, 3, 2, 4, 5};

  std::vector<int> fullSequence;
  int num_repetitions = 2; // Numero di volte che voglio ripetere il pattern
  
  for(int k=0; k < num_repetitions; ++k) {
      for(int idx : pattern) {
          fullSequence.push_back(idx);
      }
  }

  // Parametri temporali della traiettoria
  const double segmentDuration = 3.0; // Durata per ogni singolo segmento (es. A->B)
  int currentSequenceIndex = 0;
  double segmentTime = 0.0;           // Tempo locale del segmento corrente

  // ========================================================================
  // PARAMETRI DEL SISTEMA DI CONTROLLO E SICUREZZA
  // ========================================================================

  // Definizione raggi capsule robot (approssimazione fisica dei link)
  std::vector<double> robot_capsule_radius = {0.125, 0.125, 0.125, 0.125};

  // Parametri di ammettenza e campo potenziale
  const double kStiffness = 350.0;       
  const double kRepulsiveStiffness = 200.0; 

  // Parametri per la procedura di Recovery
  const double kRecoveryMaxVel = 0.25;     // [m/s] Limite ISO 10218-1 (Reduced Speed)  
  const double kQuinticPeakFactor = 1.875; // Fattore di picco per polinomi quintici (15/8)

  // Parametri delle zone di sicurezza dinamica
  SafetyParams safetyParams;
  safetyParams.r_inner = 0.0;  // Soglia di arresto immediato
  safetyParams.r_outer = 0.30; // Raggio di influenza repulsiva
  safetyParams.T_stop = 10.0;   // Tempo di arresto controllato (basato su test empirici)

  // Parametri di filtraggio forze
  const double forceFilterGain = 0.15;   //0.05;
  const double softmin_beta = 15.0; //40.0; 

  // Variabili di stato per la generazione traiettorie (polinomi quintici)
  Eigen::MatrixXd coeffsX(6, 1), coeffsY(6, 1), coeffsZ(6, 1);
  coeffsX.setZero(); coeffsY.setZero(); coeffsZ.setZero();

  // Buffer per il logging dei dati
  std::vector<franka::RobotState> log_data;
  log_data.reserve(200000); // Aumentato a 200s per coprire l'intera sequenza (~80s + margini)

  // Variabili di stato del controllo di ammettenza
  Eigen::Vector3d filteredRepulsiveForce = Eigen::Vector3d::Zero();
  Eigen::Vector3d admittanceOffset = Eigen::Vector3d::Zero();
  Eigen::Vector3d externalForce = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 1> nextTrajPosition;
  nextTrajPosition.setZero();
  Eigen::VectorXd stateX(6); stateX.setZero();

  // Matrici dello spazio di stato per l'ammettenza
  Eigen::MatrixXd A(6, 6), Ad(6, 6), B(6, 3), Bd(6, 3), C(3, 6), Cd(3, 6), D(3, 3), Dd(3, 3);
  A.setZero(); B.setZero(); C.setZero(); D.setZero();
  Ad.setZero(); Bd.setZero(); Cd.setZero(); Dd.setZero();

  Eigen::MatrixXd stiffnessMat(3, 3), dampingMat(3, 3), massMat(3, 3);
  stiffnessMat = kStiffness * Eigen::MatrixXd::Identity(3, 3);

  // Gestione Macchina a Stati (Safety State Machine)
  enum State { RUNNING, STOPPING, PAUSED, RECOVERING, FINISHED };
  State currentState = RUNNING;

  // Variabili per la gestione dello STOP
  double stopTimer = 0.0;
  Eigen::Vector3d stopStartPos = Eigen::Vector3d::Zero();
  Eigen::MatrixXd stopCoeffsX(6, 1), stopCoeffsY(6, 1), stopCoeffsZ(6, 1);

  // Variabili per la gestione del RECOVERY
  double recoveryTimer = 0.0;
  double recoveryDuration = 3.0; // Valore di default, verrà ricalcolato dinamicamente
  double frozenTime = 0.0;       // Istante temporale congelato al momento dello stop
  Eigen::MatrixXd recCoeffsX(6, 1), recCoeffsY(6, 1), recCoeffsZ(6, 1);

  // Variabili globali di ciclo
  double globalTime = 0.0;
  bool firstRun = true;
  Eigen::Vector3d startPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d endPos = Eigen::Vector3d::Zero(); 

  // Variabili per la gestione della forza repulsiva laterale
  Eigen::Vector3d prevPerpDir = Eigen::Vector3d::UnitY();
  bool prevPerpDirSet = false;
  Eigen::Vector3d lastDesiredPos = Eigen::Vector3d::Zero();
/*
  // Inizializzazione Subscriber ZMQ per ricezione dati scheletro
  SkeletonZmqSubscriber skelSub("ipc:///tmp/skeleton.ipc");
  skelSub.start();
  auto skel_t0 = std::chrono::steady_clock::now();
*/
  try {
    // Connessione al robot e configurazione parametri base
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    robot.setLoad(0.0, {0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0});

    // ========================================================================
    // FASE DI HOMING: MOVIMENTO AL PUNTO DI PARTENZA (A)
    // ========================================================================
    // Si utilizza il primo punto della lista waypoints (Index 0 = A) come start
    Eigen::Vector3d startPos_desired = waypoints[5]; // Prendo E come punto di partenza per test più rapido

    Eigen::Matrix3d startRot_desired;
    startRot_desired << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;
    Eigen::Quaterniond startOri_desired(startRot_desired);

    std::cout << "Spostamento al punto A (Home) con orientamento fissato..." << std::endl;
    franka::RobotState initial_state = robot.readOnce();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE_d.data()));
    Eigen::Vector3d p0 = initial_transform.translation();
    Eigen::Quaterniond q0(initial_transform.rotation());

    double dist = (startPos_desired - p0).norm();
    double angle_dist = q0.angularDistance(startOri_desired);

    if (dist > 0.01 || angle_dist > 0.01) {
      double duration = std::max(2.0, std::max(dist / 0.1, angle_dist / 0.5)); 
      std::vector<double> p0_x = {p0.x(), 0.0, 0.0}; std::vector<double> pf_x = {startPos_desired.x(), 0.0, 0.0};
      std::vector<double> p0_y = {p0.y(), 0.0, 0.0}; std::vector<double> pf_y = {startPos_desired.y(), 0.0, 0.0};
      std::vector<double> p0_z = {p0.z(), 0.0, 0.0}; std::vector<double> pf_z = {startPos_desired.z(), 0.0, 0.0};

      Eigen::MatrixXd cx = generateQuinticCoefficients(0.0, duration, p0_x, pf_x);
      Eigen::MatrixXd cy = generateQuinticCoefficients(0.0, duration, p0_y, pf_y);
      Eigen::MatrixXd cz = generateQuinticCoefficients(0.0, duration, p0_z, pf_z);

      double time = 0.0;
      robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();
        if (time > duration) {
             Eigen::Affine3d final_transform(startOri_desired);
             final_transform.translation() = startPos_desired;
             std::array<double, 16> final_pose;
             Eigen::Map<Eigen::Matrix4d>(final_pose.data()) = final_transform.matrix();
             return franka::MotionFinished(final_pose);
        }
        Eigen::Vector3d pd;
        trajectory_updater(pd, time, 0.0, cx, cy, cz);
        double alpha = smoothstep(0.0, duration, time);
        Eigen::Quaterniond q_curr = q0.slerp(alpha, startOri_desired);
        Eigen::Affine3d new_transform(q_curr);
        new_transform.translation() = pd;
        std::array<double, 16> new_pose;
        Eigen::Map<Eigen::Matrix4d>(new_pose.data()) = new_transform.matrix();
        return new_pose;
      });      
    }
    std::cout << "Punto A raggiunto. Avvio sequenza pick & place..." << std::endl;

    franka::Model model = robot.loadModel();

    // ========================================================================
    // LOOP DI CONTROLLO REAL-TIME (1 kHz)
    // ========================================================================
    robot.control([&](const franka::RobotState& state,
                      franka::Duration duration) -> franka::CartesianPose {
      
      if (log_data.size() < log_data.capacity()) log_data.push_back(state);

      double dt = duration.toSec();
      globalTime += dt;

      // ----------------------------------------------------------------------
      // 1. Inizializzazione del primo segmento (solo al primo ciclo)
      if (firstRun) {
        Eigen::Map<const Eigen::Matrix<double, 4, 4>> O_T_EE_start(state.O_T_EE_d.data());
        startPos = O_T_EE_start.block<3, 1>(0, 3);
        
        // Imposta il primo target della sequenza
        if (fullSequence.empty()) {
            currentState = FINISHED; // Caso sequenza vuota
        } else {
            endPos = waypoints[fullSequence[0]]; // Primo target
            
            std::vector<double> pos_0x = {startPos[0], 0.0, 0.0}; std::vector<double> pos_fx = {endPos[0], 0.0, 0.0};
            std::vector<double> pos_0y = {startPos[1], 0.0, 0.0}; std::vector<double> pos_fy = {endPos[1], 0.0, 0.0};
            std::vector<double> pos_0z = {startPos[2], 0.0, 0.0}; std::vector<double> pos_fz = {endPos[2], 0.0, 0.0};

            coeffsX = generateQuinticCoefficients(0.0, segmentDuration, pos_0x, pos_fx);
            coeffsY = generateQuinticCoefficients(0.0, segmentDuration, pos_0y, pos_fy);
            coeffsZ = generateQuinticCoefficients(0.0, segmentDuration, pos_0z, pos_fz);
        }

        nextTrajPosition = startPos;
        lastDesiredPos = startPos;
        segmentTime = 0.0;
        currentSequenceIndex = 0;
        firstRun = false;
      }

      // ----------------------------------------------------------------------
      // 2. Acquisizione dati dinamici del robot (Massa, Jacobiano, Velocità)
      std::array<double, 49> massArray = model.mass(state);
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(massArray.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
      Eigen::VectorXd dq_eigen(7);
      for (int i = 0; i < 7; i++) dq_eigen(i) = state.dq[i];
      Eigen::Vector3d currentVelocity = (J * dq_eigen).head(3);

      // ----------------------------------------------------------------------
      // 3. Aggiornamento Capsule Robot (Cinematica diretta dei giunti)
      std::array<Eigen::Vector3d, 8> frames;
      frames[0] = Eigen::Vector3d::Zero();
      auto get_pos = [&](franka::Frame f) {
        std::array<double, 16> p = model.pose(f, state);
        return Eigen::Vector3d(p[12], p[13], p[14]);
      };
      frames[1] = get_pos(franka::Frame::kJoint1); frames[2] = get_pos(franka::Frame::kJoint2);
      frames[3] = get_pos(franka::Frame::kJoint3); frames[4] = get_pos(franka::Frame::kJoint4);
      frames[5] = get_pos(franka::Frame::kJoint5); frames[6] = get_pos(franka::Frame::kJoint6);
      frames[7] = get_pos(franka::Frame::kFlange);

      /*// MODIFICA: Riduzione capsula end-effector di 5cm (0.05m) per assenza gripper
      // Calcoliamo il vettore dal giunto 6 alla flangia e lo accorciamo
      Eigen::Vector3d p_joint6 = frames[6];
      Eigen::Vector3d p_flange = frames[7];
      Eigen::Vector3d v_link7 = p_flange - p_joint6;
      double len_link7 = v_link7.norm();
      // Se il link è abbastanza lungo, arretriamo il punto finale di 5cm verso il giunto 6
      Eigen::Vector3d p_flange_reduced = (len_link7 > 0.05) ? (p_joint6 + v_link7.normalized() * (len_link7 - 0.05)) : p_flange;
      // Fine MODIFICA (da commentare se voglio mantenere la capsula completa)*/
      

      std::array<CapsuleGeo, 4> robotCapsules;
      robotCapsules[0] = {frames[1], frames[0], robot_capsule_radius[0]};
      robotCapsules[1] = {frames[3], frames[2], robot_capsule_radius[1]};
      robotCapsules[2] = {frames[5], frames[4], robot_capsule_radius[2]};
      robotCapsules[3] = {frames[7], frames[6], robot_capsule_radius[3]}; // se voglio mantenere la capsula completa
      // robotCapsules[3] = {p_flange_reduced, frames[6], robot_capsule_radius[3]};

      // ----------------------------------------------------------------------
      // 4. Logica di Sicurezza: Calcolo distanze minime Robot-Scheletro
      double minSurfaceDist = 1e9;
      int min_robot_capsule_idx = -1;
      int min_skel_capsule_idx = -1;

      std::array<double, 4> distSurf_i; distSurf_i.fill(1e9);
      std::array<Eigen::Vector3d, 4> dir_i; dir_i.fill(Eigen::Vector3d::Zero());
      std::array<Eigen::Vector3d, 4> cp_obs_i; cp_obs_i.fill(Eigen::Vector3d::Zero());
/*
      // Lettura non bloccante dal buffer ZMQ
      SkeletonCapsuleBuffer skel;
      skelSub.readLatest(skel); 
*/
      const uint64_t now_ns = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now() - skel_t0).count();
      const double max_age_s = 0.10;
      const bool skeleton_valid = (skel.n_caps > 0) && (skel.rx_time_ns > 0) /*&&
                                  ((now_ns >= skel.rx_time_ns) ? ((now_ns - skel.rx_time_ns) * 1e-9 < max_age_s) : true)*/;

      //std::cout << "Skeleton valid: " << skeleton_valid << "\n" << skel.n_caps << ", " << skel.rx_time_ns << ", " << now_ns << ", " << max_age_s << "\n";
      if (!skeleton_valid) {
        minSurfaceDist = 1e9;
      } else {
        for (int i = 0; i < 4; ++i) {
          double best = 1e9;
          Eigen::Vector3d best_cp2 = Eigen::Vector3d::Zero();
          Eigen::Vector3d best_dir = Eigen::Vector3d::Zero();

          for (int j = 0; j < (int)skel.n_caps; ++j) {
            const CapsuleGeo& sc = skel.caps[j];
            DistanceResult res = distance_to_segment(robotCapsules[i].p_start, robotCapsules[i].p_end,
                                                     sc.p_start, sc.p_end);
            const double dSurf = res.distance - sc.radius - robotCapsules[i].radius;

            if (dSurf < minSurfaceDist) {
              minSurfaceDist = dSurf;
              min_robot_capsule_idx = i;
              min_skel_capsule_idx = j;
            }
            if (dSurf < best) {
              best = dSurf;
              best_cp2 = res.closest_point_2;
              Eigen::Vector3d dvec = res.closest_point_1 - res.closest_point_2;
              const double dnorm = std::max(dvec.norm(), 1e-6);
              best_dir = dvec / dnorm;
            }
          }
          distSurf_i[i] = best;
          cp_obs_i[i] = best_cp2;
          dir_i[i] = best_dir;
        }
      }

      // ----------------------------------------------------------------------
      // 5. Macchina a Stati: Gestione Comportamento (Running, Stop, Recovery)
      if (currentState == RUNNING) {
        // A) Controllo condizione di STOP (distanza < r_inner)
        std::cout << "Distanza: " << minSurfaceDist << "\n";
        if (minSurfaceDist <= safetyParams.r_inner) {
          std::cout << "!!! STOP ATTIVATO (min surf dist: " << minSurfaceDist << ") !!!\n";
          currentState = STOPPING;
          stopTimer = 0.0;
          frozenTime = segmentTime; // Congela il tempo LOCALE del segmento

          stopStartPos = lastDesiredPos;
          Eigen::Vector3d stopEndPos = stopStartPos + currentVelocity * safetyParams.T_stop * 0.5;
          //Eigen::Vector3d stopEndPos = stopStartPos; // per vedere se il profilo di stop è più fluido senza movimento (forse con velocità iniziale bassa è già abbastanza fluido senza muovere il target)

          stopCoeffsX = compute_traj_poly5_coeffs(stopStartPos[0], currentVelocity[0], 0.0, stopEndPos[0], 0.0, 0.0, safetyParams.T_stop);
          stopCoeffsY = compute_traj_poly5_coeffs(stopStartPos[1], currentVelocity[1], 0.0, stopEndPos[1], 0.0, 0.0, safetyParams.T_stop);
          stopCoeffsZ = compute_traj_poly5_coeffs(stopStartPos[2], currentVelocity[2], 0.0, stopEndPos[2], 0.0, 0.0, safetyParams.T_stop);
        }
        // B) Calcolo Forza Repulsiva (distanza < r_outer)
        else if (minSurfaceDist < safetyParams.r_outer) {
          std::cout << "-- calcolo forza repulsiva --\n";
          double sum_w = 0.0;
          std::array<double, 4> w; w.fill(0.0);
          for (int i = 0; i < 4; i++) {
            if (distSurf_i[i] < safetyParams.r_outer) {
              w[i] = std::exp(-softmin_beta * distSurf_i[i]);
              sum_w += w[i];
            }
          }

          if (sum_w < 1e-12) {
            filteredRepulsiveForce.setZero();
          } else {
            for (int i = 0; i < 4; i++) w[i] /= sum_w;

            Eigen::Vector3d d_hat = Eigen::Vector3d::Zero();
            Eigen::Vector3d obs_point_eff = Eigen::Vector3d::Zero();

            for (int i = 0; i < 4; i++) {
              d_hat += w[i] * dir_i[i];
              obs_point_eff += w[i] * cp_obs_i[i];
            }
            double dnorm = std::max(d_hat.norm(), 1e-6);
            d_hat /= dnorm;

            // MODIFICA: Profilo quadratico per attacco più morbido (C1 continuo)
            double penetration = safetyParams.r_outer - minSurfaceDist;
            // Profilo lineare standard (da mantenere se voglio una repulsione più decisa vicino al bordo esterno)
            //Eigen::Vector3d F_std = kRepulsiveStiffness * penetration * d_hat;
            // Profilo quadratico più morbido vicino al bordo esterno (C1 continuo)
             Eigen::Vector3d F_std = kRepulsiveStiffness * (penetration * penetration / safetyParams.r_outer) * d_hat;
            
            // Calcolo componente laterale per aggiramento ostacolo
            Eigen::Vector3d t_hat = Eigen::Vector3d::Zero();
            Eigen::Vector3d t_vec = (endPos - startPos); // Direzione del segmento attuale
            if (t_vec.norm() > 1e-9) t_hat = t_vec.normalized();

            Eigen::Vector3d d_perp = d_hat - (d_hat.dot(t_hat)) * t_hat;
            if (d_perp.norm() > 1e-6) {
              prevPerpDir = d_perp.normalized();
              prevPerpDirSet = true;
            }

            Eigen::Vector3d n_hat = Eigen::Vector3d::Zero();
            if (prevPerpDirSet) {
              n_hat = prevPerpDir;
            } else {
              n_hat = t_hat.cross(Eigen::Vector3d::UnitZ());
              if (n_hat.norm() < 1e-2) n_hat = t_hat.cross(Eigen::Vector3d::UnitY());
              if (n_hat.norm() > 1e-9) n_hat.normalize();
            }
            // Profilo lineare standard (da mantenere se voglio una repulsione più decisa vicino al bordo esterno)
            //Eigen::Vector3d F_lat = kRepulsiveStiffness * penetration * n_hat;
            // Profilo quadratico più morbido vicino al bordo esterno (C1 continuo)
             Eigen::Vector3d F_lat = kRepulsiveStiffness * (penetration * penetration / safetyParams.r_outer) * n_hat;


            double col = std::abs(d_hat.dot(t_hat));
            double obstacleToLineDist = 1e9;
            if (t_hat.squaredNorm() > 0.0) {
              Eigen::Vector3d rp_rel = obs_point_eff - startPos;
              Eigen::Vector3d rp_perp = rp_rel - (rp_rel.dot(t_hat)) * t_hat;
              obstacleToLineDist = rp_perp.norm();
            }

            double obs_singularity_factor = 1.0 - smoothstep(0.0, 0.15, obstacleToLineDist);
            double alpha_col = smoothstep(0.93, 1.0, col);
            double r_th = 0.8 * safetyParams.r_outer;
            double alpha_r = 1.0 - smoothstep(r_th, safetyParams.r_outer, minSurfaceDist);
            double geometry_mix = std::max(alpha_col, obs_singularity_factor);
            double alpha = std::max(0.0, std::min(geometry_mix * alpha_r, 1.0));

            Eigen::Vector3d targetRepulsiveForce = (1.0 - alpha) * F_std + alpha * F_lat;
            //const double maxRepulsiveForce = 20.0;
            //if (targetRepulsiveForce.norm() > maxRepulsiveForce) targetRepulsiveForce = targetRepulsiveForce.normalize() * maxRepulsiveForce;
            std::cout << "Repulsive force: " << targetRepulsiveForce << "\n";
            filteredRepulsiveForce = (1.0 - forceFilterGain) * filteredRepulsiveForce + forceFilterGain * targetRepulsiveForce;
          }
        } else {
          // Nessun ostacolo vicino: decadimento naturale della forza repulsiva
          // (Il target è zero, ma si lascia che il filtro converga gradualmente)
          filteredRepulsiveForce = (1.0 - forceFilterGain) * filteredRepulsiveForce; 
        }
      } 
      // Gestione stati di eccezione (STOPPING, PAUSED, RECOVERING)
      else if (currentState == STOPPING) {
          filteredRepulsiveForce.setZero();
          if (stopTimer >= safetyParams.T_stop) currentState = PAUSED;
      } else if (currentState == PAUSED) {
          filteredRepulsiveForce.setZero();
          if (minSurfaceDist > safetyParams.r_inner + 0.05) {
              currentState = RECOVERING;
              recoveryTimer = 0.0;
              
              // Calcolo stato target sulla traiettoria al momento del freeze
              auto get_traj_state = [&](double t, const Eigen::MatrixXd& c) -> std::vector<double> {
                  double t2 = t*t, t3 = t2*t, t4 = t3*t;
                  double p = c(0) + c(1)*t + c(2)*t2 + c(3)*t3 + c(4)*t4 + c(5)*t3*t2;
                  double v = c(1) + 2*c(2)*t + 3*c(3)*t2 + 4*c(4)*t3 + 5*c(5)*t4;
                  double a = 2*c(2) + 6*c(3)*t + 12*c(4)*t2 + 20*c(5)*t3;
                  return {p, v, a};
              };
              auto tx = get_traj_state(frozenTime, coeffsX);
              auto ty = get_traj_state(frozenTime, coeffsY);
              auto tz = get_traj_state(frozenTime, coeffsZ);

              // Calcolo durata recovery dinamica (v_max = 250 mm/s)
              Eigen::Vector3d p_curr = nextTrajPosition; // Posizione dove ci siamo fermati
              Eigen::Vector3d p_tgt(tx[0], ty[0], tz[0]); // Posizione target sulla traiettoria
              double dist_rec = (p_tgt - p_curr).norm();
              
              // Calcolo anche la differenza di velocità per evitare accelerazioni infinite se dist_rec è piccola
              Eigen::Vector3d v_tgt(tx[1], ty[1], tz[1]);
              double v_diff = v_tgt.norm();
              
              double t_dist = (kQuinticPeakFactor * dist_rec) / kRecoveryMaxVel;              

              //recoveryDuration Minimo 0.5s per stabilità
              recoveryDuration = std::max(0.5, t_dist); // Per semplicità, basiamo solo sulla distanza. In pratica con i profili quintici e v_max ragionevole, anche con piccoli v_diff non dovremmo avere problemi di accelerazioni eccessive.
              std::cout << "Recovery: Dist=" << dist_rec << "m, T=" << recoveryDuration << "s\n";

              recCoeffsX = compute_traj_poly5_coeffs(nextTrajPosition[0], 0.0, 0.0, tx[0], tx[1], tx[2], recoveryDuration);
              recCoeffsY = compute_traj_poly5_coeffs(nextTrajPosition[1], 0.0, 0.0, ty[0], ty[1], ty[2], recoveryDuration);
              recCoeffsZ = compute_traj_poly5_coeffs(nextTrajPosition[2], 0.0, 0.0, tz[0], tz[1], tz[2], recoveryDuration);
          }
      } else if (currentState == RECOVERING) {
          filteredRepulsiveForce.setZero();
          if (recoveryTimer >= recoveryDuration) {
              currentState = RUNNING;
              // Ripristino il tempo LOCALE del segmento
              // Compensa l'overshoot e prepara per l'incremento (+dt) che avverrà nel blocco RUNNING successivo
              segmentTime = frozenTime + (recoveryTimer - recoveryDuration); 
          }
      }
      
      // ----------------------------------------------------------------------
      // 6. Aggiornamento Traiettoria (Nominale o di Eccezione)
      if (currentState != RUNNING && currentState != FINISHED) {
        admittanceOffset.setZero();
        externalForce.setZero();
        stateX.setZero();

        //stateX *= 0.95;
        //admittanceOffset = Cd * stateX;

        if (currentState == STOPPING) {
          stopTimer += dt;
          trajectory_updater(nextTrajPosition, stopTimer, 0.0, stopCoeffsX, stopCoeffsY, stopCoeffsZ);
        } else if (currentState == RECOVERING) {
          recoveryTimer += dt;
          trajectory_updater(nextTrajPosition, recoveryTimer, 0.0, recCoeffsX, recCoeffsY, recCoeffsZ);
        }
      } else if (currentState == RUNNING) {
        // Esecuzione normale
        externalForce = filteredRepulsiveForce;
        
        // Avanzamento tempo locale
        segmentTime += dt;

        if (segmentTime <= segmentDuration) {
           trajectory_updater(nextTrajPosition, segmentTime, 0.0, coeffsX, coeffsY, coeffsZ);
        } else {
           // Cambio segmento: Target raggiunto
           // Assicurati che nextTrajPosition sia esattamente endPos
           nextTrajPosition = endPos; 
           
           // Avanza indice sequenza
           currentSequenceIndex++;
           
           if (currentSequenceIndex >= fullSequence.size()) {
               currentState = FINISHED;
               std::cout << "Intera sequenza Pick & Place completata!\n";
           } else {
               // Prepara il prossimo segmento
               startPos = endPos; // Il vecchio end diventa start
               endPos = waypoints[fullSequence[currentSequenceIndex]]; // Nuovo target
               
               std::cout << "Target raggiunto. Inizio segmento " << currentSequenceIndex + 1 
                         << " / " << fullSequence.size() << " verso waypoint " << fullSequence[currentSequenceIndex] << "\n";

               // Ricalcolo coefficienti (partendo da v=0 a v=0)
               std::vector<double> p0_x = {startPos[0], 0.0, 0.0}; std::vector<double> pf_x = {endPos[0], 0.0, 0.0};
               std::vector<double> p0_y = {startPos[1], 0.0, 0.0}; std::vector<double> pf_y = {endPos[1], 0.0, 0.0};
               std::vector<double> p0_z = {startPos[2], 0.0, 0.0}; std::vector<double> pf_z = {endPos[2], 0.0, 0.0};

               coeffsX = generateQuinticCoefficients(0.0, segmentDuration, p0_x, pf_x);
               coeffsY = generateQuinticCoefficients(0.0, segmentDuration, p0_y, pf_y);
               coeffsZ = generateQuinticCoefficients(0.0, segmentDuration, p0_z, pf_z);
               
               segmentTime = 0.0; // Reset tempo locale
           }
        }

        // --------------------------------------------------------------------
        // 7. Controllo di Ammettenza (Integrazione dinamica)
        int steps = (int)(dt / 0.001); if (steps < 1) steps = 1;
        for (int i = 0; i < steps; i++) {
          Eigen::MatrixXd J_inv_M_inv_Jt = (J * M.inverse() * J.transpose());
          massMat = J_inv_M_inv_Jt.inverse().topLeftCorner(3, 3);
          dampingMat = 0.98 * 2.0 * (stiffnessMat.array() * massMat.array()).cwiseSqrt();

          A.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
          A.bottomLeftCorner(3, 3) = -massMat.inverse() * stiffnessMat;
          A.bottomRightCorner(3, 3) = -massMat.inverse() * dampingMat;
          B.bottomRows(3) = massMat.inverse();
          C.leftCols(3) = Eigen::MatrixXd::Identity(3, 3);

          double Ts = 0.001;
          Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
          Eigen::MatrixXd term_inv = (I - (Ts / 2.0) * A).inverse();
          Ad = term_inv * (I + (Ts / 2.0) * A);
          Bd = term_inv * B * Ts;
          Cd = C * (I + (Ts / 2.0) * A) * term_inv;
          Dd = D + C * term_inv * B * (Ts / 2.0);

          admittanceOffset = Cd * stateX + Dd * externalForce;
          stateX = Ad * stateX + Bd * externalForce;
        }
      }

      // Calcolo posa finale desiderata
      std::array<double, 16> desiredPose = state.O_T_EE_d;
      desiredPose[12] = nextTrajPosition[0] + admittanceOffset[0];
      desiredPose[13] = nextTrajPosition[1] + admittanceOffset[1];
      desiredPose[14] = nextTrajPosition[2] + admittanceOffset[2];
      lastDesiredPos = Eigen::Vector3d(desiredPose[12], desiredPose[13], desiredPose[14]);

      // Logging periodico su console
      if ((int)(globalTime * 1000) % 500 == 0 && currentState == RUNNING) {
          std::cout << "Seq: " << currentSequenceIndex << "/" << fullSequence.size() 
                    << " | t_seg: " << std::fixed << std::setprecision(2) << segmentTime
                    << " | t_tot: " << globalTime
                    << " | F_rep: " << filteredRepulsiveForce.norm() << "\n";
      }

      if (currentState == FINISHED) {
        return franka::MotionFinished(desiredPose);
      }

      return desiredPose;
    });
/*
    skelSub.stop();
*/
  } catch (const franka::Exception& ex) {
    std::cerr << "Eccezione Franka: " << ex.what() << "\n";
    save_log_to_csv(log_data, "log_error_seq.csv");
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Eccezione: " << e.what() << "\n";
    save_log_to_csv(log_data, "log_error_seq.csv");
    return -1;
  }

  save_log_to_csv(log_data, "log_capsule_success_last_hard.csv");
  return 0;
}