cmake_minimum_required(VERSION 3.5)
project(simulazione_robot_tau CXX)

# Richiesto C++14 per libfranka e Pinocchio
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DIPENDENZE ---
find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)

# --- INCLUDE ---

# --- ESEGUIBILE ---
add_executable(simulazione_robot_tau #demo_stop_event_traj
    src/simulazione_robot_tau.cpp
)

target_link_libraries(simulazione_robot_tau PRIVATE
    pinocchio::pinocchio
    Eigen3::Eigen
)