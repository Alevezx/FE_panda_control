cmake_minimum_required(VERSION 3.5)
project(demo_stop_event_traj CXX)

# Richiesto C++17 per libfranka e Pinocchio
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DIPENDENZE ---
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)

# CasADi e Pinocchio
find_package(casadi REQUIRED)
find_package(pinocchio REQUIRED)

# zeroMQ
find_library(ZMQ_LIBRARY zmq REQUIRED)

# --- INCLUDE ---

# --- ESEGUIBILE ---
add_executable(demo_stop_event_traj
    src/demo_stop_event_traj.cpp
    src/ik_solver.cpp
    src/examples_common.cpp
    src/calc_stop_duration.cpp
    src/flag_stop.cpp
    src/geometry.cpp
    src/skeleton_reader.cpp
)

target_link_options(demo_stop_event_traj PRIVATE 
    -Wl,--disable-new-dtags
    -Wl,-rpath,/usr/local/lib)

target_link_libraries(demo_stop_event_traj PRIVATE
    stdc++fs
    pinocchio::pinocchio
    Franka::Franka
    Eigen3::Eigen
    casadi
    Threads::Threads
    ${ZMQ_LIBRARY}
    /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
)

target_include_directories(demo_stop_event_traj PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)