cmake_minimum_required(VERSION 3.5)
project(demo_stop_event_traj CXX)

# Richiesto C++14 per libfranka e Pinocchio
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DIPENDENZE ---
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)

# CasADi e Pinocchio
find_package(casadi REQUIRED)
find_package(pinocchio REQUIRED)

# --- INCLUDE ---

# --- ESEGUIBILE ---
add_executable(demo_stop_event_traj
    src/demo_stop_event_traj.cpp
    src/examples_common.cpp
    src/header_stop_event.cpp
    src/flag_stop.cpp
    src/geometry.cpp
)

target_link_options(demo_stop_event_traj PRIVATE 
    -Wl,--disable-new-dtags
    -Wl,-rpath,/usr/local/lib)

target_link_libraries(demo_stop_event_traj PRIVATE
    pinocchio::pinocchio
    Franka::Franka
    Eigen3::Eigen
    casadi
    Threads::Threads
    /lib/x86_64-linux-gnu/liburdfdom_model.so
    /lib/x86_64-linux-gnu/liburdfdom_world.so
    /lib/x86_64-linux-gnu/liburdfdom_sensor.so
    /lib/x86_64-linux-gnu/liburdfdom_model_state.so
)

target_include_directories(demo_stop_event_traj PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)