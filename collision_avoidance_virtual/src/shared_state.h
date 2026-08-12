#pragma once
#include <array>
#include <vector>
#include <mutex>
#include <atomic>

//  Skeleton type 
// 15 joints, each a 3D point
using Skeleton = std::vector<std::array<double, 3>>;

//  Robot state 
// Written by control thread, read by collision checker thread
struct RobotState {
    std::array<double, 7> q;
    std::array<double, 7> q_p;
    std::array<double, 7> q_pp;
};

//  Shared variables 
inline std::mutex    robot_state_mutex;
inline RobotState    shared_robot_state;

inline std::atomic<double> shared_stop_duration{0.4};

inline std::mutex    skeleton_mutex;
inline Skeleton      shared_skeleton;

//  Thread control flags 
inline std::atomic<bool> stop_flag{false};
inline std::atomic<bool> trajectory_done{false};