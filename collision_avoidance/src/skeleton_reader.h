#pragma once
#include <array>
#include <vector>
#include <string>
#include "shared_state.h"


struct Keyframe {
    double time;
    Skeleton skeleton;
};
// ── Two modes, mirroring Python's load_skeleton / receive_skeleton ────

// Virtual mode: reads pre-recorded skeleton from XML file
// Replays positions in sync with elapsed time
// Runs in its own thread, writes to shared_skeleton
void loadSkeletonXML(const std::string& xml_path);

// Live mode: continuously reads from a temp file
// written by external pose estimator (camera)
// Runs in its own thread, writes to shared_skeleton
void receiveSkeletonLive(const std::string& temp_path);