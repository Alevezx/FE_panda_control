#pragma once
#include <array>
#include <vector>
#include <string>
#include "shared_state.h"


struct Keyframe {
    double time;
    Skeleton skeleton;
};
//  Two modes, mirroring Python's load_skeleton / receive_skeleton 

// Virtual mode: reads pre-recorded skeleton from XML file
// Replays positions in sync with elapsed time
// Runs in its own thread, writes to shared_skeleton
void loadSkeletonXML(const std::string& xml_path);

// Live mode: continuously reads from a temp file
// written by external pose estimator (camera)
// Runs in its own thread, writes to shared_skeleton
void receiveSkeletonLive(const std::string& temp_path);

// Live mode: subscribes over ZeroMQ to the merged
// multi-camera skeleton stream published by data_merging.py
// (utils.data_transmitter.DataTransmitter, topic "MERGED", device_id 10 ->
// tcp port 6001 + 10 = 6011). Assumes data_merging.py and this process run
// on the same host (per DataTransmitter's hardcoded "tcp://localhost:...").
// Runs in its own thread, writes to shared_skeleton, and sets
// skeleton_stale if no valid frame has arrived within STALE_TIMEOUT_S.
void receiveSkeletonMerged(const std::string& host = "localhost",
                            int device_id = 10,
                            const std::string& topic = "MERGED");
