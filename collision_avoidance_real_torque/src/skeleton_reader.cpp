#include "skeleton_reader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

void loadSkeletonXML(const std::string& xml_path) {

    std::ifstream file(xml_path);
    if (!file.good()) {
        std::cerr << "skeleton_reader: cannot open file " << xml_path << "\n";
        return;
    }

    std::vector<Keyframe> keyframes;

    std::string line;
    Keyframe    current;
    bool        in_keypoint = false;

    while (std::getline(file, line)) {
        // <keypoint time="0.123">
        auto kp_pos = line.find("<keypoint");
        if (kp_pos != std::string::npos) {
            auto t_pos = line.find("time='");
            auto t_end = line.find("'", t_pos + 6);
            current.time = std::stod(line.substr(t_pos + 6, t_end - t_pos - 6));
            
            current.skeleton.clear();
            current.skeleton.resize(15);
            in_keypoint = true;
            continue;
        }

        // </keypoint>
        if (line.find("</keypoint>") != std::string::npos) {
            keyframes.push_back(current);
            in_keypoint = false;
            continue;
        }

        // <point id="3">x y z</point>
        if (in_keypoint && line.find("<point") != std::string::npos) {
            auto id_pos = line.find("id='");
            auto id_end = line.find("'", id_pos + 4);
            int  id     = std::stoi(line.substr(id_pos + 4, id_end - id_pos - 4));

            auto val_start = line.find(">", id_end + 1) + 1;
            auto val_end   = line.find("<", val_start);
            std::istringstream ss(line.substr(val_start, val_end - val_start));

            // Read as whitespace-delimited tokens, then convert with std::stod —
            // operator>>(istream&, double&) fails to parse the literal text
            // "nan" on this toolchain, silently leaving the target uninitialized.
            // std::stod (which defers to strtod) handles "nan" correctly.
            std::string tok_x, tok_y, tok_z;
            ss >> tok_x >> tok_y >> tok_z;

            if (id < 0 || id >= 15) continue;
 
            try {
                double x = std::stod(tok_x);
                double y = std::stod(tok_y);
                double z = std::stod(tok_z);
                current.skeleton[id] = {x, y, z};
            } catch (const std::exception& e) {
                std::cerr << "skeleton_reader: failed to parse point id="
                          << id << " (" << e.what() << ")\n";
            }
        }
    }

    file.close();
    std::cout << "skeleton_reader: loaded " << keyframes.size() << " keyframes from XML\n";

    // replay keyframes
    auto t_start = std::chrono::steady_clock::now();

    for (const auto& kf : keyframes) {
        while (true) {
            if (trajectory_done.load()) return;

            double t_exe = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            
            if (t_exe >= kf.time) break;

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        {
            std::lock_guard<std::mutex> lock(skeleton_mutex);
            shared_skeleton = kf.skeleton;
        }
    }
}

void receiveSkeletonLive(const std::string& temp_path) {
    while (!trajectory_done.load()) {
        std::ifstream temp(temp_path);
        if (!temp.good()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        Skeleton    keypoint;
        std::string line;

        while (std::getline(temp, line)) {
            if (line.empty()) continue;

            // Parse tab-separated: id\tx\ty\tz
            // Mirrors: jnt = line.split("\t") then jnt[1:4]
            std::istringstream ss(line);
            std::string        token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, '\t'))
                tokens.push_back(token);

            if (tokens.size() < 4) continue;

            try {
                double x = std::stod(tokens[1]);
                double y = std::stod(tokens[2]);
                double z = std::stod(tokens[3]);
                keypoint.push_back({x, y, z});
            } catch (...) {
                continue;  // skip malformed lines
            }
        }

        temp.close();

        // Write to shared skeleton if we got a full skeleton
        if (keypoint.size() == 15) {
            std::lock_guard<std::mutex> lock(skeleton_mutex);
            shared_skeleton = keypoint;
        }

        // Don't spin too fast — pose estimator updates at ~30Hz, matching camera rate
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}