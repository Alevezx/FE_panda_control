#include "skeleton_reader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <limits>
#include <iomanip>
#include <zmq.h>

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

            if (id < 0 || id >= 15) continue;   // 15 is the minimum number, used in the xml files
                                                // not to be confused with the 21 points from the camera logic
 
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
        if (keypoint.size() == 15) {        // 15 is legacy number, the camera sends more anyway
                                            // if you change it to 21 check the skeleton capsule logic
            std::lock_guard<std::mutex> lock(skeleton_mutex);
            shared_skeleton = keypoint;
        }

        // Don't spin too fast — pose estimator updates at ~30Hz, matching camera rate
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}


//  receiveSkeletonMerged helpers 
 
// Parses one DataTransmitter skeleton message of the form:
//   "<topic>_<id>; [[x,y,z], [x,y,z], ...]; [c0, c1, ...]"
// as produced by DataTransmitter._send_skeleton_data (json.dumps on each
// array, semicolon-joined). Only the first array (positions) is used —
// data_merging.py always sends merged_confidence as all-ones, so it carries
// no per-joint validity information; NaN in the position array is the only
// signal we act on downstream (via DistanceResult / hasNaN()).
//
// json.dumps emits the bare, non-standard token "NaN" for missing values.
// std::stod (strtod) parses "nan"/"NaN"/"NAN" case-insensitively and
// returns a quiet NaN without throwing, so no special-casing is needed for
// that token itself.
//
// Returns false (frame rejected) only on structural malformation — a
// truncated/garbled message — not on the presence of NaN, which is a
// normal, expected value here.
static bool parseMergedSkeletonMessage(const std::string& raw, Skeleton& out) {

    //std::cout << raw << "\n";
    size_t start_idx = raw.find("[[");
    if (start_idx == std::string::npos) {
        std::cerr << "Parser error on [[\n";
        return false;
    }
    size_t end_idx = raw.find("]]", start_idx);
    if (end_idx == std::string::npos) { 
        std::cerr << "Parser error on ]]\n";
        return false;
    }
    std::string points_str = raw.substr(start_idx + 1, end_idx - (start_idx));
    
    out.clear();

    size_t pos = 0;
    while ((pos = points_str.find('[', pos)) != std::string::npos) {
        size_t close_bracket = points_str.find(']', pos);
        if (close_bracket == std::string::npos) break;

        std::string triplet = points_str.substr(pos + 1, close_bracket - (pos + 1));
        std::stringstream ss(triplet);
        std::string x_str, y_str, z_str;

        if (std::getline(ss, x_str, ',') &&
            std::getline(ss, y_str, ',') &&
            std::getline(ss, z_str)) {
            
            std::array<double, 3> pt;
            try {
                pt[0] = (x_str.find("NaN") != std::string::npos) ? NAN : std::stod(x_str);
                pt[1] = (y_str.find("NaN") != std::string::npos) ? NAN : std::stod(y_str);
                pt[2] = (z_str.find("NaN") != std::string::npos) ? NAN : std::stod(z_str);
            } catch (...) {
                pt = {NAN, NAN, NAN};
            }
            out.push_back(pt);
        }

        pos = close_bracket + 1;
    }
    /*std::cout << "Skeleton points (" << out.size() << " total):\n";
    for (size_t i = 0; i < out.size(); ++i) {
        const auto& pt = out[i];
        std::cout << "  [" << i << "] -> ";
        if (std::isnan(pt[0]) && std::isnan(pt[1]) && std::isnan(pt[2])) {
            std::cout << "NaN, NaN, NaN\n";
        } else {
            std::cout << "x: " << pt[0] << ", y: " << pt[1] << ", z: " << pt[2] << "\n";
        }
    }*/

    return !out.empty();
}

// Formats a Skeleton back into the same "[[x,y,z], ...]" shape produced by
// json.dumps() in DataTransmitter._send_skeleton_data — including the
// capital-N "NaN" token, since Python's json module matches that literal
// case-sensitively (a lowercase "nan", which is what plain ostream
// formatting of a NaN double would otherwise emit, is not valid JSON and
// would fail to parse on the analysis side).
 
void receiveSkeletonMerged(const std::string& host, int device_id, const std::string& topic,
                            const std::string& log_path) {
    const int port = 6001 + device_id;   // mirrors DataTransmitter's port = base_port + device_id
    const std::string endpoint = "tcp://" + host + ":" + std::to_string(port);
    const std::string subscribe_topic = topic + "_" + std::to_string(device_id);
 
    // Fail-safe: if no valid frame has arrived within this long, flag the
    // skeleton data as stale so the collision checker forces a stop rather
    // than continuing to trust a frozen (or never-received) skeleton.
    constexpr double STALE_TIMEOUT_S = 0.1;
    constexpr int    POLL_TIMEOUT_MS = 100;  // re-check trajectory_done / staleness this often
 
    // logging, opened once at thread start
    std::ofstream log_file;
    if (!log_path.empty()) {
        log_file.open(log_path, std::ios::out | std::ios::app);
        if (!log_file.is_open()) {
            std::cerr << "skeleton_reader: WARNING could not open log file '"
                       << log_path << "' — continuing without logging\n";
        } else {
            std::cout << "skeleton_reader: logging received frames to '" << log_path << "'\n";
        }
    }

    void* ctx  = zmq_ctx_new();
    void* sock = zmq_socket(ctx, ZMQ_SUB);
 
    int conflate = 1;  // mirrors the sender's SNDHWM=1: we only ever want the latest frame
    zmq_setsockopt(sock, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    zmq_setsockopt(sock, ZMQ_SUBSCRIBE, subscribe_topic.c_str(), subscribe_topic.size());
 
    if (zmq_connect(sock, endpoint.c_str()) != 0) {
        std::cerr << "skeleton_reader: failed to connect to " << endpoint
                   << ": " << zmq_strerror(zmq_errno()) << "\n";
        zmq_close(sock);
        zmq_ctx_destroy(ctx);
        skeleton_stale.store(true);
        return;
    }
 
    std::cout << "skeleton_reader: subscribed to '" << subscribe_topic
               << "' at " << endpoint << "\n";
 
    auto last_valid_frame = std::chrono::steady_clock::now();
    bool warned_stale = false;
 
    zmq_pollitem_t items[] = { { sock, 0, ZMQ_POLLIN, 0 } };
 
    while (!trajectory_done.load()) {
        int rc = zmq_poll(items, 1, POLL_TIMEOUT_MS);
 
        if (rc > 0 && (items[0].revents & ZMQ_POLLIN)) {
            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int nbytes = zmq_msg_recv(&msg, sock, 0);
 
            if (nbytes >= 0) {
                std::string raw(static_cast<char*>(zmq_msg_data(&msg)), nbytes);
                zmq_msg_close(&msg);

                //std::cout << raw << "\n";
 
                Skeleton parsed;
                if (parseMergedSkeletonMessage(raw, parsed)) {
                    /*std::cout << "Skeleton points (" << parsed.size() << " total):\n";
                    for (size_t i = 0; i < parsed.size(); ++i) {
                        const auto& pt = parsed[i];
                        std::cout << "  [" << i << "] -> ";
                        if (std::isnan(pt[0]) && std::isnan(pt[1]) && std::isnan(pt[2])) {
                            std::cout << "NaN, NaN, NaN\n";
                        } else {
                            std::cout << "x: " << pt[0] << ", y: " << pt[1] << ", z: " << pt[2] << "\n";
                        }
                    }*/
                    if (log_file.is_open()) {
                        log_file << parsed.size() << "\n";
                        for (size_t i = 0; i < parsed.size(); ++i) {
                            const auto& pt = parsed[i];
                            log_file << "  [" << i << "] -> ";
                            if (std::isnan(pt[0]) && std::isnan(pt[1]) && std::isnan(pt[2])) {
                                log_file << "NaN, NaN, NaN\n";
                            } else {
                                log_file << "x: " << pt[0] << ", y: " << pt[1] << ", z: " << pt[2] << "\n";
                            }
                        }
                    }
                    {
                        std::lock_guard<std::mutex> lock(skeleton_mutex);
                        shared_skeleton = std::move(parsed);
                    }

                    last_valid_frame = std::chrono::steady_clock::now();
                    skeleton_stale.store(false);
 
                    if (warned_stale) {
                        std::cerr << "skeleton_reader: recovered, receiving fresh frames again\n";
                        warned_stale = false;
                    }
                } else {
                    std::cerr << "skeleton_reader: discarding malformed skeleton message\n";
                }
            } else {
                zmq_msg_close(&msg);
            }
        }
 
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_valid_frame).count();
 
        // last_valid_frame starts at thread-launch time, so this timeout
        // applies equally to "never received a frame at all" and to
        // "was receiving, then stopped" — both are the same failure mode
        // from the collision checker's point of view.
        if (elapsed > STALE_TIMEOUT_S) {
            skeleton_stale.store(true);
            if (!warned_stale) {
                std::cerr << "skeleton_reader: WARNING no valid skeleton frame in "
                           << elapsed << "s (> " << STALE_TIMEOUT_S
                           << "s) — flagging stale, forcing stop condition\n";
                warned_stale = true;
            }
        }
    }
 
    zmq_close(sock);
    zmq_ctx_destroy(ctx);

    zmq_close(sock);
    zmq_ctx_destroy(ctx);

    if (log_file.is_open()) {
        log_file.close();
        std::cout << "skeleton_reader: closed log file '" << log_path << "'\n";
    }
}
