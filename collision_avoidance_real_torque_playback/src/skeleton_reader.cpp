#include "skeleton_reader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <limits>
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


// ── receiveSkeletonMerged helpers ───────────────────────────────────────
 
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
    auto first_semi = raw.find("; ");
    if (first_semi == std::string::npos) return false;
 
    auto arr_start = raw.find('[', first_semi);
    if (arr_start == std::string::npos) return false;
 
    // Depth-count to the matching closing bracket so we grab exactly the
    // position array and don't run into the confidence array that follows.
    int depth = 0;
    size_t arr_end = std::string::npos;
    for (size_t i = arr_start; i < raw.size(); ++i) {
        if (raw[i] == '[') depth++;
        else if (raw[i] == ']') {
            depth--;
            if (depth == 0) { arr_end = i; break; }
        }
    }
    if (arr_end == std::string::npos) return false;
 
    std::string body = raw.substr(arr_start + 1, arr_end - arr_start - 1);
 
    Skeleton parsed;
    size_t pos = 0;
    while (pos < body.size()) {
        auto t_start = body.find('[', pos);
        if (t_start == std::string::npos) break;
        auto t_end = body.find(']', t_start);
        if (t_end == std::string::npos) break;
 
        std::string triplet = body.substr(t_start + 1, t_end - t_start - 1);
        std::array<double, 3> point{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()
        };
 
        std::istringstream ss(triplet);
        std::string tok;
        int i = 0;
        while (std::getline(ss, tok, ',') && i < 3) {
            auto a = tok.find_first_not_of(" \t");
            if (a != std::string::npos) {
                auto b = tok.find_last_not_of(" \t");
                try {
                    point[i] = std::stod(tok.substr(a, b - a + 1));
                } catch (const std::exception&) {
                    // leave as NaN — malformed single value, not a reason
                    // to reject the whole joint or the whole frame
                }
            }
            ++i;
        }
        if (i != 3) return false;  // genuinely truncated triplet -> reject frame
 
        parsed.push_back(point);
        pos = t_end + 1;
    }
 
    if (parsed.empty()) return false;
 
    out = std::move(parsed);
    return true;
}
 
void receiveSkeletonMerged(const std::string& host, int device_id, const std::string& topic) {
    const int port = 6001 + device_id;   // mirrors DataTransmitter's port = base_port + device_id
    const std::string endpoint = "tcp://" + host + ":" + std::to_string(port);
    const std::string subscribe_topic = topic + "_" + std::to_string(device_id);
 
    // Fail-safe: if no valid frame has arrived within this long, flag the
    // skeleton data as stale so the collision checker forces a stop rather
    // than continuing to trust a frozen (or never-received) skeleton.
    constexpr double STALE_TIMEOUT_S = 0.1;
    constexpr int    POLL_TIMEOUT_MS = 100;  // re-check trajectory_done / staleness this often
 
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
 
                Skeleton parsed;
                if (parseMergedSkeletonMessage(raw, parsed)) {
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
}
