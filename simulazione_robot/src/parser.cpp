#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <cmath>
#include <iostream>



using Skeleton = std::vector<std::array<double, 3>>;

static bool parseMergedSkeletonMessage(const std::string& raw, Skeleton& out) {
    size_t start_idx = raw.find("[[");
    if (start_idx == std::string::npos) return false;

    size_t end_idx = raw.find("]]", start_idx);
    if (end_idx == std::string::npos) return false;

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

    return !out.empty();
}

int main() {
    std::string raw = "MERGED_10; [[-0.20506539940834045, -0.13627255707979202, 0.7120959162712097], [-0.2233787328004837, -0.06571776419878006, 0.663881778717041], [-0.25218236446380615, -0.07535262405872345, 0.6475233435630798], [-1.4324250221252441, 1.2997841835021973, 0.7384591698646545], [NaN, NaN, NaN], [NaN, NaN, NaN], [NaN, NaN, NaN], [NaN, NaN, NaN], [NaN, NaN, NaN], [-0.23778054863214493, -0.07053519412875175, 0.6557025611400604], [-0.23423340916633606, 0.03262082301080227, 0.42816682159900665], [-0.24785013496875763, 0.06621512770652771, 0.42706963419914246], [-0.2206166833639145, -0.000973481684923172, 0.42926400899887085], [-0.2585764527320862, -0.13817232847213745, 0.4113174080848694], [NaN, NaN, NaN], [-0.28337007761001587, -0.06781148165464401, 0.18420208990573883], [NaN, NaN, NaN], [-0.25132063031196594, -0.058384522795677185, 0.16410420835018158], [-0.2674388587474823, -0.052775297313928604, 0.2130645215511322], [-0.34543824195861816, -0.0891549363732338, 0.10325377434492111], [-0.6859784126281738, 0.1943461298942566, 0.10019435733556747]]; [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]";
    Skeleton skeleton;

    if (parseMergedSkeletonMessage(raw, skeleton)) {
        std::cout << "Parsed good skeleton:\n";
        for (size_t i = 0; i < skeleton.size(); ++i) {
            const auto& pt = skeleton[i];
            std::cout << "  [" << i << "] -> ";
            if (std::isnan(pt[0]) && std::isnan(pt[1]) && std::isnan(pt[2])) {
                std::cout << "NaN, NaN, NaN\n";
            } else {
                std::cout << "x: " << pt[0] << ", y: " << pt[1] << ", z: " << pt[2] << "\n";
            }
        } 
    }
}