// golden_gap_data/ の MATLAB正解データとの照合(gap_extraction_spec.md 7.1節)。
// 実行: test_gap_extraction <golden_gap_data のディレクトリ>
#include <multi_explore_mapping/gap_extraction.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) out.push_back(tok);
    return out;
}

double parse_double(const std::string& s) {
    if (s == "Inf" || s == "inf") return std::numeric_limits<double>::infinity();
    if (s == "-Inf" || s == "-inf") return -std::numeric_limits<double>::infinity();
    if (s == "NaN" || s == "nan") return std::numeric_limits<double>::quiet_NaN();
    return std::stod(s);
}

struct GoldenGap {
    int type;  // 1=open, 2=jump
    int idx_start, idx_end;  // 1始まり(MATLAB)
    double target_x, target_y, extent;
};

// golden_params.json の固定キーを決め打ちで読む(フル JSON パーサは使わない)。
double json_number(const std::string& text, const std::string& key) {
    const auto pos = text.find("\"" + key + "\"");
    if (pos == std::string::npos) throw std::runtime_error("key not found: " + key);
    const auto colon = text.find(':', pos);
    const auto comma = text.find_first_of(",}", colon);
    return std::stod(text.substr(colon + 1, comma - colon - 1));
}

bool json_bool(const std::string& text, const std::string& key) {
    const auto pos = text.find("\"" + key + "\"");
    const auto colon = text.find(':', pos);
    const auto comma = text.find_first_of(",}", colon);
    const std::string v = text.substr(colon + 1, comma - colon - 1);
    return v.find("true") != std::string::npos;
}

std::string json_string(const std::string& text, const std::string& key) {
    const auto pos = text.find("\"" + key + "\"");
    const auto q1 = text.find('"', text.find(':', pos));
    const auto q2 = text.find('"', q1 + 1);
    return text.substr(q1 + 1, q2 - q1 - 1);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <golden_gap_data_dir>" << std::endl;
        return 2;
    }
    const std::string dir = argv[1];

    // --- params.json ---
    std::ifstream pf(dir + "/golden_params.json");
    std::stringstream pbuf;
    pbuf << pf.rdbuf();
    const std::string ptext = pbuf.str();

    gap_extraction::Params params;
    params.median_win = static_cast<int>(json_number(ptext, "medianWin"));
    params.min_open_samples = static_cast<int>(json_number(ptext, "minOpenSamples"));
    params.min_gap_width = json_number(ptext, "minGapWidth");
    params.target_push = json_number(ptext, "targetPush");
    params.abd_lambda_deg = json_number(ptext, "lambda") * 180.0 / M_PI;
    params.abd_sigma = json_number(ptext, "sigma");
    params.min_jump = json_number(ptext, "minJump");
    params.min_support = static_cast<int>(json_number(ptext, "minSupport"));
    params.support_open_edges = json_bool(ptext, "supportOpenEdges");
    params.jump_ratio = json_number(ptext, "jumpRatio");
    params.corner_offset = json_number(ptext, "cornerOffset");
    params.jump_target_mode = (json_string(ptext, "jumpTargetMode") == "corner")
                                   ? gap_extraction::Params::JumpTargetMode::kCorner
                                   : gap_extraction::Params::JumpTargetMode::kBetween;

    std::printf("params: median_win=%d min_open=%d min_gap_width=%.3f target_push=%.3f "
                "lambda_deg=%.3f sigma=%.3f min_jump=%.3f min_support=%d support_open=%d "
                "jump_ratio=%.3f corner_offset=%.3f\n",
                params.median_win, params.min_open_samples, params.min_gap_width,
                params.target_push, params.abd_lambda_deg, params.abd_sigma, params.min_jump,
                params.min_support, params.support_open_edges ? 1 : 0, params.jump_ratio,
                params.corner_offset);

    // --- scans.csv: frame -> LaserScan ---
    std::map<int, sensor_msgs::msg::LaserScan> scans;
    {
        std::ifstream f(dir + "/golden_scans.csv");
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            auto cols = split_csv(line);
            const int frame = std::stoi(cols[0]);
            sensor_msgs::msg::LaserScan s;
            s.angle_min = std::stod(cols[1]);
            s.angle_increment = std::stod(cols[2]);
            s.range_min = std::stod(cols[3]);
            s.range_max = std::stod(cols[4]);
            s.ranges.reserve(cols.size() - 5);
            for (size_t i = 5; i < cols.size(); ++i) s.ranges.push_back(static_cast<float>(parse_double(cols[i])));
            scans[frame] = std::move(s);
        }
    }

    // --- counts.csv: frame -> expected count ---
    std::map<int, int> counts;
    {
        std::ifstream f(dir + "/golden_counts.csv");
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            auto cols = split_csv(line);
            counts[std::stoi(cols[0])] = std::stoi(cols[1]);
        }
    }

    // --- gaps.csv: frame -> ordered expected gaps (open全部→jump全部、入力順) ---
    std::map<int, std::vector<GoldenGap>> gaps;
    {
        std::ifstream f(dir + "/golden_gaps.csv");
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            auto cols = split_csv(line);
            const int frame = std::stoi(cols[0]);
            GoldenGap g;
            g.type = std::stoi(cols[1]);
            g.idx_start = std::stoi(cols[2]);
            g.idx_end = std::stoi(cols[3]);
            g.target_x = std::stod(cols[4]);
            g.target_y = std::stod(cols[5]);
            g.extent = std::stod(cols[6]);
            gaps[frame].push_back(g);
        }
    }

    int n_frames = 0, n_frames_ok = 0;
    int n_gaps_total = 0, n_gaps_ok = 0;
    const double tol = 1e-4;

    for (const auto& [frame, scan] : scans) {
        ++n_frames;
        const auto result = gap_extraction::extract_gaps(scan, params);
        const auto& expected = gaps[frame];
        const int expected_count = counts.count(frame) ? counts[frame] : -1;

        bool frame_ok = true;
        if (static_cast<int>(result.size()) != expected_count) {
            std::printf("frame %d: count mismatch: got=%zu expected=%d\n", frame, result.size(),
                        expected_count);
            frame_ok = false;
        }
        if (result.size() != expected.size()) {
            std::printf("frame %d: result/golden_gaps size mismatch: got=%zu golden=%zu\n", frame,
                        result.size(), expected.size());
            frame_ok = false;
        }

        const size_t n_compare = std::min(result.size(), expected.size());
        for (size_t i = 0; i < n_compare; ++i) {
            ++n_gaps_total;
            const auto& r = result[i];
            const auto& e = expected[i];
            const int r_type = (r.type == gap_extraction::GapType::kOpen) ? 1 : 2;
            const int r_idx_start = r.idx_start + 1;  // 0始まり -> 1始まり
            const int r_idx_end = r.idx_end + 1;

            bool ok = true;
            if (r_type != e.type) ok = false;
            if (r_idx_start != e.idx_start) ok = false;
            if (r_idx_end != e.idx_end) ok = false;
            if (std::abs(r.target_x - e.target_x) > tol) ok = false;
            if (std::abs(r.target_y - e.target_y) > tol) ok = false;
            if (std::abs(r.extent - e.extent) > tol) ok = false;

            if (ok) {
                ++n_gaps_ok;
            } else {
                frame_ok = false;
                std::printf(
                    "frame %d gap %zu MISMATCH: got(type=%d,idx=%d-%d,x=%.6f,y=%.6f,e=%.6f) "
                    "want(type=%d,idx=%d-%d,x=%.6f,y=%.6f,e=%.6f)\n",
                    frame, i, r_type, r_idx_start, r_idx_end, r.target_x, r.target_y, r.extent,
                    e.type, e.idx_start, e.idx_end, e.target_x, e.target_y, e.extent);
            }
        }

        if (frame_ok) ++n_frames_ok;
    }

    std::printf("\n=== summary ===\nframes: %d/%d OK\ngaps:   %d/%d OK\n", n_frames_ok, n_frames,
                n_gaps_ok, n_gaps_total);
    return (n_frames_ok == n_frames) ? 0 : 1;
}
