#include <multi_explore_mapping/gap_extraction.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace gap_extraction {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

int normalize_odd(int w) {
    if (w < 1) w = 1;
    return 2 * (w / 2) + 1;
}

// median_win幅(奇数)の中央値フィルタ。循環(モジュロn)窓。
// 戻り値はinfを含みうる(isOpen判定に使う)。
void median_filter_circular(const std::vector<double>& r, int win, std::vector<double>& out) {
    const int n = static_cast<int>(r.size());
    out.assign(n, 0.0);
    const int h = (win - 1) / 2;
    std::vector<double> window(win);
    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < win; ++k) {
            const int idx = ((i - h + k) % n + n) % n;
            window[k] = r[idx];
        }
        std::nth_element(window.begin(), window.begin() + win / 2, window.end());
        out[i] = window[win / 2];
    }
}

}  // namespace

std::vector<Gap> extract_gaps(const sensor_msgs::msg::LaserScan& scan, const Params& params) {
    std::vector<Gap> gaps;
    const int n = static_cast<int>(scan.ranges.size());
    if (n < 3) return gaps;

    const double inc = scan.angle_increment;
    const double lambda = params.abd_lambda_deg * M_PI / 180.0;
    const double sigma = params.abd_sigma;
    const int win = normalize_odd(params.median_win);

    // 4.1 前処理: inf正規化 + 中央値フィルタ
    static thread_local std::vector<double> r;
    r.resize(n);
    for (int i = 0; i < n; ++i) {
        const float v = scan.ranges[i];
        r[i] = (std::isnan(v) || v < scan.range_min || v > scan.range_max)
                   ? kInf
                   : static_cast<double>(v);
    }

    static thread_local std::vector<double> rf;
    median_filter_circular(r, win, rf);

    static thread_local std::vector<uint8_t> is_open;
    is_open.resize(n);
    for (int i = 0; i < n; ++i) is_open[i] = std::isinf(rf[i]) ? 1 : 0;

    // 4.2 連続性の判定(ABD)。brk[i] = i と i+1 の間が途切れているか。
    static thread_local std::vector<uint8_t> brk;
    brk.resize(n);
    for (int i = 0; i < n; ++i) {
        const int nx = (i + 1) % n;
        if (is_open[i] || is_open[nx]) {
            brk[i] = 1;
            continue;
        }
        const double d_max =
            std::min(rf[i], rf[nx]) * std::sin(inc) / std::sin(lambda - inc) + 3.0 * sigma;
        brk[i] = (std::abs(rf[nx] - rf[i]) > d_max) ? 1 : 0;
    }

    // countSupport(s, dir): sは元のスキャン番号。brk[]はこの番号空間。
    auto count_support = [&](int s, int dir) {
        int c = 1;
        while (c < params.min_support) {
            int t;
            bool cut;
            if (dir > 0) {
                t = (s + 1) % n;
                cut = brk[s] != 0;
            } else {
                t = (s - 1 + n) % n;
                cut = brk[t] != 0;
            }
            if (cut) break;
            ++c;
            s = t;
        }
        return c;
    };

    // 4.3 並べ替え: 先頭が必ず非inf(非open)になるよう回転する。
    int k0 = -1;
    for (int i = 0; i < n; ++i) {
        if (!is_open[i]) { k0 = i; break; }
    }
    if (k0 < 0) return gaps;  // 全周open: ギャップなしで終了

    static thread_local std::vector<int> idx;     // 長さ n+1
    static thread_local std::vector<uint8_t> ro;   // ro[m] = is_open[idx[m]]
    static thread_local std::vector<double> rr;    // rr[m] = rf[idx[m]]
    static thread_local std::vector<double> xx, yy;
    idx.resize(n + 1);
    ro.resize(n + 1);
    rr.resize(n + 1);
    xx.resize(n + 1);
    yy.resize(n + 1);
    for (int m = 0; m <= n; ++m) {
        const int orig = (k0 + m) % n;
        idx[m] = orig;
        ro[m] = is_open[orig];
        rr[m] = rf[orig];
        if (!ro[m]) {
            const double th = scan.angle_min + static_cast<double>(orig) * inc;
            xx[m] = rr[m] * std::cos(th);
            yy[m] = rr[m] * std::sin(th);
        } else {
            xx[m] = 0.0;
            yy[m] = 0.0;
        }
    }

    // 4.4 開口ギャップ(open): ro[1..n-1]の中の連続true区間
    int a = -1;
    for (int m = 1; m <= n; ++m) {
        if (ro[m] && a < 0) {
            a = m;
        } else if (!ro[m] && a >= 0) {
            const int b = m - 1;
            const int length = b - a + 1;
            do {
                if (length < params.min_open_samples) break;

                const double pax = xx[a - 1], pay = yy[a - 1];
                const double pbx = xx[b + 1], pby = yy[b + 1];
                const double width = std::hypot(pbx - pax, pby - pay);
                if (width < params.min_gap_width) break;

                if (params.support_open_edges) {
                    if (count_support(idx[a - 1], -1) < params.min_support) break;
                    if (count_support(idx[b + 1], +1) < params.min_support) break;
                }

                double tx = (pax + pbx) * 0.5;
                double ty = (pay + pby) * 0.5;
                const double tnorm = std::hypot(tx, ty);
                if (tnorm > 1e-9) {
                    tx += params.target_push * tx / tnorm;
                    ty += params.target_push * ty / tnorm;
                }

                Gap g;
                g.idx_start = idx[a];
                g.idx_end   = idx[b];
                g.type      = GapType::kOpen;
                g.target_x  = tx;
                g.target_y  = ty;
                g.extent    = width;
                gaps.push_back(g);
            } while (false);
            a = -1;
        }
    }

    // 4.5 段差ギャップ(jump): 隣接ペア (rr[j], rr[j+1]), j=0..n-1
    for (int j = 0; j < n; ++j) {
        const bool both = !ro[j] && !ro[j + 1];
        if (!both) continue;

        const double ra = rr[j], rb = rr[j + 1];
        const double d_max = std::min(ra, rb) * std::sin(inc) / std::sin(lambda - inc) + 3.0 * sigma;
        const double diff_r = std::abs(rb - ra);
        const bool is_jump = diff_r > d_max && diff_r >= params.min_jump;
        if (!is_jump) continue;

        int near, far, dir_near, dir_far;
        if (ra <= rb) {
            near = j;     far = j + 1;  dir_near = -1; dir_far = +1;
        } else {
            near = j + 1; far = j;      dir_near = +1; dir_far = -1;
        }

        if (count_support(idx[near], dir_near) < params.min_support) continue;
        if (count_support(idx[far], dir_far) < params.min_support) continue;

        const double pnx = xx[near], pny = yy[near];
        const double pfx = xx[far],  pfy = yy[far];

        double tx, ty;
        if (params.jump_target_mode == Params::JumpTargetMode::kBetween) {
            tx = pnx + params.jump_ratio * (pfx - pnx);
            ty = pny + params.jump_ratio * (pfy - pny);
        } else {
            const double pn_norm = std::hypot(pnx, pny);
            double radial_x = 1.0, radial_y = 0.0;
            if (pn_norm > 1e-9) { radial_x = pnx / pn_norm; radial_y = pny / pn_norm; }
            double tang_x = -radial_y, tang_y = radial_x;
            const double dot = tang_x * (pfx - pnx) + tang_y * (pfy - pny);
            if (dot < 0.0) { tang_x = -tang_x; tang_y = -tang_y; }
            tx = pnx + params.corner_offset * tang_x;
            ty = pny + params.corner_offset * tang_y;
        }

        Gap g;
        g.idx_start = idx[near];
        g.idx_end   = idx[far];
        g.type      = GapType::kJump;
        g.target_x  = tx;
        g.target_y  = ty;
        g.extent    = diff_r;
        gaps.push_back(g);
    }

    return gaps;
}

}  // namespace gap_extraction
