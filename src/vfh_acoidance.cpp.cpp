#include <multi_explore_mapping/vfh_avoidance.hpp>
#include <algorithm>

namespace avoidance {

VFH::VFH(double safe_dist, double vfh_threshold, double valley_min_deg,
         double emergency_dist, double angular_speed)
    : safe_(safe_dist),
      vfh_t_(vfh_threshold),
      v_min_(valley_min_deg * M_PI / 180.0),
      emerg_d_(emergency_dist),
      ang_(angular_speed),
      hist_(N, 0.0)
{}

// --------------------------------------------------------------------------
// Step 1: build polar obstacle density histogram from cluster list.
//
// Each cluster contributes its angular half-width ±atan2(radius+0.05, min_dist)
// to the histogram, weighted by proximity.  Single-beam noise that fails to
// form a valid cluster never appears here.

void VFH::build_histogram(const std::vector<cluster::Cluster>& clusters) {
    std::fill(hist_.begin(), hist_.end(), 0.0);
    const double step = 2.0 * M_PI / N;

    for (const auto& c : clusters) {
        if (c.min_dist < 0.01) continue;

        const double half_ang = std::atan2(c.radius + 0.05, c.min_dist);
        const double a_lo = c.angle - half_ang;
        const double a_hi = c.angle + half_ang;

        // Weight: 1.0 at origin, 0.0 at 2× safe_distance
        const double w = std::max(0.0, (safe_ * 2.0 - c.min_dist) / (safe_ * 2.0));
        if (w <= 0.0) continue;

        const int s_lo = static_cast<int>((a_lo + M_PI) / step);
        const int s_hi = static_cast<int>((a_hi + M_PI) / step) + 1;
        for (int s = s_lo; s <= s_hi; ++s)
            hist_[((s % N) + N) % N] += w;
    }
}

// --------------------------------------------------------------------------
// Step 2: check if any forward-hemisphere cluster is inside the safety fence.
// Returns the threatening cluster's angle, or NaN if no emergency.

double VFH::check_emergency(const std::vector<cluster::Cluster>& clusters) const {
    double closest    = std::numeric_limits<double>::max();
    double best_angle = std::numeric_limits<double>::quiet_NaN();

    for (const auto& c : clusters) {
        if (std::abs(c.angle) > M_PI * 0.5) continue;   // forward hemisphere only
        if (c.min_dist < closest) {
            closest    = c.min_dist;
            best_angle = c.angle;
        }
    }
    return (closest < emerg_d_) ? best_angle
                                 : std::numeric_limits<double>::quiet_NaN();
}

// --------------------------------------------------------------------------
// Step 3: check whether the ±15° forward cone is blocked.

bool VFH::front_blocked() const {
    const double step  = 2.0 * M_PI / N;
    const int    front = N / 2;
    const int    half_w = std::max(1, static_cast<int>(M_PI / 12.0 / step));
    for (int d = -half_w; d <= half_w; ++d)
        if (hist_[(front + d + N) % N] > vfh_t_) return true;
    return false;
}

// --------------------------------------------------------------------------
// Step 4: find passable valleys in the histogram.
// Doubles the array to handle circular wrap-around correctly.
// Returns {angle_to_valley_centre [rad], valley_width [rad]}.

std::vector<std::pair<double, double>> VFH::find_valleys() const {
    const double step    = 2.0 * M_PI / N;
    const int    min_sec = std::max(1, static_cast<int>(v_min_ / step));
    std::vector<std::pair<double, double>> result;

    int run_start = -1;
    for (int i = 0; i < 2 * N; ++i) {
        const bool blocked = hist_[i % N] > vfh_t_;
        if (!blocked) {
            if (run_start < 0) run_start = i;
        } else {
            if (run_start >= 0) {
                const int length = i - run_start;
                if (length >= min_sec) {
                    const int    mid   = (run_start + i - 1) / 2;
                    double angle = (mid % N) * step - M_PI;
                    angle = std::atan2(std::sin(angle), std::cos(angle));
                    result.push_back({angle, length * step});
                }
                run_start = -1;
            }
        }
    }
    // Handle run that continues to the end of the doubled array
    if (run_start >= 0) {
        const int length = 2 * N - run_start;
        if (length >= min_sec) {
            const int mid  = (run_start + 2 * N - 1) / 2;
            double angle   = (mid % N) * step - M_PI;
            angle = std::atan2(std::sin(angle), std::cos(angle));
            result.push_back({angle, length * step});
        }
    }
    return result;
}

// --------------------------------------------------------------------------
// Main entry point: run pipeline and return avoidance decision.

AvoidResult VFH::compute(const std::vector<cluster::Cluster>& clusters) {
    build_histogram(clusters);

    // Emergency takes priority over everything else
    const double emerg_angle = check_emergency(clusters);
    if (!std::isnan(emerg_angle)) {
        const double az = ang_ * (emerg_angle >= 0.0 ? -1.0 : 1.0);
        return {AvoidResult::State::EMERGENCY, az};
    }

    if (!front_blocked())
        return {AvoidResult::State::CLEAR, 0.0};

    // Forward blocked: steer toward the nearest valley
    const auto valleys = find_valleys();
    if (!valleys.empty()) {
        const auto best = std::min_element(valleys.begin(), valleys.end(),
            [](const auto& a, const auto& b) {
                return std::abs(a.first) < std::abs(b.first);
            });
        const double az = ang_ * (best->first >= 0.0 ? 1.0 : -1.0);
        return {AvoidResult::State::BLOCKED, az};
    }

    // Fully surrounded: rotate toward the less-congested side
    const int front = N / 2;
    double left = 0.0, right = 0.0;
    for (int i = 0; i < N / 4; ++i) {
        left  += hist_[(front + i) % N];
        right += hist_[(front - 1 - i + N) % N];
    }
    const double az = (left <= right) ? ang_ : -ang_;
    return {AvoidResult::State::BLOCKED, az};
}

} // namespace avoidance
