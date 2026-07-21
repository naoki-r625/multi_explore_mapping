/**
 * Sensor-based autonomous exploration node.
 *
 * Obstacle avoidance : VFH (Vector Field Histogram) from raw LaserScan
 * Exploration        : gap frontier detection + duplicate prevention
 *
 * Reference: Paper A (Makino et al. 2019) Sec.3.1, 3.2, 3.3, 3.5.1
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

class SensorExploreNode : public rclcpp::Node {
public:
    static constexpr int N = 36;   // VFH sectors (10° each)

    SensorExploreNode() : Node("sensor_explore") {
        declare_parameter("linear_speed",    0.2);
        declare_parameter("angular_speed",   0.6);
        declare_parameter("safe_distance",   0.5);
        declare_parameter("vfh_threshold",   1.5);
        declare_parameter("valley_min_deg", 30.0);
        declare_parameter("dup_radius",      1.5);
        declare_parameter("dup_time",      200.0);

        lin_   = get_parameter("linear_speed").as_double();
        ang_   = get_parameter("angular_speed").as_double();
        safe_  = get_parameter("safe_distance").as_double();
        vfh_t_ = get_parameter("vfh_threshold").as_double();
        v_min_ = get_parameter("valley_min_deg").as_double() * M_PI / 180.0;
        dup_r_ = get_parameter("dup_radius").as_double();
        dup_t_ = get_parameter("dup_time").as_double();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
                latest_scan_ = *m; has_scan_ = true;
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { on_odom(*m); });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { control_loop(); });

        RCLCPP_INFO(get_logger(),
            "sensor_explore (VFH+gap+dup): safe=%.2f lin=%.2f ang=%.2f",
            safe_, lin_, ang_);
    }

private:
    double lin_, ang_, safe_, vfh_t_, v_min_, dup_r_, dup_t_;

    sensor_msgs::msg::LaserScan latest_scan_;
    bool has_scan_ = false;

    struct Pose   { double x, y, yaw; };
    struct OdomPt { double x, y, t;   };
    Pose   pose_{};
    bool   has_odom_ = false;
    std::vector<OdomPt> history_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------------------
    // Odometry: track pose and keep 10-minute position history

    void on_odom(const nav_msgs::msg::Odometry& msg) {
        double x = msg.pose.pose.position.x;
        double y = msg.pose.pose.position.y;
        const auto& q = msg.pose.pose.orientation;
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        pose_     = {x, y, yaw};
        has_odom_ = true;

        double t = now().seconds();
        history_.push_back({x, y, t});
        const double cutoff = t - 600.0;
        history_.erase(
            std::remove_if(history_.begin(), history_.end(),
                [cutoff](const OdomPt& p) { return p.t < cutoff; }),
            history_.end());
    }

    // ------------------------------------------------------------------
    // VFH: build polar obstacle density histogram from raw scan beams.
    //
    // Each beam within safe_distance*2 adds a proximity-weighted density
    // to its corresponding sector.

    std::vector<double> build_vfh(const sensor_msgs::msg::LaserScan& scan) {
        std::vector<double> hist(N, 0.0);
        const double step = 2.0 * M_PI / N;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max * 0.99)
                continue;
            if (r >= safe_ * 2.0) continue;

            double a = scan.angle_min + i * scan.angle_increment;
            a = std::atan2(std::sin(a), std::cos(a));  // normalise to [-π, π]

            int s = static_cast<int>((a + M_PI) / step);
            s = std::max(0, std::min(N - 1, s));

            hist[s] += (safe_ * 2.0 - r) / (safe_ * 2.0);
        }
        return hist;
    }

    bool front_blocked(const std::vector<double>& hist) {
        const double step   = 2.0 * M_PI / N;
        const int    front  = N / 2;
        const int    half_w = std::max(1, static_cast<int>(M_PI / 12.0 / step));
        for (int d = -half_w; d <= half_w; ++d)
            if (hist[(front + d + N) % N] > vfh_t_) return true;
        return false;
    }

    // Find passable valleys (consecutive low-density sectors wider than valley_min_deg).
    // Doubles the array to handle circular wrap-around.
    std::vector<std::pair<double, double>> vfh_valleys(const std::vector<double>& hist) {
        const double step    = 2.0 * M_PI / N;
        const int    min_sec = std::max(1, static_cast<int>(v_min_ / step));
        std::vector<std::pair<double, double>> result;
        int run_start = -1;

        for (int i = 0; i < 2 * N; ++i) {
            bool blocked = hist[i % N] > vfh_t_;
            if (!blocked) {
                if (run_start < 0) run_start = i;
            } else {
                if (run_start >= 0) {
                    int length = i - run_start;
                    if (length >= min_sec) {
                        int    mid   = (run_start + i - 1) / 2;
                        double angle = (mid % N) * step - M_PI;
                        angle = std::atan2(std::sin(angle), std::cos(angle));
                        result.push_back({angle, length * step});
                    }
                    run_start = -1;
                }
            }
        }
        if (run_start >= 0) {
            int length = 2 * N - run_start;
            if (length >= min_sec) {
                int    mid   = (run_start + 2 * N - 1) / 2;
                double angle = (mid % N) * step - M_PI;
                angle = std::atan2(std::sin(angle), std::cos(angle));
                result.push_back({angle, length * step});
            }
        }
        return result;
    }

    // ------------------------------------------------------------------
    // Gap frontier detection (Paper A Sec.3.2, adapted)
    //
    // Obstacle↔open transitions between adjacent beams are candidate frontiers.
    // Sorted nearest-to-forward first.

    std::vector<std::pair<double, double>> find_gap_targets() const {
        std::vector<std::pair<double, double>> targets;
        const auto& r = latest_scan_.ranges;
        const size_t n = r.size();

        for (size_t i = 0; i + 1 < n; ++i) {
            bool h1 = std::isfinite(r[i])   && r[i]   > latest_scan_.range_min
                      && r[i]   < latest_scan_.range_max * 0.97;
            bool h2 = std::isfinite(r[i+1]) && r[i+1] > latest_scan_.range_min
                      && r[i+1] < latest_scan_.range_max * 0.97;
            if (h1 == h2) continue;

            double a = latest_scan_.angle_min + (i + 0.5) * latest_scan_.angle_increment;
            a = std::atan2(std::sin(a), std::cos(a));
            if (std::abs(a) < M_PI * 0.75)
                targets.push_back({a, h1 ? r[i] : r[i+1]});
        }
        std::sort(targets.begin(), targets.end(),
            [](const auto& p, const auto& q) {
                return std::abs(p.first) < std::abs(q.first);
            });
        return targets;
    }

    // ------------------------------------------------------------------
    // Duplicate exploration prevention (Paper A Sec.3.3 + 3.5.1)

    bool is_duplicate(double angle, double dist) const {
        if (!has_odom_ || history_.empty()) return false;
        const double world_a = pose_.yaw + angle;
        const double proj    = std::min(dist, dup_r_ * 1.5);
        const double tx = pose_.x + proj * std::cos(world_a);
        const double ty = pose_.y + proj * std::sin(world_a);
        const double t_now = now().seconds();
        for (const auto& p : history_)
            if (std::hypot(tx - p.x, ty - p.y) < dup_r_ && (t_now - p.t) < dup_t_)
                return true;
        return false;
    }

    // ------------------------------------------------------------------
    // Control loop (10 Hz)

    void control_loop() {
        if (!has_scan_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for LiDAR data...");
            return;
        }

        auto hist = build_vfh(latest_scan_);
        geometry_msgs::msg::Twist cmd;

        if (!front_blocked(hist)) {
            // Forward clear: steer toward the nearest non-duplicate frontier
            double target_angle = 0.0;
            for (const auto& [a, d] : find_gap_targets())
                if (!is_duplicate(a, d)) { target_angle = a; break; }

            cmd.linear.x = lin_;
            if (std::abs(target_angle) > 0.15)
                cmd.angular.z = ang_ * 0.4 * (target_angle > 0.0 ? 1.0 : -1.0);

        } else {
            // Forward blocked: rotate toward best VFH valley
            auto valleys = vfh_valleys(hist);
            if (!valleys.empty()) {
                auto best = std::min_element(valleys.begin(), valleys.end(),
                    [](const auto& a, const auto& b) {
                        return std::abs(a.first) < std::abs(b.first);
                    });
                cmd.angular.z = ang_ * (best->first >= 0.0 ? 1.0 : -1.0);
            } else {
                // Fully surrounded: rotate toward the less-congested side
                const int front = N / 2;
                double left = 0.0, right = 0.0;
                for (int i = 0; i < N / 4; ++i) {
                    left  += hist[(front + i) % N];
                    right += hist[(front - 1 - i + N) % N];
                }
                cmd.angular.z = (left <= right) ? ang_ : -ang_;
            }
            RCLCPP_DEBUG(get_logger(), "blocked: steer=%.2f", cmd.angular.z);
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorExploreNode>());
    rclcpp::shutdown();
}
