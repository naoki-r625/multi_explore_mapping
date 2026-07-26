/**
 * Sensor-based autonomous exploration node.
 *
 * Obstacle avoidance : cluster-based VFH via sensor_processor
 * Exploration        : persistent frontier store + multi-robot coordination
 *
 * Multi-robot topics
 *   /shared_exploration_poses  (PoseStamped, 1 Hz): own odometry trail
 *   /shared_frontiers          (PoseArray,   1 Hz): detected-but-unvisited branch points
 *   /shared_targets            (PoseStamped, 5 Hz): current navigation goal (claim)
 *     — When robot A claims a frontier as its nav goal, peers treat that
 *       position as "virtually visited" so they select a different frontier.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <multi_explore_mapping/sensor_processor.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

class SensorExploreNode : public rclcpp::Node {
public:
    SensorExploreNode() : Node("sensor_explore") {
        declare_parameter("linear_speed",    0.2);
        declare_parameter("angular_speed",   0.6);
        declare_parameter("safe_distance",   0.5);
        declare_parameter("vfh_threshold",   1.5);
        declare_parameter("valley_min_deg", 30.0);
        declare_parameter("emergency_dist",  0.1);
        declare_parameter("robot_radius",   0.089);
        declare_parameter("dup_radius",      1.5);
        declare_parameter("dup_time",      600.0);   // match 10-min history window
        declare_parameter("frontier_ttl",  300.0);
        declare_parameter("front_cone_deg", 30.0);
        declare_parameter("min_gap_width",   0.4);
        declare_parameter("odom_frame", std::string("odom"));

        lin_          = get_parameter("linear_speed").as_double();
        ang_          = get_parameter("angular_speed").as_double();
        safe_         = get_parameter("safe_distance").as_double();
        vfh_t_        = get_parameter("vfh_threshold").as_double();
        v_deg_        = get_parameter("valley_min_deg").as_double();
        emerg_        = get_parameter("emergency_dist").as_double();
        robot_r_      = get_parameter("robot_radius").as_double();
        dup_r_        = get_parameter("dup_radius").as_double();
        dup_t_        = get_parameter("dup_time").as_double();
        frontier_ttl_ = get_parameter("frontier_ttl").as_double();
        fcone_        = get_parameter("front_cone_deg").as_double();
        min_gap_w_    = get_parameter("min_gap_width").as_double();
        odom_frame_   = get_parameter("odom_frame").as_string();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
                latest_scan_ = *m; has_scan_ = true;
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { on_odom(*m); });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        ns_ = get_namespace();

        // Multi-robot: pose trail (1 Hz)
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/shared_exploration_poses", rclcpp::SystemDefaultsQoS());
        peer_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/shared_exploration_poses", rclcpp::SystemDefaultsQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { on_peer_pose(*m); });

        // Multi-robot: frontier sharing (1 Hz)
        frontier_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/shared_frontiers", rclcpp::SystemDefaultsQoS());
        frontier_peer_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/shared_frontiers", rclcpp::SystemDefaultsQoS(),
            [this](geometry_msgs::msg::PoseArray::SharedPtr m) { on_peer_frontiers(*m); });

        // Multi-robot: navigation target claim (5 Hz)
        target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/shared_targets", rclcpp::SystemDefaultsQoS());
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/shared_targets", rclcpp::SystemDefaultsQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { on_peer_target(*m); });

        viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "exploration_markers", rclcpp::SystemDefaultsQoS());

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { control_loop(); });

        share_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publish_pose(); publish_frontiers(); });

        target_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),   // 5 Hz
            [this]() { publish_target(); });

        viz_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { publish_markers(); });

        RCLCPP_INFO(get_logger(),
            "sensor_explore: safe=%.2f r=%.3f emerg=%.2f "
            "dup_r=%.1f dup_t=%.0f gap_w=%.2f ns=%s",
            safe_, robot_r_, emerg_, dup_r_, dup_t_, min_gap_w_, ns_.c_str());
    }

private:
    double lin_, ang_, safe_, vfh_t_, v_deg_, emerg_, robot_r_;
    double dup_r_, dup_t_, frontier_ttl_, fcone_, min_gap_w_;
    std::string odom_frame_;

    sensor_msgs::msg::LaserScan latest_scan_;
    bool has_scan_ = false;

    struct Pose   { double x, y, yaw; };
    struct OdomPt { double x, y, t;   };

    Pose pose_{};
    bool has_odom_ = false;

    std::vector<OdomPt> history_;
    std::map<std::string, std::vector<OdomPt>> peer_histories_;

    // Per-peer latest claimed navigation target.
    // Stored as OdomPt for the timestamp; only the latest target per peer is kept.
    std::map<std::string, OdomPt> peer_targets_;

    std::string ns_;

    double target_x_{0.0}, target_y_{0.0};
    bool   has_target_{false};

    // ------------------------------------------------------------------
    // Persistent frontier store

    struct Frontier {
        double x, y;    // world coordinates [m]
        double t;       // discovery time [s]
        double width;   // gap opening width [m]
    };
    std::vector<Frontier> frontier_store_;

    // ------------------------------------------------------------------
    // ROS handles

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  peer_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    frontier_peer_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr       frontier_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr share_timer_;
    rclcpp::TimerBase::SharedPtr target_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;

    // ------------------------------------------------------------------
    // Odometry

    void on_odom(const nav_msgs::msg::Odometry& msg) {
        const double x = msg.pose.pose.position.x;
        const double y = msg.pose.pose.position.y;
        const auto& q = msg.pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        pose_     = {x, y, yaw};
        has_odom_ = true;

        const double t      = now().seconds();
        const double cutoff = t - 600.0;
        history_.push_back({x, y, t});
        history_.erase(
            std::remove_if(history_.begin(), history_.end(),
                [cutoff](const OdomPt& p) { return p.t < cutoff; }),
            history_.end());
    }

    // ------------------------------------------------------------------
    // Visited / claimed check
    //
    // A world position counts as "handled" when:
    //   (a) this robot was within dup_r_ of it within the last dup_t_ seconds
    //   (b) a peer was within dup_r_ of it within the last dup_t_ seconds
    //   (c) a peer has CLAIMED it as its current navigation goal

    bool is_visited(double wx, double wy) const {
        const double t_now = now().seconds();
        for (const auto& p : history_)
            if (std::hypot(wx - p.x, wy - p.y) < dup_r_ && (t_now - p.t) < dup_t_)
                return true;
        for (const auto& [pns, hist] : peer_histories_)
            for (const auto& p : hist)
                if (std::hypot(wx - p.x, wy - p.y) < dup_r_ && (t_now - p.t) < dup_t_)
                    return true;
        for (const auto& [pns, tgt] : peer_targets_)
            if (std::hypot(wx - tgt.x, wy - tgt.y) < dup_r_ && (t_now - tgt.t) < dup_t_)
                return true;
        return false;
    }

    void add_frontier(double wx, double wy, double width = 1.0) {
        if (is_visited(wx, wy)) return;
        for (const auto& f : frontier_store_)
            if (std::hypot(wx - f.x, wy - f.y) < dup_r_ * 0.5) return;
        frontier_store_.push_back({wx, wy, now().seconds(), width});
    }

    void prune_frontiers() {
        const double t_now = now().seconds();
        frontier_store_.erase(
            std::remove_if(frontier_store_.begin(), frontier_store_.end(),
                [&](const Frontier& f) {
                    return (t_now - f.t > frontier_ttl_) || is_visited(f.x, f.y);
                }),
            frontier_store_.end());
    }

    // ------------------------------------------------------------------
    // Multi-robot: pose trail sharing (1 Hz)

    void publish_pose() {
        if (!has_odom_) return;
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = now();
        msg.header.frame_id = ns_;
        msg.pose.position.x = pose_.x;
        msg.pose.position.y = pose_.y;
        pose_pub_->publish(msg);
    }

    void on_peer_pose(const geometry_msgs::msg::PoseStamped& msg) {
        if (msg.header.frame_id == ns_) return;
        const double t = now().seconds();
        auto& hist = peer_histories_[msg.header.frame_id];
        hist.push_back({msg.pose.position.x, msg.pose.position.y, t});
        const double cutoff = t - 600.0;
        hist.erase(
            std::remove_if(hist.begin(), hist.end(),
                [cutoff](const OdomPt& p) { return p.t < cutoff; }),
            hist.end());
    }

    // ------------------------------------------------------------------
    // Multi-robot: frontier sharing (1 Hz)

    void publish_frontiers() {
        if (frontier_store_.empty()) return;
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp    = now();
        msg.header.frame_id = ns_;
        for (const auto& f : frontier_store_) {
            geometry_msgs::msg::Pose p;
            p.position.x = f.x;
            p.position.y = f.y;
            msg.poses.push_back(p);
        }
        frontier_pub_->publish(msg);
    }

    void on_peer_frontiers(const geometry_msgs::msg::PoseArray& msg) {
        if (msg.header.frame_id == ns_) return;
        for (const auto& p : msg.poses)
            add_frontier(p.position.x, p.position.y, 1.0);
    }

    // ------------------------------------------------------------------
    // Multi-robot: navigation target claim (5 Hz)

    void publish_target() {
        if (!has_target_) return;
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = now();
        msg.header.frame_id = ns_;
        msg.pose.position.x = target_x_;
        msg.pose.position.y = target_y_;
        target_pub_->publish(msg);
    }

    void on_peer_target(const geometry_msgs::msg::PoseStamped& msg) {
        if (msg.header.frame_id == ns_) return;
        peer_targets_[msg.header.frame_id] = {
            msg.pose.position.x, msg.pose.position.y, now().seconds()
        };
    }

    // ------------------------------------------------------------------
    // Control loop (10 Hz)

    void control_loop() {
        if (!has_scan_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for LiDAR data...");
            return;
        }

        const auto ps = sensor_proc::process(
            latest_scan_, safe_, vfh_t_, v_deg_, emerg_, ang_,
            robot_r_, fcone_, min_gap_w_);

        // Convert gap targets (robot-frame polar + width) to world frontiers
        if (has_odom_) {
            for (const auto& [a, d, w] : ps.gap_targets) {
                const double wa = pose_.yaw + a;
                add_frontier(pose_.x + d * std::cos(wa),
                             pose_.y + d * std::sin(wa), w);
            }
            prune_frontiers();
        }

        geometry_msgs::msg::Twist cmd;

        if (ps.emergency) {
            cmd.angular.z = ps.avoidance_angular_z;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "EMERGENCY: obstacle inside %.2fm fence", emerg_);

        } else if (!ps.front_blocked) {
            // Select best frontier: cost = |bearing| + 0.1*dist − 0.3*width
            // Prefer forward + nearby + wide openings
            double best_angle = 0.0;
            double best_cost  = std::numeric_limits<double>::max();
            double best_fx = 0.0, best_fy = 0.0;

            if (has_odom_) {
                for (const auto& f : frontier_store_) {
                    if (is_visited(f.x, f.y)) continue;
                    const double dx   = f.x - pose_.x;
                    const double dy   = f.y - pose_.y;
                    const double raw  = std::atan2(dy, dx) - pose_.yaw;
                    const double ang  = std::atan2(std::sin(raw), std::cos(raw));
                    const double dist = std::hypot(dx, dy);
                    const double cost = std::abs(ang) + 0.1 * dist - 0.3 * f.width;
                    if (cost < best_cost) {
                        best_cost  = cost;
                        best_angle = ang;
                        best_fx    = f.x;
                        best_fy    = f.y;
                    }
                }
                if (best_cost < std::numeric_limits<double>::max()) {
                    target_x_   = best_fx;
                    target_y_   = best_fy;
                    has_target_ = true;
                }
            }

            cmd.linear.x = lin_;
            if (std::abs(best_angle) > 0.15)
                cmd.angular.z = ang_ * 0.4 * (best_angle > 0.0 ? 1.0 : -1.0);

        } else {
            cmd.angular.z = ps.avoidance_angular_z;
            RCLCPP_DEBUG(get_logger(), "blocked: steer=%.2f", cmd.angular.z);
        }

        cmd_pub_->publish(cmd);
    }

    // ------------------------------------------------------------------
    // Visualization (2 Hz)
    //
    //   ns_/path        LINE_STRIP  own odometry trail
    //   ns_/frontiers   SPHERE_LIST frontier store (yellow=fresh → grey=aged)
    //   ns_/target      SPHERE      current nav goal (magenta)
    //   peer_ns/path    LINE_STRIP  peer trails (semi-transparent)

    static visualization_msgs::msg::Marker::_color_type
    ns_color(const std::string& ns, float alpha = 1.0f) {
        static const std::array<std::array<float, 3>, 5> pal = {{
            {0.3f, 0.5f, 1.0f}, {0.2f, 0.85f, 0.3f}, {1.0f, 0.35f, 0.35f},
            {1.0f, 0.6f,  0.1f}, {0.75f, 0.3f, 0.9f},
        }};
        const size_t idx = std::hash<std::string>{}(ns) % pal.size();
        visualization_msgs::msg::Marker::_color_type c;
        c.r = pal[idx][0]; c.g = pal[idx][1]; c.b = pal[idx][2]; c.a = alpha;
        return c;
    }

    void publish_markers() {
        visualization_msgs::msg::MarkerArray arr;
        const auto stamp    = now();
        const auto lifetime = rclcpp::Duration::from_seconds(2.0);
        int id = 0;

        auto make_base = [&](int type, const std::string& marker_ns) {
            visualization_msgs::msg::Marker m;
            m.header.stamp    = stamp;
            m.header.frame_id = odom_frame_;
            m.ns      = marker_ns;
            m.id      = id++;
            m.type    = type;
            m.action  = visualization_msgs::msg::Marker::ADD;
            m.lifetime = lifetime;
            return m;
        };

        // Own path
        {
            using M = visualization_msgs::msg::Marker;
            auto m  = make_base(M::LINE_STRIP, ns_ + "/path");
            m.scale.x = 0.03;
            m.color   = ns_color(ns_);
            double lx = std::numeric_limits<double>::max(), ly = 0.0;
            for (const auto& p : history_) {
                if (std::hypot(p.x - lx, p.y - ly) < 0.1) continue;
                geometry_msgs::msg::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.05;
                m.points.push_back(pt);
                lx = p.x; ly = p.y;
            }
            if (m.points.size() >= 2) arr.markers.push_back(m);
        }

        // Frontier store (yellow=fresh → grey=aged)
        {
            using M = visualization_msgs::msg::Marker;
            auto m  = make_base(M::SPHERE_LIST, ns_ + "/frontiers");
            m.scale.x = m.scale.y = m.scale.z = 0.15;
            const double t_now = now().seconds();
            for (const auto& f : frontier_store_) {
                geometry_msgs::msg::Point pt;
                pt.x = f.x; pt.y = f.y; pt.z = 0.12;
                m.points.push_back(pt);
                const float age = static_cast<float>(
                    std::min(1.0, (t_now - f.t) / frontier_ttl_));
                visualization_msgs::msg::Marker::_color_type c;
                c.r = 1.0f; c.g = 0.9f - 0.6f * age;
                c.b = 0.1f + 0.6f * age; c.a = 1.0f;
                m.colors.push_back(c);
            }
            if (!m.points.empty()) arr.markers.push_back(m);
        }

        // Current nav target (magenta)
        if (has_target_) {
            using M = visualization_msgs::msg::Marker;
            auto m  = make_base(M::SPHERE, ns_ + "/target");
            m.pose.position.x = target_x_;
            m.pose.position.y = target_y_;
            m.pose.position.z = 0.2;
            m.scale.x = m.scale.y = m.scale.z = 0.25;
            m.color.r = 1.0f; m.color.g = 0.0f;
            m.color.b = 1.0f; m.color.a = 1.0f;
            arr.markers.push_back(m);
        }

        // Peer paths (semi-transparent)
        for (const auto& [peer_ns, hist] : peer_histories_) {
            using M = visualization_msgs::msg::Marker;
            auto m  = make_base(M::LINE_STRIP, peer_ns + "/path");
            m.scale.x = 0.03;
            m.color   = ns_color(peer_ns, 0.6f);
            for (const auto& p : hist) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.05;
                m.points.push_back(pt);
            }
            if (m.points.size() >= 2) arr.markers.push_back(m);
        }

        if (!arr.markers.empty()) viz_pub_->publish(arr);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorExploreNode>());
    rclcpp::shutdown();
}
