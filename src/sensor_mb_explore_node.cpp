/**
 * Sensor-based exploration with Nav2 move-base control.
 *
 * Frontier detection : gap-based (sensor_processor), same algorithm as sensor_explore_node
 * Movement control   : Nav2 NavigateToPose action (obstacle avoidance delegated to Nav2)
 * Multi-robot        : same /shared_* topics as sensor_explore_node
 *
 * Global costmap uses each robot's own SLAM map (/robot_N/map), not the merged map.
 * Start with: ros2 launch multi_explore_mapping sensor_mb_explore.launch.py
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
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

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class SensorMbExploreNode : public rclcpp::Node {
public:
    SensorMbExploreNode() : Node("sensor_mb_explore") {
        declare_parameter("safe_distance",     0.5);
        declare_parameter("robot_radius",      0.13);
        declare_parameter("dup_radius",         1.5);
        declare_parameter("dup_time",         600.0);
        declare_parameter("frontier_ttl",     300.0);
        declare_parameter("min_gap_width",      0.4);
        declare_parameter("odom_frame",   std::string("map"));
        declare_parameter("global_frame", std::string("map"));
        declare_parameter("progress_timeout",  60.0);
        declare_parameter("goal_tolerance",     0.5);
        declare_parameter("blacklist_radius",   1.0);

        safe_             = get_parameter("safe_distance").as_double();
        robot_r_          = get_parameter("robot_radius").as_double();
        dup_r_            = get_parameter("dup_radius").as_double();
        dup_t_            = get_parameter("dup_time").as_double();
        frontier_ttl_     = get_parameter("frontier_ttl").as_double();
        min_gap_w_        = get_parameter("min_gap_width").as_double();
        odom_frame_       = get_parameter("odom_frame").as_string();
        global_frame_     = get_parameter("global_frame").as_string();
        progress_timeout_ = get_parameter("progress_timeout").as_double();
        goal_tol_         = get_parameter("goal_tolerance").as_double();
        blacklist_r_      = get_parameter("blacklist_radius").as_double();

        ns_ = get_namespace();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
                latest_scan_ = *m; has_scan_ = true;
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { on_odom(*m); });

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/shared_exploration_poses", rclcpp::SystemDefaultsQoS());
        peer_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/shared_exploration_poses", rclcpp::SystemDefaultsQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { on_peer_pose(*m); });

        frontier_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/shared_frontiers", rclcpp::SystemDefaultsQoS());
        frontier_peer_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/shared_frontiers", rclcpp::SystemDefaultsQoS(),
            [this](geometry_msgs::msg::PoseArray::SharedPtr m) { on_peer_frontiers(*m); });

        target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/shared_targets", rclcpp::SystemDefaultsQoS());
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/shared_targets", rclcpp::SystemDefaultsQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { on_peer_target(*m); });

        viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "exploration_markers", rclcpp::SystemDefaultsQoS());

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() { exploration_loop(); });
        share_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publish_pose(); publish_frontiers(); });
        target_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() { publish_target(); });
        viz_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { publish_markers(); });

        RCLCPP_INFO(get_logger(),
            "sensor_mb_explore: safe=%.2f r=%.3f dup_r=%.1f gap_w=%.2f timeout=%.0fs ns=%s",
            safe_, robot_r_, dup_r_, min_gap_w_, progress_timeout_, ns_.c_str());
    }

private:
    double safe_, robot_r_, dup_r_, dup_t_, frontier_ttl_, min_gap_w_;
    double progress_timeout_, goal_tol_, blacklist_r_;
    std::string odom_frame_, global_frame_, ns_;

    sensor_msgs::msg::LaserScan latest_scan_;
    bool has_scan_ = false;

    struct Pose   { double x, y, yaw; };
    struct OdomPt { double x, y, t;   };

    Pose pose_{};
    bool has_odom_ = false;

    std::vector<OdomPt>                        history_;
    std::map<std::string, std::vector<OdomPt>> peer_histories_;
    std::map<std::string, OdomPt>              peer_targets_;

    double target_x_{0.0}, target_y_{0.0};
    bool   has_target_{false};

    struct Frontier   { double x, y, t, width; };
    struct BlackEntry { double x, y, t; };

    std::vector<Frontier>   frontier_store_;
    std::vector<BlackEntry> blacklist_;

    // Nav2 action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    GoalHandleNav::SharedPtr current_goal_handle_;
    enum class NavState { IDLE, MOVING } nav_state_{NavState::IDLE};
    rclcpp::Time goal_sent_time_;
    double current_goal_x_{0.0}, current_goal_y_{0.0};

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  peer_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    frontier_peer_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr       frontier_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr timer_, share_timer_, target_timer_, viz_timer_;

    // ------------------------------------------------------------------
    // Odometry

    void on_odom(const nav_msgs::msg::Odometry& msg) {
        const double x   = msg.pose.pose.position.x;
        const double y   = msg.pose.pose.position.y;
        const auto&  q   = msg.pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        pose_ = {x, y, yaw}; has_odom_ = true;

        const double t = now().seconds(), cutoff = t - 600.0;
        history_.push_back({x, y, t});
        history_.erase(std::remove_if(history_.begin(), history_.end(),
            [cutoff](const OdomPt& p) { return p.t < cutoff; }), history_.end());
    }

    // ------------------------------------------------------------------
    // Frontier management

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

    bool is_blacklisted(double wx, double wy) const {
        const double t_now = now().seconds();
        for (const auto& b : blacklist_)
            if (std::hypot(wx - b.x, wy - b.y) < blacklist_r_ && t_now - b.t < 120.0)
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
    // Multi-robot coordination

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
        hist.erase(std::remove_if(hist.begin(), hist.end(),
            [cutoff](const OdomPt& p) { return p.t < cutoff; }), hist.end());
    }

    void publish_frontiers() {
        if (frontier_store_.empty()) return;
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp    = now();
        msg.header.frame_id = ns_;
        for (const auto& f : frontier_store_) {
            geometry_msgs::msg::Pose p;
            p.position.x = f.x; p.position.y = f.y;
            msg.poses.push_back(p);
        }
        frontier_pub_->publish(msg);
    }

    void on_peer_frontiers(const geometry_msgs::msg::PoseArray& msg) {
        if (msg.header.frame_id == ns_) return;
        for (const auto& p : msg.poses)
            add_frontier(p.position.x, p.position.y, 1.0);
    }

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
    // Nav2 action

    void send_nav_goal(double x, double y) {
        if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "navigate_to_pose server not available");
            return;
        }

        auto goal                         = NavigateToPose::Goal{};
        goal.pose.header.frame_id         = global_frame_;
        goal.pose.header.stamp            = now();
        goal.pose.pose.position.x         = x;
        goal.pose.pose.position.y         = y;
        goal.pose.pose.orientation.w      = 1.0;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
        opts.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr& gh) {
                if (!gh) {
                    RCLCPP_WARN(get_logger(), "Goal rejected by Nav2");
                    nav_state_ = NavState::IDLE;
                    return;
                }
                current_goal_handle_ = gh;
            };
        opts.result_callback =
            [this](const GoalHandleNav::WrappedResult& res) {
                using Code = rclcpp_action::ResultCode;
                if (res.code == Code::ABORTED) {
                    RCLCPP_WARN(get_logger(),
                        "Nav2 aborted (%.2f, %.2f) — blacklisting for 120s",
                        current_goal_x_, current_goal_y_);
                    blacklist_.push_back({current_goal_x_, current_goal_y_, now().seconds()});
                }
                nav_state_ = NavState::IDLE;
                current_goal_handle_.reset();
            };

        nav_client_->async_send_goal(goal, opts);
        current_goal_x_ = x;
        current_goal_y_ = y;
        goal_sent_time_ = now();
        nav_state_      = NavState::MOVING;
        target_x_   = x;
        target_y_   = y;
        has_target_ = true;
        RCLCPP_INFO(get_logger(), "Nav goal → (%.2f, %.2f)", x, y);
    }

    // ------------------------------------------------------------------
    // Exploration loop (5 Hz)

    void exploration_loop() {
        if (!has_scan_ || !has_odom_) return;

        // Gap detection: reuse sensor_proc::process; only gap_targets are used here.
        // VFH outputs are discarded — obstacle avoidance is delegated to Nav2.
        const auto ps = sensor_proc::process(
            latest_scan_, safe_, 1.5, 30.0, 0.1, 0.6, robot_r_, 30.0, min_gap_w_);
        for (const auto& [a, d, w] : ps.gap_targets) {
            const double wa = pose_.yaw + a;
            add_frontier(pose_.x + d * std::cos(wa), pose_.y + d * std::sin(wa), w);
        }
        prune_frontiers();

        // While Nav2 is driving, only watch for the progress timeout
        if (nav_state_ == NavState::MOVING) {
            if ((now() - goal_sent_time_).seconds() > progress_timeout_) {
                RCLCPP_WARN(get_logger(), "Progress timeout — cancelling (%.2f, %.2f)",
                    current_goal_x_, current_goal_y_);
                if (current_goal_handle_)
                    nav_client_->async_cancel_goal(current_goal_handle_);
                blacklist_.push_back({current_goal_x_, current_goal_y_, now().seconds()});
                nav_state_ = NavState::IDLE;
                current_goal_handle_.reset();
            }
            return;
        }

        // IDLE: select best unvisited, non-blacklisted frontier
        // Cost: |bearing| + 0.25*dist − 0.3*width + peer_proximity_penalty
        double best_cost = std::numeric_limits<double>::max();
        double best_fx = 0.0, best_fy = 0.0;
        bool   found = false;

        for (const auto& f : frontier_store_) {
            if (is_visited(f.x, f.y))    continue;
            if (is_blacklisted(f.x, f.y)) continue;

            const double dx   = f.x - pose_.x;
            const double dy   = f.y - pose_.y;
            const double raw  = std::atan2(dy, dx) - pose_.yaw;
            const double ang  = std::atan2(std::sin(raw), std::cos(raw));
            const double dist = std::hypot(dx, dy);
            double cost = std::abs(ang) + 0.25 * dist - 0.3 * f.width;

            // Penalize frontiers near peer robots (linear falloff within 2*dup_r_)
            const double avoid_r = dup_r_ * 2.0;
            for (const auto& [peer_ns, hist] : peer_histories_) {
                if (hist.empty()) continue;
                const auto& p  = hist.back();
                const double d = std::hypot(f.x - p.x, f.y - p.y);
                if (d < avoid_r)
                    cost += (M_PI * 0.5) * (1.0 - d / avoid_r);
            }

            if (cost < best_cost) {
                best_cost = cost; best_fx = f.x; best_fy = f.y; found = true;
            }
        }

        if (found && std::hypot(best_fx - pose_.x, best_fy - pose_.y) > goal_tol_)
            send_nav_goal(best_fx, best_fy);
    }

    // ------------------------------------------------------------------
    // Visualization (same as sensor_explore_node)

    static visualization_msgs::msg::Marker::_color_type
    ns_color(const std::string& ns, float alpha = 1.0f) {
        static const std::array<std::array<float, 3>, 5> pal = {{
            {0.3f, 0.5f, 1.0f}, {0.2f, 0.85f, 0.3f}, {1.0f, 0.35f, 0.35f},
            {1.0f, 0.6f, 0.1f}, {0.75f, 0.3f, 0.9f},
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
            m.ns = marker_ns; m.id = id++;
            m.type   = type;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = lifetime;
            return m;
        };

        // Own path
        {
            using M = visualization_msgs::msg::Marker;
            auto m  = make_base(M::LINE_STRIP, ns_ + "/path");
            m.scale.x = 0.03; m.color = ns_color(ns_);
            double lx = std::numeric_limits<double>::max(), ly = 0.0;
            for (const auto& p : history_) {
                if (std::hypot(p.x - lx, p.y - ly) < 0.1) continue;
                geometry_msgs::msg::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.05; m.points.push_back(pt);
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
                pt.x = f.x; pt.y = f.y; pt.z = 0.12; m.points.push_back(pt);
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
            m.scale.x = 0.03; m.color = ns_color(peer_ns, 0.6f);
            for (const auto& p : hist) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.05; m.points.push_back(pt);
            }
            if (m.points.size() >= 2) arr.markers.push_back(m);
        }

        if (!arr.markers.empty()) viz_pub_->publish(arr);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMbExploreNode>());
    rclcpp::shutdown();
}
