/**
 * Sensor-based autonomous exploration node.
 *
 * Obstacle avoidance : cluster-based VFH via sensor_processor
 * Exploration        : gap frontier detection + duplicate prevention
 *
 * Reference: Paper A (Makino et al. 2019) Sec.3.1, 3.2, 3.3, 3.5.1
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <multi_explore_mapping/sensor_processor.hpp>
#include <algorithm>
#include <cmath>
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
        declare_parameter("emergency_dist",  0.1);   // clearance from robot body edge [m]
        declare_parameter("robot_radius",   0.089);  // TurtleBot3 Burger half-width [m]
        declare_parameter("dup_radius",      1.5);
        declare_parameter("dup_time",      200.0);

        lin_      = get_parameter("linear_speed").as_double();
        ang_      = get_parameter("angular_speed").as_double();
        safe_     = get_parameter("safe_distance").as_double();
        vfh_t_    = get_parameter("vfh_threshold").as_double();
        v_deg_    = get_parameter("valley_min_deg").as_double();
        emerg_    = get_parameter("emergency_dist").as_double();
        robot_r_  = get_parameter("robot_radius").as_double();
        dup_r_    = get_parameter("dup_radius").as_double();
        dup_t_    = get_parameter("dup_time").as_double();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
                latest_scan_ = *m; has_scan_ = true;
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { on_odom(*m); });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Multi-robot coordination: share own visited positions with peer robots.
        // All robots publish/subscribe to the same global topic.
        // frame_id carries the namespace so each robot can ignore its own messages.
        ns_ = get_namespace();
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/shared_exploration_poses", rclcpp::SystemDefaultsQoS());

        peer_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/shared_exploration_poses", rclcpp::SystemDefaultsQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { on_peer_pose(*m); });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { control_loop(); });

        share_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publish_pose(); });

        RCLCPP_INFO(get_logger(),
            "sensor_explore: safe=%.2f robot_r=%.3f emerg=%.2f lin=%.2f ang=%.2f ns=%s",
            safe_, robot_r_, emerg_, lin_, ang_, ns_.c_str());
    }

private:
    double lin_, ang_, safe_, vfh_t_, v_deg_, emerg_, robot_r_, dup_r_, dup_t_;

    sensor_msgs::msg::LaserScan latest_scan_;
    bool has_scan_ = false;

    struct Pose   { double x, y, yaw; };
    struct OdomPt { double x, y, t;   };
    Pose   pose_{};
    bool   has_odom_ = false;
    std::vector<OdomPt> history_;       // own visited positions (high-freq odom)
    std::vector<OdomPt> peer_history_;  // other robots' visited positions (1 Hz shared)
    std::string ns_;                    // own namespace, used to filter self-published poses

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr peer_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr share_timer_;

    // ------------------------------------------------------------------
    // Odometry: track pose and keep a 10-minute position history

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
    // Multi-robot coordination: publish own pose at 1 Hz

    void publish_pose() {
        if (!has_odom_) return;
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = now();
        msg.header.frame_id = ns_;   // identifies the publishing robot
        msg.pose.position.x = pose_.x;
        msg.pose.position.y = pose_.y;
        pose_pub_->publish(msg);
    }

    // Receive peer robots' poses and store in peer_history_.
    // Own messages (same namespace) are ignored to avoid double-counting.

    void on_peer_pose(const geometry_msgs::msg::PoseStamped& msg) {
        if (msg.header.frame_id == ns_) return;   // skip own messages

        const double t = now().seconds();
        peer_history_.push_back({msg.pose.position.x, msg.pose.position.y, t});

        const double cutoff = t - 600.0;
        peer_history_.erase(
            std::remove_if(peer_history_.begin(), peer_history_.end(),
                [cutoff](const OdomPt& p) { return p.t < cutoff; }),
            peer_history_.end());
    }

    // ------------------------------------------------------------------
    // Duplicate exploration prevention (Paper A Sec.3.3 + 3.5.1)
    //
    // Projects the gap target to an estimated world position and checks
    // whether this robot OR any peer has visited that vicinity recently.

    bool is_duplicate(double angle, double dist) const {
        if (!has_odom_) return false;
        const double world_a = pose_.yaw + angle;
        const double proj    = std::min(dist, dup_r_ * 1.5);
        const double tx = pose_.x + proj * std::cos(world_a);
        const double ty = pose_.y + proj * std::sin(world_a);
        const double t_now = now().seconds();

        for (const auto& p : history_)
            if (std::hypot(tx - p.x, ty - p.y) < dup_r_ && (t_now - p.t) < dup_t_)
                return true;

        for (const auto& p : peer_history_)
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

        const auto ps = sensor_proc::process(
            latest_scan_, safe_, vfh_t_, v_deg_, emerg_, ang_, robot_r_);

        geometry_msgs::msg::Twist cmd;

        if (ps.emergency) {
            // Cluster inside safety fence: stop immediately and rotate away.
            cmd.angular.z = ps.avoidance_angular_z;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "EMERGENCY: obstacle inside %.2fm fence", emerg_);

        } else if (!ps.front_blocked) {
            // Forward clear: steer toward nearest non-duplicate gap frontier.
            double target_angle = 0.0;
            for (const auto& [a, d] : ps.gap_targets)
                if (!is_duplicate(a, d)) { target_angle = a; break; }

            cmd.linear.x = lin_;
            if (std::abs(target_angle) > 0.15)
                cmd.angular.z = ang_ * 0.4 * (target_angle > 0.0 ? 1.0 : -1.0);

        } else {
            // Forward blocked: rotate toward the VFH valley selected by sensor_proc.
            cmd.angular.z = ps.avoidance_angular_z;
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
