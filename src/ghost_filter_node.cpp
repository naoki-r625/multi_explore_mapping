#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

class GhostFilterNode : public rclcpp::Node {
public:
    GhostFilterNode()
    : Node("ghost_filter"),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_)
    {
        declare_parameter<std::vector<std::string>>("peer_namespaces", std::vector<std::string>{});
        declare_parameter<double>("mask_radius", 0.3);

        peer_namespaces_ = get_parameter("peer_namespaces").as_string_array();
        mask_radius_     = get_parameter("mask_radius").as_double();

        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { filter(msg); });

        pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "scan_filtered", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "GhostFilter ready | peers=%zu mask_radius=%.2fm",
            peer_namespaces_.size(), mask_radius_);
    }

private:
    tf2_ros::Buffer           tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<std::string>  peer_namespaces_;
    double                    mask_radius_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    pub_;

    void filter(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto out = *msg;
        const std::string& scan_frame = msg->header.frame_id;

        // 実機のLiDARドライバはno-return(検出なし)をinfで返す(REP-117)のが
        // 一般的だが、Gazebo classicのray sensorプラグインはrange_maxちょうど
        // の数値をそのまま返す。slam_toolboxはこのrange_maxちょうどの値を
        // 「確実にクリア」として扱わないため、センサー範囲内なのに永久に未知
        // のまま残るセルができる。ここでinfに正規化して costmap/slam_toolbox
        // 双方の既存レイトレーシングに正しく「range_maxまでクリア」と伝える。
        for (auto& r : out.ranges) {
            if (r >= out.range_max - 1e-3f) {
                r = std::numeric_limits<float>::infinity();
            }
        }

        for (const auto& peer_ns : peer_namespaces_) {
            const std::string peer_frame = peer_ns + "/base_footprint";
            try {
                auto tf = tf_buffer_.lookupTransform(
                    scan_frame, peer_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.05));

                const double ox = tf.transform.translation.x;
                const double oy = tf.transform.translation.y;

                for (size_t i = 0; i < out.ranges.size(); ++i) {
                    const float r = out.ranges[i];
                    if (!std::isfinite(r) || r <= 0.0f) continue;

                    const double angle = out.angle_min + i * out.angle_increment;
                    const double ex = r * std::cos(angle);
                    const double ey = r * std::sin(angle);

                    if (std::hypot(ex - ox, ey - oy) < mask_radius_)
                        out.ranges[i] = std::numeric_limits<float>::infinity();
                }
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "TF lookup failed (%s): %s", peer_frame.c_str(), e.what());
            }
        }

        pub_->publish(out);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GhostFilterNode>());
    rclcpp::shutdown();
    return 0;
}
