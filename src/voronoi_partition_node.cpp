/**
 * Voronoi Partition Node
 * =======================
 * Computes a per-robot territory mask over the merged map (/map, published
 * by icp_map_matching_node) and publishes it as /<robot>/voronoi_mask
 * (nav_msgs/OccupancyGrid, values: 100=exclusive, 50=shared buffer band,
 * 0=other robot's territory, -1=unassigned).
 *
 * Robot positions are read from TF (map -> <robot>/base_footprint) every
 * cycle, so this works uniformly regardless of whether a given robot is
 * currently running sensor_explore_node or the Nav2 + frontier_explore_node
 * stack — territory allocation is orthogonal to exploration mode.
 *
 * See docs/voronoi_partition.md for the design rationale (why geodesic
 * distance, why the buffer band, why unreached cells stay unassigned).
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "multi_explore_mapping/voronoi_partition.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

class VoronoiPartitionNode : public rclcpp::Node {
public:
    VoronoiPartitionNode()
    : rclcpp::Node("voronoi_partition"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        declare_parameter<std::vector<std::string>>("robot_names", std::vector<std::string>{});
        declare_parameter<std::string>("base_frame_suffix", "base_footprint");
        declare_parameter<std::string>("global_frame", "map");
        declare_parameter<std::string>("map_topic", "/map");
        declare_parameter<double>("buffer_width_m", 2.0);
        declare_parameter<double>("recompute_period_sec", 5.0);
        declare_parameter<int>("downsample_factor", 2);
        declare_parameter<int>("obstacle_threshold", 50);
        declare_parameter<bool>("visualize", true);
        declare_parameter<double>("hysteresis_margin_m", 1.5);

        robot_names_        = get_parameter("robot_names").as_string_array();
        base_frame_suffix_  = get_parameter("base_frame_suffix").as_string();
        global_frame_       = get_parameter("global_frame").as_string();
        buffer_width_m_     = get_parameter("buffer_width_m").as_double();
        downsample_factor_  = std::max(1, static_cast<int>(get_parameter("downsample_factor").as_int()));
        obstacle_threshold_ = static_cast<int8_t>(get_parameter("obstacle_threshold").as_int());
        visualize_          = get_parameter("visualize").as_bool();
        hysteresis_margin_m_ = get_parameter("hysteresis_margin_m").as_double();

        if (robot_names_.empty()) {
            RCLCPP_WARN(get_logger(),
                "robot_names parameter is empty — no /voronoi_mask topics will be published. "
                "Pass it from adaptive_explore.launch.py (derived from the ROBOTS list).");
        }

        const std::string map_topic = get_parameter("map_topic").as_string();
        map_sub_ = create_subscription<OccupancyGrid>(
            map_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(map_mutex_);
                latest_map_ = msg;
            });

        for (size_t i = 0; i < robot_names_.size(); ++i) {
            mask_pubs_[robot_names_[i]] = create_publisher<OccupancyGrid>(
                "/" + robot_names_[i] + "/voronoi_mask",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
            RCLCPP_INFO(get_logger(), "  %s -> color #%zu", robot_names_[i].c_str(), i % 6);
        }

        if (visualize_) {
            viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                "/voronoi_partition/markers",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        }

        schedule_next_cycle(1.0);  // let /map + TF warm up a little before the first attempt

        RCLCPP_INFO(get_logger(),
            "voronoi_partition ready | robots=%zu buffer=%.2fm hysteresis=%.2fm downsample=%d "
            "map_topic=%s visualize=%s",
            robot_names_.size(), buffer_width_m_, hysteresis_margin_m_, downsample_factor_,
            map_topic.c_str(), visualize_ ? "true" : "false");
    }

private:
    // ------------------------------------------------------------------
    // Self-rescheduling timer (same pattern as icp_map_matching_node):
    // avoids overlapping cycles if a computation runs long on a large map.

    void schedule_next_cycle(double delay_sec) {
        timer_ = create_wall_timer(
            std::chrono::duration<double>(std::max(0.1, delay_sec)),
            [this]() { run_cycle(); });
    }

    void run_cycle() {
        timer_->cancel();
        const auto t0 = std::chrono::steady_clock::now();
        recompute_and_publish();
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const double period = get_parameter("recompute_period_sec").as_double();

        RCLCPP_DEBUG(get_logger(), "voronoi cycle: %.3fs (target %.2fs)", elapsed, period);
        schedule_next_cycle(period - elapsed);
    }

    // ------------------------------------------------------------------
    // Map downsampling (occupied > free > unknown priority, same
    // convention as icp_map_matching_node::downsample). Ownership only
    // needs metre-scale resolution, so this keeps large worlds (e.g.
    // rmf_demos airport_terminal) tractable.

    static OccupancyGrid downsample(const OccupancyGrid& src, int factor) {
        if (factor <= 1) return src;

        OccupancyGrid dst;
        dst.header = src.header;
        dst.info   = src.info;
        dst.info.resolution = src.info.resolution * factor;

        const int W  = static_cast<int>(src.info.width);
        const int H  = static_cast<int>(src.info.height);
        const int dW = (W + factor - 1) / factor;
        const int dH = (H + factor - 1) / factor;
        dst.info.width  = static_cast<uint32_t>(dW);
        dst.info.height = static_cast<uint32_t>(dH);
        dst.data.assign(static_cast<size_t>(dW) * dH, -1);

        for (int row = 0; row < H; ++row) {
            const int drow = row / factor;
            for (int col = 0; col < W; ++col) {
                const int8_t v = src.data[static_cast<size_t>(row) * W + col];
                const size_t didx = static_cast<size_t>(drow) * dW + (col / factor);
                if (dst.data[didx] < v) dst.data[didx] = v;
            }
        }
        return dst;
    }

    // ------------------------------------------------------------------

    bool lookup_robot_xy(const std::string& name, double& x, double& y) {
        const std::string frame = name + "/" + base_frame_suffix_;
        try {
            const auto tf = tf_buffer_.lookupTransform(
                global_frame_, frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "TF lookup failed (%s -> %s): %s", global_frame_.c_str(), frame.c_str(), e.what());
            return false;
        }
    }

    void recompute_and_publish() {
        OccupancyGrid::SharedPtr map_ptr;
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            map_ptr = latest_map_;
        }
        if (!map_ptr) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                "Waiting for merged map...");
            return;
        }

        // robot_names_ の並びを毎サイクル固定して robots[i] の i を安定させる
        // (compute_partition の owner インデックスがこの i と一致するため、
        // ヒステリシス比較が前回サイクルの同じロボットを正しく参照できる)。
        // TFが一時的に取れない場合は最後にわかっている位置で埋め、その
        // ロボットの担当領域が一瞬だけ消えて隣に明け渡される→揺り戻す、
        // という余計な入れ替わりが起きないようにする。
        std::vector<voronoi::RobotPose> robots;
        robots.reserve(robot_names_.size());
        for (const auto& name : robot_names_) {
            double x = 0.0, y = 0.0;
            if (lookup_robot_xy(name, x, y)) {
                last_known_pose_[name] = {x, y};
            } else {
                auto it = last_known_pose_.find(name);
                if (it == last_known_pose_.end()) continue;  // まだ一度も見えていない
                x = it->second.first;
                y = it->second.second;
            }
            robots.push_back({name, x, y});
        }
        if (robots.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "No robot poses available this cycle — skipping partition");
            return;
        }

        const OccupancyGrid work_map = downsample(*map_ptr, downsample_factor_);
        const auto fields = voronoi::compute_partition(
            work_map, robots, obstacle_threshold_, &prev_fields_, hysteresis_margin_m_);

        for (size_t i = 0; i < robots.size(); ++i) {
            auto it = mask_pubs_.find(robots[i].name);
            if (it == mask_pubs_.end()) continue;  // shouldn't happen
            auto mask = voronoi::build_mask(fields, static_cast<int>(i), buffer_width_m_);
            mask.header.stamp    = now();
            mask.header.frame_id = global_frame_;
            it->second->publish(mask);
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 15000,
            "Published voronoi masks for %zu/%zu robots (map %dx%d @ %.2fm/cell)",
            robots.size(), robot_names_.size(),
            static_cast<int>(work_map.info.width), static_cast<int>(work_map.info.height),
            static_cast<double>(work_map.info.resolution));

        publish_visualization(fields);
        prev_fields_ = fields;
    }

    // ------------------------------------------------------------------
    // RViz visualization: one POINTS marker covering the whole partition,
    // colored by owning robot. Shared buffer-band cells (see build_mask)
    // are drawn at lower alpha so the boundary between territories is
    // visible at a glance. Unassigned cells (owner < 0) are left blank.

    static visualization_msgs::msg::Marker::_color_type robot_color(int idx) {
        static const std::array<std::array<float, 3>, 6> pal = {{
            {0.3f, 0.5f, 1.0f}, {0.2f, 0.85f, 0.3f}, {1.0f, 0.35f, 0.35f},
            {1.0f, 0.6f,  0.1f}, {0.75f, 0.3f, 0.9f}, {0.2f, 0.85f, 0.85f},
        }};
        const auto& p = pal[static_cast<size_t>(idx) % pal.size()];
        visualization_msgs::msg::Marker::_color_type c;
        c.r = p[0]; c.g = p[1]; c.b = p[2]; c.a = 1.0f;
        return c;
    }

    void publish_visualization(const voronoi::PartitionFields& fields) {
        if (!visualize_ || !viz_pub_) return;

        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp    = now();
        m.ns   = "voronoi_partition";
        m.id   = 0;
        m.type = visualization_msgs::msg::Marker::POINTS;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = static_cast<float>(fields.resolution);

        for (int y = 0; y < fields.height; ++y) {
            for (int x = 0; x < fields.width; ++x) {
                const size_t idx = static_cast<size_t>(y) * fields.width + x;
                const int8_t owner = fields.owner[idx];
                if (owner < 0) continue;  // unassigned: leave blank

                geometry_msgs::msg::Point pt;
                pt.x = fields.origin_x + (x + 0.5) * fields.resolution;
                pt.y = fields.origin_y + (y + 0.5) * fields.resolution;
                pt.z = 0.02;
                m.points.push_back(pt);

                const bool shared = (fields.second_owner[idx] >= 0) &&
                    ((fields.dist_second[idx] - fields.dist_owner[idx]) <
                     static_cast<float>(buffer_width_m_));
                auto c = robot_color(owner);
                c.a = shared ? 0.35f : 0.75f;
                m.colors.push_back(c);
            }
        }

        if (!m.points.empty()) arr.markers.push_back(m);
        viz_pub_->publish(arr);
    }

    // ------------------------------------------------------------------

    std::vector<std::string> robot_names_;
    std::string base_frame_suffix_;
    std::string global_frame_;
    double buffer_width_m_;
    int    downsample_factor_;
    int8_t obstacle_threshold_;
    bool   visualize_;
    double hysteresis_margin_m_;

    tf2_ros::Buffer           tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
    std::map<std::string, rclcpp::Publisher<OccupancyGrid>::SharedPtr> mask_pubs_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    OccupancyGrid::SharedPtr latest_map_;
    std::mutex                map_mutex_;

    // ヒステリシス用の前回計算結果、およびTF瞬断時のフォールバック位置
    voronoi::PartitionFields prev_fields_;
    std::map<std::string, std::pair<double, double>> last_known_pose_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoronoiPartitionNode>());
    rclcpp::shutdown();
    return 0;
}
