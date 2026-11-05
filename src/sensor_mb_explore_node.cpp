/**
 * Sensor-based exploration node, Nav2-driven (move-base style).
 *
 * Frontier selection : gap detection via sensor_processor (same algorithm as
 *                       sensor_explore_node) + persistent frontier store +
 *                       multi-robot coordination.
 * Movement            : delegated entirely to Nav2 (navigate_to_pose action).
 *                       This node never publishes cmd_vel itself — local
 *                       obstacle avoidance is Nav2's controller_server +
 *                       costmap, same as frontier_explore_node.
 *
 * Multi-robot topics
 *   /shared_exploration_poses  (PoseStamped, 1 Hz): own odometry trail
 *   /shared_frontiers          (PoseArray,   1 Hz): detected-but-unvisited branch points
 *   /shared_targets            (PoseStamped, 5 Hz): current navigation goal (claim)
 *     — When robot A claims a frontier as its nav goal, peers treat that
 *       position as "virtually visited" so they select a different frontier.
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <multi_explore_mapping/sensor_processor.hpp>
#include <multi_explore_mapping/voronoi_partition.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <map>
#include <memory>
#include <string>
#include <vector>

class SensorMbExploreNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav   = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandlePath     = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    SensorMbExploreNode()
    : Node("sensor_mb_explore"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
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
        // ギャップ抽出(docs/gap_extraction_spec.md, gap_extraction.hpp)
        declare_parameter("gap_median_win",        3);
        declare_parameter("gap_min_open_samples",  3);
        declare_parameter("gap_min_width",         0.6);
        declare_parameter("gap_target_push",       0.5);
        declare_parameter("gap_abd_lambda_deg",   10.0);
        declare_parameter("gap_abd_sigma",         0.01);
        declare_parameter("gap_min_jump",          0.5);
        declare_parameter("gap_min_support",       3);
        declare_parameter("gap_support_open_edges", false);
        declare_parameter("gap_jump_target_mode",  std::string("between"));
        declare_parameter("gap_jump_ratio",        0.5);
        declare_parameter("gap_corner_offset",     0.35);
        // 凹コーナー対策(gap_extraction.hpp参照)。既定は無効(inf)。
        declare_parameter("gap_max_open_depth_diff_m",
            std::numeric_limits<double>::infinity());
        declare_parameter("odom_frame", std::string("odom"));
        declare_parameter("global_frame", std::string("map"));
        declare_parameter("use_voronoi_partition", true);
        declare_parameter("filter_explored_frontiers", true);
        declare_parameter("merged_map_topic", std::string("/map"));
        declare_parameter("explored_obstacle_threshold", 50);
        declare_parameter("progress_timeout",  90.0);
        declare_parameter("blacklist_radius",   1.0);
        declare_parameter("blacklist_clear_sec", 60.0);
        // 目標選択のコストに使う「距離」を、直線距離ではなくNav2の実パス長
        // (compute_path_to_pose)で評価するためのキャッシュ設定。候補ごとに
        // 毎サイクル問い合わせるとplanner_serverの負荷が大きいので、結果を
        // path_cache_ttl_sec の間キャッシュし、同時問い合わせ数も上限を設ける。
        declare_parameter("path_cost_scale",      0.25);
        declare_parameter("path_cache_ttl_sec",    5.0);
        declare_parameter("max_path_queries_per_cycle", 6);

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

        gap_params_.median_win         = static_cast<int>(get_parameter("gap_median_win").as_int());
        gap_params_.min_open_samples   = static_cast<int>(get_parameter("gap_min_open_samples").as_int());
        gap_params_.min_gap_width      = get_parameter("gap_min_width").as_double();
        gap_params_.target_push        = get_parameter("gap_target_push").as_double();
        gap_params_.abd_lambda_deg     = get_parameter("gap_abd_lambda_deg").as_double();
        gap_params_.abd_sigma          = get_parameter("gap_abd_sigma").as_double();
        gap_params_.min_jump           = get_parameter("gap_min_jump").as_double();
        gap_params_.min_support        = static_cast<int>(get_parameter("gap_min_support").as_int());
        gap_params_.support_open_edges = get_parameter("gap_support_open_edges").as_bool();
        gap_params_.jump_target_mode   =
            (get_parameter("gap_jump_target_mode").as_string() == "corner")
                ? gap_extraction::Params::JumpTargetMode::kCorner
                : gap_extraction::Params::JumpTargetMode::kBetween;
        gap_params_.jump_ratio         = get_parameter("gap_jump_ratio").as_double();
        gap_params_.corner_offset      = get_parameter("gap_corner_offset").as_double();
        gap_params_.max_open_depth_diff_m =
            get_parameter("gap_max_open_depth_diff_m").as_double();

        odom_frame_   = get_parameter("odom_frame").as_string();
        global_frame_ = get_parameter("global_frame").as_string();
        use_voronoi_  = get_parameter("use_voronoi_partition").as_bool();
        filter_explored_   = get_parameter("filter_explored_frontiers").as_bool();
        merged_map_topic_  = get_parameter("merged_map_topic").as_string();
        explored_obstacle_threshold_ =
            static_cast<int8_t>(get_parameter("explored_obstacle_threshold").as_int());
        progress_timeout_   = get_parameter("progress_timeout").as_double();
        blacklist_radius_   = get_parameter("blacklist_radius").as_double();
        blacklist_clear_sec_ = get_parameter("blacklist_clear_sec").as_double();
        path_cost_scale_    = get_parameter("path_cost_scale").as_double();
        path_cache_ttl_sec_ = get_parameter("path_cache_ttl_sec").as_double();
        max_path_queries_per_cycle_ =
            static_cast<int>(get_parameter("max_path_queries_per_cycle").as_int());

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
                latest_scan_ = *m; has_scan_ = true;
            });

        ns_ = get_namespace();
        robot_base_frame_ =
            (!ns_.empty() && ns_.front() == '/' ? ns_.substr(1) : ns_) + "/base_footprint";

        // Nav2 アクションクライアント（相対名なので名前空間下で <ns>/navigate_to_pose に解決）
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        // フロンティア選択のコストに使う実パス長の問い合わせ用
        // （<ns>/compute_path_to_pose、planner_server が提供）
        path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");

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

        // ボロノイ担当領域マスク（voronoi_partition_node が配信）。
        // 相対トピック名なので、名前空間下では自動的に <ns>/voronoi_mask に解決される。
        if (use_voronoi_) {
            voronoi_mask_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                "voronoi_mask",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { voronoi_mask_ = m; });
        }

        // 統合マップ（icp_map_matching_node が配信）。既に探査済み（自分 or 相手）の
        // 自由空間に落ちたフラッグを弾くために参照する。絶対トピック名なので
        // 名前空間に関係なく共有の /map を購読する。
        if (filter_explored_) {
            merged_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                merged_map_topic_,
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { merged_map_ = m; });
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
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

        last_blacklist_clear_ = this->now();

        RCLCPP_INFO(get_logger(),
            "sensor_mb_explore: safe=%.2f r=%.3f gap_w=%.2f progress_timeout=%.0f ns=%s",
            safe_, robot_r_, gap_params_.min_gap_width, progress_timeout_, ns_.c_str());
    }

private:
    double ang_, safe_, vfh_t_, v_deg_, emerg_, robot_r_;
    double dup_r_, dup_t_, frontier_ttl_, fcone_;
    gap_extraction::Params gap_params_;
    std::string odom_frame_;
    bool use_voronoi_;
    nav_msgs::msg::OccupancyGrid::SharedPtr voronoi_mask_;
    bool filter_explored_;
    std::string merged_map_topic_;
    int8_t explored_obstacle_threshold_;
    nav_msgs::msg::OccupancyGrid::SharedPtr merged_map_;
    std::string global_frame_;
    std::string robot_base_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

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
    // Nav2 goal lifecycle (ported from frontier_explore_node.cpp)

    enum class State { IDLE, MOVING };
    State state_ = State::IDLE;

    // 同じ(ように見える)ゴールがNav2に連続でrejectされた場合、進捗タイムアウト
    // (MOVING状態でしか働かない)には引っかからずMOVING↔IDLEを高速に往復して
    // しまう。N回連続rejectでブラックリストして抜け出す。
    static constexpr int kMaxGoalRejects = 3;

    double progress_timeout_;
    double blacklist_radius_;
    double blacklist_clear_sec_;

    rclcpp::Time last_progress_time_;
    rclcpp::Time last_blacklist_clear_;
    geometry_msgs::msg::PoseStamped current_goal_;
    uint64_t current_goal_seq_ = 0;  // cancel後の古いcallbackを無視するためのシーケンス番号
    int reject_goal_streak_ = 0;     // Nav2に連続でacceptされなかった回数

    // 到達失敗したフロンティアのブラックリスト(Nav2のreject/abort用。
    // frontier_store_の重複排除とは別の、ナビゲーション失敗の記憶)
    std::vector<std::pair<double, double>> nav_blacklist_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    // ------------------------------------------------------------------
    // 実パス長キャッシュ(目標選択のコストで直線距離の代わりに使う)

    double path_cost_scale_;
    double path_cache_ttl_sec_;
    int    max_path_queries_per_cycle_;

    struct PathCacheEntry { double length; rclcpp::Time t; };
    std::map<std::pair<long, long>, PathCacheEntry> path_cache_;
    std::set<std::pair<long, long>> path_pending_;

    rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;

    // frontier_store_の座標は連続値なので、0.1m格子に丸めてキャッシュキーに
    // する(わずかな再検出のたびに別候補として再問い合わせしないため)。
    static std::pair<long, long> path_key(double x, double y) {
        return {std::lround(x * 10.0), std::lround(y * 10.0)};
    }

    // 候補(fx,fy)へのロボットからの実パス長を返す。キャッシュが新しければ
    // それを返す。キャッシュがない/古い場合は非同期で問い合わせを開始しつつ
    // (多重発行はpath_pending_で防ぐ)、結果が届くまでの間は直線距離を暫定値
    // として返す。query_budget が尽きたら問い合わせ自体をスキップする
    // (frontier_store_が大きい時にplanner_serverへ殺到しないための上限)。
    double path_length_for(double fx, double fy, double robot_x, double robot_y, int& query_budget) {
        const double straight = std::hypot(fx - robot_x, fy - robot_y);
        const auto key = path_key(fx, fy);
        const auto it = path_cache_.find(key);
        if (it != path_cache_.end() &&
            (now() - it->second.t).seconds() < path_cache_ttl_sec_) {
            return it->second.length;
        }
        if (query_budget > 0 && path_pending_.find(key) == path_pending_.end() &&
            path_client_->action_server_is_ready()) {
            --query_budget;
            path_pending_.insert(key);

            auto goal = ComputePathToPose::Goal();
            goal.goal.header.frame_id = global_frame_;
            goal.goal.header.stamp    = now();
            goal.goal.pose.position.x = fx;
            goal.goal.pose.position.y = fy;
            goal.goal.pose.orientation.w = 1.0;
            goal.use_start = false;  // 現在のロボット位置を起点にする

            auto opts = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
            opts.goal_response_callback =
                [this, key](const GoalHandlePath::SharedPtr& gh) {
                    if (!gh) path_pending_.erase(key);  // rejectされたら次サイクルで再試行
                };
            opts.result_callback =
                [this, key](const GoalHandlePath::WrappedResult& res) {
                    path_pending_.erase(key);
                    if (res.code != rclcpp_action::ResultCode::SUCCEEDED) return;
                    const auto& poses = res.result->path.poses;
                    double len = 0.0;
                    for (size_t i = 1; i < poses.size(); ++i) {
                        len += std::hypot(
                            poses[i].pose.position.x - poses[i - 1].pose.position.x,
                            poses[i].pose.position.y - poses[i - 1].pose.position.y);
                    }
                    path_cache_[key] = {len, now()};
                };
            path_client_->async_send_goal(goal, opts);
        }
        return straight;  // 届くまでの暫定値
    }

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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  peer_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    frontier_peer_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  target_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr     voronoi_mask_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr     merged_map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr       frontier_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr share_timer_;
    rclcpp::TimerBase::SharedPtr target_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;

    // ------------------------------------------------------------------
    // World-frame pose (TF: global_frame_ -> robot_base_frame_)
    //
    // Uses TF rather than the raw /odom topic: /odom is dead-reckoning
    // relative to each robot's own spawn point, not the shared "map" frame.
    // Everything below (frontier_store_, the /shared_* topics consumed by
    // peers, and the Voronoi territory check) compares positions across
    // robots or against the merged map, so it needs a common frame.

    bool update_world_pose() {
        try {
            const auto tf = tf_buffer_.lookupTransform(
                global_frame_, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
            const auto& t = tf.transform.translation;
            const auto& q = tf.transform.rotation;
            pose_.x   = t.x;
            pose_.y   = t.y;
            pose_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            has_odom_ = true;

            const double t_now  = now().seconds();
            const double cutoff = t_now - 600.0;
            history_.push_back({pose_.x, pose_.y, t_now});
            history_.erase(
                std::remove_if(history_.begin(), history_.end(),
                    [cutoff](const OdomPt& p) { return p.t < cutoff; }),
                history_.end());
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "TF %s -> %s unavailable: %s",
                global_frame_.c_str(), robot_base_frame_.c_str(), e.what());
            has_odom_ = false;
            return false;
        }
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

    // 統合マップ上で、この位置が既に探査済みの自由空間かどうか。
    // 範囲外・未観測(-1)・障害物は「確認できない」側に倒して false
    // (=除外しない)とする。統合マップにはICP統合のラグがあるため、
    // 「まだ反映されていないだけの新規領域」を誤って捨てないための配慮。
    // 逆に自由空間だと確認できた場合だけ、自分 or 相手が既に見た場所として弾く。
    bool is_map_explored(double wx, double wy) const {
        if (!filter_explored_ || !merged_map_) return false;
        const auto& info = merged_map_->info;
        if (info.width == 0 || info.height == 0 || info.resolution <= 0.0f) return false;
        const int gx = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
        const int gy = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
        if (gx < 0 || gx >= static_cast<int>(info.width) ||
            gy < 0 || gy >= static_cast<int>(info.height))
            return false;
        const size_t idx = static_cast<size_t>(gy) * info.width + static_cast<size_t>(gx);
        if (idx >= merged_map_->data.size()) return false;
        const int8_t v = merged_map_->data[idx];
        return v >= 0 && v < explored_obstacle_threshold_;
    }

    void add_frontier(double wx, double wy, double width = 1.0) {
        if (is_visited(wx, wy)) return;
        if (is_map_explored(wx, wy)) return;
        for (const auto& f : frontier_store_)
            if (std::hypot(wx - f.x, wy - f.y) < dup_r_ * 0.5) return;
        frontier_store_.push_back({wx, wy, now().seconds(), width});
    }

    bool is_blacklisted(double cx, double cy) const {
        for (const auto& [bx, by] : nav_blacklist_)
            if (std::hypot(cx - bx, cy - by) < blacklist_radius_) return true;
        return false;
    }

    // respect_territory=true のときのみボロノイセル外を弾く。マスク未受信、
    // または use_voronoi_=false のときは常に true（制約なし）を返す。
    bool in_own_territory(double wx, double wy, bool respect_territory) const {
        if (!respect_territory || !use_voronoi_) return true;
        if (!voronoi_mask_) return false;
        return voronoi::in_own_territory(*voronoi_mask_, wx, wy);
    }

    void prune_frontiers() {
        const double t_now = now().seconds();
        frontier_store_.erase(
            std::remove_if(frontier_store_.begin(), frontier_store_.end(),
                [&](const Frontier& f) {
                    return (t_now - f.t > frontier_ttl_) || is_visited(f.x, f.y) ||
                           is_map_explored(f.x, f.y);
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
    //
    // has_target_/target_x_/target_y_ double as both "what we last told
    // peers we're going for" (published here) and "what Nav2 goal is
    // currently active" (set/cleared in send_nav_goal()/cancel_current_goal()
    // and the goal_response_callback/result_callback below).

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
    // Nav2 goal send / cancel (ported from frontier_explore_node.cpp)

    void send_nav_goal(double x, double y) {
        if (!nav_client_->action_server_is_ready()) {
            RCLCPP_WARN(get_logger(), "navigate_to_pose action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = global_frame_;
        goal_msg.pose.header.stamp    = now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;
        current_goal_ = goal_msg.pose;

        // シーケンス番号をインクリメント: 古いゴールのcallbackを無視するため
        const uint64_t seq = ++current_goal_seq_;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        opts.goal_response_callback =
            [this, seq](const GoalHandleNav::SharedPtr& gh) {
                if (seq != current_goal_seq_) return;  // 古いゴールのcallbackを無視
                if (!gh) {
                    ++reject_goal_streak_;
                    RCLCPP_WARN(get_logger(),
                        "Goal rejected by Nav2 (%d/%d) at (%.2f, %.2f)",
                        reject_goal_streak_, kMaxGoalRejects,
                        current_goal_.pose.position.x, current_goal_.pose.position.y);
                    // rejectはaction acceptレベルの即時拒否なのでMOVING状態を経由せず、
                    // progress_timeoutの監視対象にならない。放置すると同じ目標を
                    // 毎ループ再送し続けて全く動かなくなるため、ここで自前に
                    // ブラックリスト行きにして次のフロンティアへ逃がす。
                    if (reject_goal_streak_ >= kMaxGoalRejects) {
                        RCLCPP_WARN(get_logger(),
                            "Repeated rejection -> blacklisting (%.2f, %.2f)",
                            current_goal_.pose.position.x, current_goal_.pose.position.y);
                        nav_blacklist_.emplace_back(current_goal_.pose.position.x,
                                                    current_goal_.pose.position.y);
                        reject_goal_streak_ = 0;
                    }
                    state_ = State::IDLE;
                    has_target_ = false;
                } else {
                    reject_goal_streak_ = 0;
                    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2");
                }
            };

        opts.result_callback =
            [this, seq](const GoalHandleNav::WrappedResult& res) {
                if (seq != current_goal_seq_) return;  // cancel後の古いcallbackを無視
                switch (res.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Reached target frontier!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(get_logger(),
                            "Navigation aborted -> blacklisting (%.2f, %.2f)",
                            current_goal_.pose.position.x,
                            current_goal_.pose.position.y);
                        nav_blacklist_.emplace_back(current_goal_.pose.position.x,
                                                    current_goal_.pose.position.y);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(get_logger(), "Navigation canceled");
                        break;
                    default:
                        break;
                }
                state_ = State::IDLE;
                has_target_ = false;
            };

        nav_client_->async_send_goal(goal_msg, opts);
        state_ = State::MOVING;
        last_progress_time_ = now();
        target_x_ = x; target_y_ = y; has_target_ = true;

        RCLCPP_INFO(get_logger(), "Navigating to frontier (%.2f, %.2f)", x, y);
    }

    void cancel_current_goal() {
        if (state_ == State::MOVING) {
            nav_client_->async_cancel_all_goals();
            state_ = State::IDLE;
            has_target_ = false;
        }
    }

    // ------------------------------------------------------------------
    // Control loop (5 Hz)
    //
    // Only decides WHERE to go (gap-based frontier selection, same as
    // sensor_explore_node) and hands that off to Nav2 via send_nav_goal().
    // It never publishes cmd_vel: local obstacle avoidance is entirely
    // Nav2's controller_server + costmap, same as frontier_explore_node.

    void control_loop() {
        if (!has_scan_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for LiDAR data...");
            return;
        }

        // ブラックリストの定期クリア
        if ((now() - last_blacklist_clear_).seconds() > blacklist_clear_sec_) {
            if (!nav_blacklist_.empty()) {
                RCLCPP_INFO(get_logger(), "Clearing blacklist (%zu entries)", nav_blacklist_.size());
                nav_blacklist_.clear();
            }
            last_blacklist_clear_ = now();
        }

        update_world_pose();  // failure leaves has_odom_=false

        // gap_targets（フロンティア抽出）はロボット座標系基準なので、位置がなくても
        // 計算自体はできるが、ワールド座標への変換・ストアへの追加にはTFが要る。
        const auto ps = sensor_proc::process(
            latest_scan_, safe_, vfh_t_, v_deg_, emerg_, ang_,
            robot_r_, fcone_, gap_params_);

        if (!has_odom_) return;

        for (const auto& [a, d, w] : ps.gap_targets) {
            const double wa = pose_.yaw + a;
            add_frontier(pose_.x + d * std::cos(wa), pose_.y + d * std::sin(wa), w);
        }
        prune_frontiers();

        // 進捗タイムアウト監視
        if (state_ == State::MOVING) {
            const double elapsed = (now() - last_progress_time_).seconds();
            if (elapsed > progress_timeout_) {
                RCLCPP_WARN(get_logger(), "Progress timeout (%.1fs) -> cancel and replan", elapsed);
                nav_blacklist_.emplace_back(current_goal_.pose.position.x,
                                            current_goal_.pose.position.y);
                cancel_current_goal();
            }
        }

        // ─── チャタリング防止: 現在向かっている目標がまだ有効なら維持する ───
        if (state_ == State::MOVING) {
            const double gx = current_goal_.pose.position.x;
            const double gy = current_goal_.pose.position.y;
            const bool still_in_store = std::any_of(frontier_store_.begin(), frontier_store_.end(),
                [&](const Frontier& f) { return std::hypot(f.x - gx, f.y - gy) < 1.0; });
            const bool still_valid = still_in_store && !is_visited(gx, gy) &&
                !is_map_explored(gx, gy) && in_own_territory(gx, gy, use_voronoi_);
            if (still_valid) return;
            RCLCPP_INFO(get_logger(),
                "Current target vanished, claimed, explored, or left own territory. Replanning...");
            cancel_current_goal();
        }

        // ベストなフロンティアを選択
        // cost = |bearing| + path_cost_scale*実パス長 − 0.3*width + peer_penalty
        // 前方・近距離(直線でなくNav2の実パス長)・広い開口を優先し、
        // 相手ロボットの近くは減点する。
        double best_cost = std::numeric_limits<double>::max();
        double best_fx = 0.0, best_fy = 0.0;
        bool   found = false;

        // Voronoi有効時は自分の担当範囲だけを候補にする。
        // マスク未受信・自領域に候補なしなら次の更新まで待つ。
        const bool respect_territory = use_voronoi_;
        int query_budget = max_path_queries_per_cycle_;
        for (const auto& f : frontier_store_) {
            if (is_visited(f.x, f.y)) continue;
            if (is_blacklisted(f.x, f.y)) continue;
            if (!in_own_territory(f.x, f.y, respect_territory)) continue;

            const double dx   = f.x - pose_.x;
            const double dy   = f.y - pose_.y;
            const double raw  = std::atan2(dy, dx) - pose_.yaw;
            const double ang  = std::atan2(std::sin(raw), std::cos(raw));
            const double dist = path_length_for(f.x, f.y, pose_.x, pose_.y, query_budget);
            double cost = std::abs(ang) + path_cost_scale_ * dist - 0.3 * f.width;

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
                best_cost = cost;
                best_fx   = f.x;
                best_fy   = f.y;
                found     = true;
            }
        }

        if (!found) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "No reachable frontier (store=%zu, blacklist=%zu)",
                frontier_store_.size(), nav_blacklist_.size());
            return;
        }

        send_nav_goal(best_fx, best_fy);
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
    rclcpp::spin(std::make_shared<SensorMbExploreNode>());
    rclcpp::shutdown();
}
