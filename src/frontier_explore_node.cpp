#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "multi_explore_mapping/voronoi_partition.hpp"

#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <algorithm>
#include <random>
#include <map>
#include <string>

class FrontierExplorerNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    FrontierExplorerNode()
    : Node("frontier_explorer"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // パラメータ宣言と初期化
        this->declare_parameter<std::string>("robot_base_frame", "base_link");
        this->declare_parameter<std::string>("global_frame", "map");
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<double>("planner_frequency", 1.0);
        this->declare_parameter<double>("progress_timeout", 30.0);
        this->declare_parameter<double>("min_frontier_size", 0.3);
        this->declare_parameter<double>("potential_scale", 1.0);
        this->declare_parameter<double>("gain_scale", 3.0);
        this->declare_parameter<double>("backward_penalty_scale", 1.0);
        this->declare_parameter<bool>("visualize", true);
        this->declare_parameter<double>("blacklist_radius", 1.0);
        this->declare_parameter<double>("blacklist_clear_sec", 60.0);
        this->declare_parameter<bool>("use_voronoi_partition", true);
        this->declare_parameter<std::vector<std::string>>("peer_namespaces", std::vector<std::string>{});
        this->declare_parameter<double>("peer_claim_radius", 1.0);

        robot_base_frame_  = this->get_parameter("robot_base_frame").as_string();
        global_frame_      = this->get_parameter("global_frame").as_string();
        planner_frequency_ = this->get_parameter("planner_frequency").as_double();
        progress_timeout_  = this->get_parameter("progress_timeout").as_double();
        min_frontier_size_ = this->get_parameter("min_frontier_size").as_double();
        potential_scale_   = this->get_parameter("potential_scale").as_double();
        gain_scale_        = this->get_parameter("gain_scale").as_double();
        backward_penalty_scale_ = this->get_parameter("backward_penalty_scale").as_double();
        visualize_         = this->get_parameter("visualize").as_bool();
        blacklist_radius_  = this->get_parameter("blacklist_radius").as_double();
        blacklist_clear_sec_ = this->get_parameter("blacklist_clear_sec").as_double();
        use_voronoi_        = this->get_parameter("use_voronoi_partition").as_bool();
        peer_namespaces_    = this->get_parameter("peer_namespaces").as_string_array();
        peer_claim_radius_  = this->get_parameter("peer_claim_radius").as_double();
        std::string map_topic = this->get_parameter("map_topic").as_string();

        // 統合コストマップをサブスクライブ
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1)
        );

        // ボロノイ担当領域マスク（voronoi_partition_node が配信）。
        // 相対トピック名なので、名前空間 robot_i 下では自動的に
        // /robot_i/voronoi_mask に解決される。
        if (use_voronoi_) {
            voronoi_mask_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "voronoi_mask",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { voronoi_mask_ = msg; }
            );
        }

        if (visualize_) {
            frontier_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "frontiers", 10);
        }

        // 相手ロボットへ自分の現在のナビゲーション目標を配信する。相手はこれを
        // 見て、同じ/近いフロンティアを選ばないようにする（ボロノイの境界共有
        // バッファ帯やフォールバックだけでは、境界付近で両者が同じ場所を最適解
        // と判断してしまう競合を防げないため）。
        target_claim_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "current_target", rclcpp::QoS(1).transient_local().reliable());

        // 相手ロボットの現在目標を購読し、自分のフロンティア選択から除外する。
        for (const auto& peer_ns : peer_namespaces_) {
            auto sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/" + peer_ns + "/current_target",
                rclcpp::QoS(1).transient_local().reliable(),
                [this, peer_ns](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                    if (msg->header.frame_id.empty()) {
                        peer_targets_.erase(peer_ns);
                    } else {
                        peer_targets_[peer_ns] = *msg;
                    }
                });
            peer_target_subs_.push_back(sub);
        }

        // Nav2 アクションクライアントの生成
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // メイン制御ループタイマー
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / planner_frequency_),
            std::bind(&FrontierExplorerNode::exploration_loop, this)
        );

        last_progress_time_ = this->now();
        last_blacklist_clear_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "FrontierExplorer ready | base=%s global=%s gain=%.1f potential=%.1f "
            "backward_penalty=%.1f min_size=%.2fm",
            robot_base_frame_.c_str(), global_frame_.c_str(),
            gain_scale_, potential_scale_, backward_penalty_scale_, min_frontier_size_);
    }

private:
    struct Cell { int x, y; };

    struct Frontier {
        std::vector<Cell> cells;
        double centroid_x;
        double centroid_y;
        double size;
        double score;
    };

    enum class State { IDLE, MOVING };
    State state_ = State::IDLE;

    // 同じ(ように見える)ゴールがNav2に連続でrejectされた場合、進捗タイムアウト
    // (MOVING状態でしか働かない)には引っかからずMOVING↔IDLEを高速に往復して
    // しまう。N回連続rejectでブラックリストして抜け出す。
    static constexpr int kMaxGoalRejects = 3;

    std::string robot_base_frame_;
    std::string global_frame_;
    double planner_frequency_;
    double progress_timeout_;
    double min_frontier_size_;
    double potential_scale_;
    double gain_scale_;
    double backward_penalty_scale_;
    bool visualize_;
    double blacklist_radius_;
    double blacklist_clear_sec_;
    bool use_voronoi_;
    std::vector<std::string> peer_namespaces_;
    double peer_claim_radius_;

    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    bool map_updated_ = false;
    nav_msgs::msg::OccupancyGrid::SharedPtr voronoi_mask_;

    rclcpp::Time last_progress_time_;
    rclcpp::Time last_blacklist_clear_;
    geometry_msgs::msg::PoseStamped current_goal_;
    std::shared_ptr<GoalHandleNav> current_goal_handle_;
    uint64_t current_goal_seq_ = 0;  // cancel後の古いcallbackを無視するためのシーケンス番号
    int reject_goal_streak_ = 0;     // Nav2に連続でacceptされなかった回数

    // 到達失敗したフロンティアのブラックリスト
    std::vector<std::pair<double,double>> blacklist_;

    // 相手ロボットの現在のナビゲーション目標（namespace -> 最新の受信値）。
    // frame_idが空のメッセージを受け取ったキーは即座にeraseされるので、
    // このmapに存在するエントリは「相手が今まさに向かっている場所」を表す。
    std::map<std::string, geometry_msgs::msg::PointStamped> peer_targets_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr voronoi_mask_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_claim_pub_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr> peer_target_subs_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = msg;
        map_updated_ = true;
    }

    // ----------------------------------------------------------------
    // フロンティアセル検出 (8近傍・有符号int8_t対応版)
    // ----------------------------------------------------------------
    std::vector<Cell> detect_frontier_cells(const nav_msgs::msg::OccupancyGrid &map)
    {
        std::vector<Cell> frontier_cells;
        int W = static_cast<int>(map.info.width);
        int H = static_cast<int>(map.info.height);

        // 8近傍オフセット
        const int dx[] = {1,-1, 0, 0, 1, 1,-1,-1};
        const int dy[] = {0, 0, 1,-1, 1,-1, 1,-1};

        for (int y = 1; y < H - 1; y++) {
            for (int x = 1; x < W - 1; x++) {
                int8_t raw_val = map.data[y * W + x];
        
                // コストマップ仕様: -1は未知(NO_INFORMATION), 100以上は障害物または危険領域
                if (raw_val == -1 || raw_val >= 100) continue; 

                bool has_unknown = false;
                for (int d = 0; d < 8; d++) {
                    int nx = x + dx[d], ny = y + dy[d];
                    if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;

                    // 斜め方向の隣接は、両脇の直交セル(水平側・垂直側)の
                    // どちらかが障害物なら壁越しの「角抜け」とみなしてスキップ
                    // する。8近傍の対角判定は物理的な遮蔽を考慮しないため、
                    // 壁のすぐ外側のセルが壁の内側の未知領域と斜めにだけ接して
                    // いる場合でも本来は繋がっていない。これを弾かないと、壁が
                    // グリッドに対して斜めのときにできる階段状のギザギザ角
                    // ほぼ全てが誤ってフロンティア判定されてしまう。
                    if (dx[d] != 0 && dy[d] != 0) {
                        int8_t flank_h = map.data[y * W + (x + dx[d])];
                        int8_t flank_v = map.data[(y + dy[d]) * W + x];
                        if (flank_h >= 100 || flank_v >= 100) continue;
                    }

                    // 隣接セルに「未知（-1）」があるセルを境界線（フロンティア）とする
                    if (map.data[ny * W + nx] == -1) {
                        has_unknown = true;
                        break;
                    }
                }
                if (has_unknown) frontier_cells.push_back({x, y});
            }
        }
        return frontier_cells;
    }

    // ----------------------------------------------------------------
    // 到達可能フィルタ: ロボット位置から既知の自由セルだけをたどって
    // 到達できるフロンティアセルのみ残す。壁の向こう側の未知領域に接する
    // だけのフロンティア(経路が存在しない)を選択前に除外する。
    // 斜め移動は両脇の直交セルが通行可能な場合のみ許可(壁の角抜け防止)。
    // ----------------------------------------------------------------
    std::vector<Cell> filter_reachable_cells(
        const std::vector<Cell> &cells,
        const nav_msgs::msg::OccupancyGrid &map,
        double robot_x, double robot_y)
    {
        const int W = static_cast<int>(map.info.width);
        const int H = static_cast<int>(map.info.height);
        const double res = map.info.resolution;
        const double ox = map.info.origin.position.x;
        const double oy = map.info.origin.position.y;

        auto passable = [&](int x, int y) {
            if (x < 0 || x >= W || y < 0 || y >= H) return false;
            const int8_t v = map.data[y * W + x];
            // 99 = Nav2のinscribed(壁からロボット半径以内=中心が入れない)。
            // ここまで通行不可にすることで、壁の1セル欠けや壁際の高コスト帯
            // 経由で壁の向こう側へ漏れる経路を塞ぐ(Nav2プランナーの通行判定と同じ)。
            return v != -1 && v < 99;
        };

        const int rx = static_cast<int>(std::floor((robot_x - ox) / res));
        const int ry = static_cast<int>(std::floor((robot_y - oy) / res));

        // ロボットのセルが障害物/範囲外にかかっている場合は、近傍の通行可能セルを起点にする
        int sx = -1, sy = -1;
        const int kSearch = 10;
        int best_d2 = std::numeric_limits<int>::max();
        for (int dy = -kSearch; dy <= kSearch; ++dy) {
            for (int dx = -kSearch; dx <= kSearch; ++dx) {
                const int d2 = dx * dx + dy * dy;
                if (d2 < best_d2 && passable(rx + dx, ry + dy)) {
                    best_d2 = d2; sx = rx + dx; sy = ry + dy;
                }
            }
        }
        if (sx < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Robot is not on/near a known free cell; skipping reachability filter");
            return cells;
        }

        std::vector<bool> reach(static_cast<size_t>(W) * H, false);
        std::queue<Cell> q;
        reach[sy * W + sx] = true;
        q.push({sx, sy});

        const int dxs[] = {1,-1, 0, 0, 1, 1,-1,-1};
        const int dys[] = {0, 0, 1,-1, 1,-1, 1,-1};
        while (!q.empty()) {
            Cell c = q.front(); q.pop();
            for (int d = 0; d < 8; ++d) {
                const int nx = c.x + dxs[d], ny = c.y + dys[d];
                if (!passable(nx, ny) || reach[ny * W + nx]) continue;
                if (dxs[d] != 0 && dys[d] != 0 &&
                    (!passable(c.x + dxs[d], c.y) || !passable(c.x, c.y + dys[d])))
                    continue;
                reach[ny * W + nx] = true;
                q.push({nx, ny});
            }
        }

        std::vector<Cell> out;
        out.reserve(cells.size());
        for (const auto &c : cells)
            if (reach[c.y * W + c.x]) out.push_back(c);
        return out;
    }

    // ----------------------------------------------------------------
    // BFS クラスタリング (8近傍)
    // ----------------------------------------------------------------
    std::vector<Frontier> cluster_frontiers(
        const std::vector<Cell> &frontier_cells,
        const nav_msgs::msg::OccupancyGrid &map)
    {
        int W   = static_cast<int>(map.info.width);
        int H   = static_cast<int>(map.info.height);
        double res = map.info.resolution;
        double ox  = map.info.origin.position.x;
        double oy  = map.info.origin.position.y;

        std::vector<bool> is_frontier(W * H, false);
        for (const auto &c : frontier_cells)
            is_frontier[c.y * W + c.x] = true;

        std::vector<bool> visited(W * H, false);
        std::vector<Frontier> clusters;

        const int dx[] = {1,-1, 0, 0, 1, 1,-1,-1};
        const int dy[] = {0, 0, 1,-1, 1,-1, 1,-1};

        for (const auto &start : frontier_cells) {
            int idx = start.y * W + start.x;
            if (visited[idx]) continue;

            Frontier cluster;
            std::queue<Cell> bfs;
            bfs.push(start);
            visited[idx] = true;

            while (!bfs.empty()) {
                Cell cur = bfs.front(); bfs.pop();
                cluster.cells.push_back(cur);

                for (int d = 0; d < 8; d++) {
                    int nx = cur.x + dx[d], ny = cur.y + dy[d];
                    if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                    int nidx = ny * W + nx;
                    if (!visited[nidx] && is_frontier[nidx]) {
                        visited[nidx] = true;
                        bfs.push({nx, ny});
                    }
                }
            }

            double sum_x = 0.0, sum_y = 0.0;
            for (const auto &c : cluster.cells) {
                sum_x += ox + (c.x + 0.5) * res;
                sum_y += oy + (c.y + 0.5) * res;
            }
            double n = static_cast<double>(cluster.cells.size());
            double mean_x = sum_x / n;
            double mean_y = sum_y / n;

            // 幾何重心は凹形状クラスタ(角を回り込む・壁沿いに湾曲する等)だと
            // クラスタのどのセルとも一致せず、未知領域や障害物寄りに落ちて
            // 非到達なナビゲーション目標になりうる。重心に最も近い実際の
            // フロンティアセルにスナップすることで、必ず既知かつ走行可能な
            // セルをゴールにする。
            double best_d2 = std::numeric_limits<double>::infinity();
            double snap_x = mean_x, snap_y = mean_y;
            for (const auto &c : cluster.cells) {
                double wx = ox + (c.x + 0.5) * res;
                double wy = oy + (c.y + 0.5) * res;
                double d2 = (wx - mean_x) * (wx - mean_x) + (wy - mean_y) * (wy - mean_y);
                if (d2 < best_d2) { best_d2 = d2; snap_x = wx; snap_y = wy; }
            }

            cluster.centroid_x = snap_x;
            cluster.centroid_y = snap_y;
            cluster.size       = n * res;
            cluster.score      = 0.0;
            clusters.push_back(cluster);
        }
        return clusters;
    }

    bool is_blacklisted(double cx, double cy) const {
        for (const auto &[bx, by] : blacklist_) {
            if (std::hypot(cx - bx, cy - by) < blacklist_radius_) return true;
        }
        return false;
    }

    // 相手ロボットが今まさに向かっている場所の近くかどうか。ボロノイの境界
    // 共有バッファ帯やフォールバックだけでは境界付近の同時選択を防げないため、
    // 領域制約とは独立に常にチェックする。
    bool is_peer_claimed(double cx, double cy) const {
        for (const auto &[peer_ns, pt] : peer_targets_) {
            (void)peer_ns;
            if (std::hypot(cx - pt.point.x, cy - pt.point.y) < peer_claim_radius_) return true;
        }
        return false;
    }

    // respect_territory=true のときは、自分のボロノイセル外
    // （voronoi_mask の値が 50/100 でない候補）を選択肢から除外する。
    Frontier* select_best_frontier(std::vector<Frontier> &frontiers,
                                   double robot_x, double robot_y, double robot_yaw,
                                   bool respect_territory) {
        Frontier *best = nullptr;
        double best_score = -std::numeric_limits<double>::infinity();

        for (auto &f : frontiers) {
            if (f.size < min_frontier_size_) continue;
            if (is_blacklisted(f.centroid_x, f.centroid_y)) continue;
            if (is_peer_claimed(f.centroid_x, f.centroid_y)) continue;
            if (respect_territory && voronoi_mask_ &&
                !voronoi::in_own_territory(*voronoi_mask_, f.centroid_x, f.centroid_y))
                continue;

            // 距離の計算（安全のため最小値を0.1mに制限）
            double dist = std::max(0.1, std::hypot(f.centroid_x - robot_x, f.centroid_y - robot_y));

            // 既存の利得・距離評価は保ち、進行方向に対して後方の候補だけ減点する。
            // 横〜前方の候補には影響させず、真後ろほど減点を大きくする。
            const double bearing = std::atan2(f.centroid_y - robot_y,
                                              f.centroid_x - robot_x);
            const double heading_error = std::abs(std::atan2(
                std::sin(bearing - robot_yaw), std::cos(bearing - robot_yaw)));
            constexpr double kHalfPi = 1.5707963267948966;
            const double backward_ratio = std::max(0.0,
                (heading_error - kHalfPi) / kHalfPi);
            const double backward_penalty = backward_penalty_scale_ * backward_ratio;

            // sizeをそのまま使うと外壁など巨大フロンティアが距離ペナルティを圧倒する。
            // log(1+size) でスケールを抑制し、近くの中型フロンティアも競争できるようにする。
            f.score = (gain_scale_ * std::log(1.0 + f.size)) -
                      (potential_scale_ * dist) - backward_penalty;

            if (f.score > best_score) {
                best_score = f.score;
                best       = &f;
            }
        }
        return best;
    }

    // ボロノイ制約を有効にした場合は、マスクが届いていない間も含めて
    // 自分の担当領域内だけから選ぶ。担当領域に候補がなければ待つ。
    Frontier* select_frontier_in_territory(std::vector<Frontier> &frontiers,
                                           double robot_x, double robot_y, double robot_yaw) {
        if (!use_voronoi_)
            return select_best_frontier(frontiers, robot_x, robot_y, robot_yaw,
                                        /*respect_territory=*/false);
        if (!voronoi_mask_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for Voronoi mask; refusing to select an unassigned frontier");
            return nullptr;
        }
        return select_best_frontier(frontiers, robot_x, robot_y, robot_yaw,
                                    /*respect_territory=*/true);
    }

    bool get_robot_pose(double &x, double &y, double &yaw) {
        try {
            auto tf = tf_buffer_.lookupTransform(
                global_frame_, robot_base_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(0.5));
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            const auto &q = tf.transform.rotation;
            yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            return true;
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "TF lookup failed: %s -> %s | %s", global_frame_.c_str(), robot_base_frame_.c_str(), e.what());
            return false;
        }
    }

    // 自分の現在目標を相手ロボットへ配信する。frame_idを空にしたメッセージが
    // 「目標なし（解除）」の合図。
    void publish_target_claim(double x, double y) {
        geometry_msgs::msg::PointStamped msg;
        msg.header.frame_id = global_frame_;
        msg.header.stamp    = this->now();
        msg.point.x = x;
        msg.point.y = y;
        target_claim_pub_->publish(msg);
    }

    void clear_target_claim() {
        geometry_msgs::msg::PointStamped msg;
        msg.header.frame_id = "";
        msg.header.stamp    = this->now();
        target_claim_pub_->publish(msg);
    }

    void send_nav_goal(double x, double y) {
        if (!nav_client_->action_server_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = global_frame_;
        goal_msg.pose.header.stamp    = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;
        current_goal_ = goal_msg.pose;

        // シーケンス番号をインクリメント: 古いゴールのcallbackを無視するため
        uint64_t seq = ++current_goal_seq_;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        opts.goal_response_callback =
            [this, seq](const GoalHandleNav::SharedPtr &gh) {
                if (seq != current_goal_seq_) return;  // 古いゴールのcallbackを無視
                if (!gh) {
                    ++reject_goal_streak_;
                    RCLCPP_WARN(this->get_logger(),
                        "Goal rejected by Nav2 (%d/%d) at (%.2f, %.2f)",
                        reject_goal_streak_, kMaxGoalRejects,
                        current_goal_.pose.position.x, current_goal_.pose.position.y);
                    // rejectはaction acceptレベルの即時拒否なのでMOVING状態を経由せず、
                    // progress_timeoutの監視対象にならない。放置すると同じ目標を
                    // 毎ループ再送し続けて全く動かなくなるため、ここで自前に
                    // ブラックリスト行きにして次のフロンティアへ逃がす。
                    if (reject_goal_streak_ >= kMaxGoalRejects) {
                        RCLCPP_WARN(this->get_logger(),
                            "Repeated rejection → blacklisting (%.2f, %.2f)",
                            current_goal_.pose.position.x, current_goal_.pose.position.y);
                        blacklist_.emplace_back(current_goal_.pose.position.x,
                                                current_goal_.pose.position.y);
                        reject_goal_streak_ = 0;
                    }
                    state_ = State::IDLE;
                    clear_target_claim();
                } else {
                    reject_goal_streak_ = 0;
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2");
                }
            };

        opts.result_callback =
            [this, seq](const GoalHandleNav::WrappedResult &res) {
                if (seq != current_goal_seq_) {
                    // cancel後に新ゴールが送信済みのため、このcallbackは無視する
                    RCLCPP_DEBUG(this->get_logger(), "Stale result callback ignored (seq %lu != %lu)",
                        seq, current_goal_seq_);
                    return;
                }
                switch (res.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Reached target frontier!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(),
                            "Navigation aborted → blacklisting (%.2f, %.2f)",
                            current_goal_.pose.position.x,
                            current_goal_.pose.position.y);
                        blacklist_.emplace_back(current_goal_.pose.position.x,
                                                current_goal_.pose.position.y);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "Navigation canceled");
                        break;
                    default:
                        break;
                }
                state_ = State::IDLE;
                clear_target_claim();
            };

        nav_client_->async_send_goal(goal_msg, opts);
        state_ = State::MOVING;
        last_progress_time_ = this->now();
        publish_target_claim(x, y);

        RCLCPP_INFO(this->get_logger(), "Navigating to frontier (%.2f, %.2f)", x, y);
    }

    void cancel_current_goal() {
        if (state_ == State::MOVING) {
            nav_client_->async_cancel_all_goals();
            state_ = State::IDLE;
            clear_target_claim();
        }
    }

    void publish_frontiers(const std::vector<Frontier> &frontiers,
                           const Frontier *selected)
    {
        if (!visualize_ || !frontier_publisher_) return;

        visualization_msgs::msg::MarkerArray arr;
        int id = 0;

        // ブラックリスト（赤）
        for (const auto &[bx, by] : blacklist_) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = global_frame_;
            m.header.stamp    = this->now();
            m.ns = "blacklist"; m.id = id++;
            m.type   = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = bx; m.pose.position.y = by;
            m.pose.position.z = 0.1; m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = blacklist_radius_ * 2.0;
            m.color.r = 1.0; m.color.a = 0.3;
            m.lifetime = rclcpp::Duration::from_seconds(2.0 / planner_frequency_);
            arr.markers.push_back(m);
        }

        // マップから判別されたフロンティア候補（sensor_explore_nodeの
        // フラッグ表示と同じ構造: 1つのSPHERE_LISTにまとめ、選択中の目標は
        // 混ぜずに別マーカー(target)として分離する）
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = global_frame_;
            m.header.stamp    = this->now();
            m.ns   = "detected_frontiers";
            m.id   = id++;
            m.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.15;
            m.lifetime = rclcpp::Duration::from_seconds(2.0 / planner_frequency_);

            for (const auto &f : frontiers) {
                if (is_blacklisted(f.centroid_x, f.centroid_y)) continue;  // 赤丸で既に表現済み

                geometry_msgs::msg::Point pt;
                pt.x = f.centroid_x; pt.y = f.centroid_y; pt.z = 0.1;
                m.points.push_back(pt);

                visualization_msgs::msg::Marker::_color_type c;
                if (f.size >= min_frontier_size_) {
                    c.r = 0.0; c.g = 0.5; c.b = 1.0; c.a = 0.8;  // 有効（青）
                } else {
                    c.r = 0.5; c.g = 0.5; c.b = 0.5; c.a = 0.3;  // サイズ未満（灰）
                }
                m.colors.push_back(c);
            }
            if (!m.points.empty()) arr.markers.push_back(m);
        }

        // 選択された目標（sensor_explore_nodeのマゼンタtargetと同じ表現）
        if (selected) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = global_frame_;
            m.header.stamp    = this->now();
            m.ns   = "target";
            m.id   = id++;
            m.type   = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = selected->centroid_x;
            m.pose.position.y = selected->centroid_y;
            m.pose.position.z = 0.2;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.25;
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0;
            m.lifetime = rclcpp::Duration::from_seconds(2.0 / planner_frequency_);
            arr.markers.push_back(m);
        }

        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.insert(arr.markers.begin(), del);
        frontier_publisher_->publish(arr);
    }

    // ----------------------------------------------------------------
    // メイン制御ループ
    // ----------------------------------------------------------------
    void exploration_loop() {
        // ブラックリストの定期クリア処理
        if ((this->now() - last_blacklist_clear_).seconds() > blacklist_clear_sec_) {
            if (!blacklist_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Clearing blacklist (%zu entries)", blacklist_.size());
                blacklist_.clear();
            }
            last_blacklist_clear_ = this->now();
        }

        if (!current_map_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for map...");
            return;
        }

        if (!tf_buffer_.canTransform(global_frame_, robot_base_frame_, tf2::TimePointZero)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for TF: %s -> %s", global_frame_.c_str(), robot_base_frame_.c_str());
            return;
        }

        double robot_x, robot_y, robot_yaw;
        if (!get_robot_pose(robot_x, robot_y, robot_yaw)) return;

        // 進捗タイムアウト監視
        if (state_ == State::MOVING) {
            double elapsed = (this->now() - last_progress_time_).seconds();
            if (elapsed > progress_timeout_) {
                RCLCPP_WARN(this->get_logger(), "Progress timeout (%.1fs) → cancel and replan", elapsed);
                blacklist_.emplace_back(current_goal_.pose.position.x, current_goal_.pose.position.y);
                cancel_current_goal();
            }
            if (!map_updated_) return;
        }
        map_updated_ = false;

        // フロンティア検出とクラスタリング
        auto frontier_cells = detect_frontier_cells(*current_map_);
        const size_t detected_count = frontier_cells.size();
        frontier_cells = filter_reachable_cells(frontier_cells, *current_map_, robot_x, robot_y);

        if (frontier_cells.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No reachable frontier cells (detected=%zu, reachable=0). "
                "Either exploration is complete or every frontier is walled off.",
                detected_count);
            return;
        }

        auto frontiers = cluster_frontiers(frontier_cells, *current_map_);

        // ─── 協調・チャタリング防止ロジック ───
        if (state_ == State::MOVING) {
            double gx = current_goal_.pose.position.x;
            double gy = current_goal_.pose.position.y;

            // 現在目指しているゴールの近傍にfrontierが残り、かつ最新の
            // Voronoiマスクでも自分の担当領域なら、現在の走行を続ける。
            const bool current_goal_in_territory = !use_voronoi_ ||
                (voronoi_mask_ && voronoi::in_own_territory(
                    *voronoi_mask_, gx, gy));
            bool current_frontier_still_exists = false;
            if (current_goal_in_territory) {
                for (const auto &f : frontiers) {
                    if (std::hypot(f.centroid_x - gx, f.centroid_y - gy) < 1.0) {
                        current_frontier_still_exists = true;
                        break;
                    }
                }
            }

            // フロンティアが残っているなら、余計な再計画をせずに直進を維持
            if (current_frontier_still_exists) {
                Frontier *best = select_frontier_in_territory(
                    frontiers, robot_x, robot_y, robot_yaw);
                publish_frontiers(frontiers, best);
                return;
            } else {
                // frontier消滅・障害物化・担当境界の移動のいずれでも、
                // 現在のゴールが担当外になったらキャンセルして再選択する。
                RCLCPP_INFO(this->get_logger(),
                    "Current target vanished or left own territory. Replanning...");
                cancel_current_goal();
            }
        }
        // ──────────────────────────────────────

        // ベストなフロンティアを選択
        Frontier *best = select_frontier_in_territory(
            frontiers, robot_x, robot_y, robot_yaw);

        if (!best) {
            size_t too_small = 0, blk = 0, claimed = 0;
            double max_size = 0.0;
            for (const auto &f : frontiers) {
                max_size = std::max(max_size, f.size);
                if (f.size < min_frontier_size_) { ++too_small; continue; }
                if (is_blacklisted(f.centroid_x, f.centroid_y)) { ++blk; continue; }
                if (is_peer_claimed(f.centroid_x, f.centroid_y)) { ++claimed; }
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No selectable frontier (total=%zu too_small=%zu(max_size=%.2fm, min=%.2fm) "
                "blacklisted=%zu peer_claimed=%zu, blacklist_entries=%zu)",
                frontiers.size(), too_small, max_size, min_frontier_size_,
                blk, claimed, blacklist_.size());
            return;
        }

        // Rviz用可視化マーカーの配信
        publish_frontiers(frontiers, best);

        RCLCPP_INFO(this->get_logger(),
            "Frontier: (%.2f, %.2f) size=%.2fm score=%.2f | clusters=%zu blacklist=%zu",
            best->centroid_x, best->centroid_y, best->size, best->score,
            frontiers.size(), blacklist_.size());

        // 新しい目標を送信
        send_nav_goal(best->centroid_x, best->centroid_y);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
