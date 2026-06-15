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

#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <algorithm>
#include <random>

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
        this->declare_parameter<bool>("visualize", true);
        this->declare_parameter<double>("blacklist_radius", 1.0);
        this->declare_parameter<double>("blacklist_clear_sec", 60.0);

        robot_base_frame_  = this->get_parameter("robot_base_frame").as_string();
        global_frame_      = this->get_parameter("global_frame").as_string();
        planner_frequency_ = this->get_parameter("planner_frequency").as_double();
        progress_timeout_  = this->get_parameter("progress_timeout").as_double();
        min_frontier_size_ = this->get_parameter("min_frontier_size").as_double();
        potential_scale_   = this->get_parameter("potential_scale").as_double();
        gain_scale_        = this->get_parameter("gain_scale").as_double();
        visualize_         = this->get_parameter("visualize").as_bool();
        blacklist_radius_  = this->get_parameter("blacklist_radius").as_double();
        blacklist_clear_sec_ = this->get_parameter("blacklist_clear_sec").as_double();
        std::string map_topic = this->get_parameter("map_topic").as_string();

        // 統合コストマップをサブスクライブ
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1)
        );

        if (visualize_) {
            frontier_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "frontiers", 10);
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
            "FrontierExplorer ready | base=%s global=%s gain=%.1f potential=%.1f min_size=%.2fm",
            robot_base_frame_.c_str(), global_frame_.c_str(),
            gain_scale_, potential_scale_, min_frontier_size_);
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

    std::string robot_base_frame_;
    std::string global_frame_;
    double planner_frequency_;
    double progress_timeout_;
    double min_frontier_size_;
    double potential_scale_;
    double gain_scale_;
    bool visualize_;
    double blacklist_radius_;
    double blacklist_clear_sec_;

    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    bool map_updated_ = false;

    rclcpp::Time last_progress_time_;
    rclcpp::Time last_blacklist_clear_;
    geometry_msgs::msg::PoseStamped current_goal_;
    std::shared_ptr<GoalHandleNav> current_goal_handle_;

    // 到達失敗したフロンティアのブラックリスト
    std::vector<std::pair<double,double>> blacklist_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_publisher_;
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
            cluster.centroid_x = sum_x / n;
            cluster.centroid_y = sum_y / n;
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

    Frontier* select_best_frontier(
        std::vector<Frontier> &frontiers,
        double robot_x, double robot_y)
    {
        Frontier *best = nullptr;
        double best_score = -std::numeric_limits<double>::infinity();

        for (auto &f : frontiers) {
            if (f.size < min_frontier_size_) continue;
            if (is_blacklisted(f.centroid_x, f.centroid_y)) continue;

            double dist = std::max(0.01, std::hypot(f.centroid_x - robot_x,
                                                     f.centroid_y - robot_y));
            // スコア評価式：ゲイン（未探索面積）とポテンシャル（距離ペナルティ）
            f.score = gain_scale_ * f.size - potential_scale_ * dist;

            if (f.score > best_score) {
                best_score = f.score;
                best       = &f;
            }
        }
        return best;
    }

    bool get_robot_pose(double &x, double &y) {
        try {
            auto tf = tf_buffer_.lookupTransform(
                global_frame_, robot_base_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(0.5));
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            return true;
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "TF lookup failed: %s -> %s | %s", global_frame_.c_str(), robot_base_frame_.c_str(), e.what());
            return false;
        }
    }

    void send_nav_goal(double x, double y) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
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

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        opts.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr &gh) {
                if (!gh) {
                    RCLCPP_WARN(this->get_logger(), "Goal rejected by Nav2, will retry");
                    state_ = State::IDLE;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2");
                }
            };

        opts.result_callback =
            [this](const GoalHandleNav::WrappedResult &res) {
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
            };

        nav_client_->async_send_goal(goal_msg, opts);
        state_ = State::MOVING;
        last_progress_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Navigating to frontier (%.2f, %.2f)", x, y);
    }

    void cancel_current_goal() {
        if (state_ == State::MOVING) {
            nav_client_->async_cancel_all_goals();
            state_ = State::IDLE;
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

        // フロンティア一覧
        for (const auto &f : frontiers) {
            bool is_valid    = (f.size >= min_frontier_size_);
            bool is_selected = (selected && &f == selected);
            bool is_bl       = is_blacklisted(f.centroid_x, f.centroid_y);

            visualization_msgs::msg::Marker m;
            m.header.frame_id = global_frame_;
            m.header.stamp    = this->now();
            m.ns = "frontiers"; m.id = id++;
            m.type   = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = f.centroid_x;
            m.pose.position.y = f.centroid_y;
            m.pose.position.z = 0.1;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = std::max(0.2, std::min(1.0, f.size * 0.1));
            m.lifetime = rclcpp::Duration::from_seconds(2.0 / planner_frequency_);

            if (is_bl) {
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.5;
            } else if (is_selected) {
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0; // ターゲット（緑）
            } else if (is_valid) {
                m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0; m.color.a = 0.8; // 有効（青）
            } else {
                m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.3; // サイズ未満（灰）
            }
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

        double robot_x, robot_y;
        if (!get_robot_pose(robot_x, robot_y)) return;

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

        if (frontier_cells.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "No frontier cells in costmap. Map exploration might be completed.");
            return;
        }

        auto frontiers = cluster_frontiers(frontier_cells, *current_map_);

        // ─── 協調・チャタリング防止ロジック ───
        if (state_ == State::MOVING) {
            double gx = current_goal_.pose.position.x;
            double gy = current_goal_.pose.position.y;

            // 現在目指しているゴールの近傍（1.0m以内）にまだ未探索フロンティアが存在するか
            bool current_frontier_still_exists = false;
            for (const auto &f : frontiers) {
                if (std::hypot(f.centroid_x - gx, f.centroid_y - gy) < 1.0) {
                    current_frontier_still_exists = true;
                    break;
                }
            }

            // フロンティアが残っているなら、余計な再計画をせずに直進を維持
            if (current_frontier_still_exists) {
                Frontier *best = select_best_frontier(frontiers, robot_x, robot_y);
                publish_frontiers(frontiers, best);
                return; 
            } else {
                // 他のロボットに開拓されて消滅、あるいは障害物で埋まった場合は即座に次へ切り替え
                RCLCPP_INFO(this->get_logger(), "Current target vanished or cleared by other robot. Replanning...");
                cancel_current_goal();
            }
        }
        // ──────────────────────────────────────

        // ベストなフロンティアを選択
        Frontier *best = select_best_frontier(frontiers, robot_x, robot_y);

        if (!best) {
            long valid_count = std::count_if(frontiers.begin(), frontiers.end(),
                [this](const Frontier &f){ return f.size >= min_frontier_size_; });
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No reachable frontier (total=%zu valid=%ld blacklisted=%zu)",
                frontiers.size(), valid_count, blacklist_.size());
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