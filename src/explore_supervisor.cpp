#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>

/**
 * @brief Multi-Robot Exploration Supervisor
 * 
 * 複数ロボットの探索を統括するノード
 * 機能:
 * - 各ロボットの探索進捗を監視
 * - 地図の統合状況を確認
 * - 探索の開始/停止を制御
 * - マルチロボット間の協調指示を送信
 */
class ExploreSupervisor : public rclcpp::Node {
public:
    ExploreSupervisor() : Node("explore_supervisor") {
        // ==================== パラメータ宣言 ====================
        this->declare_parameter<int>("num_robots", 2);
        this->declare_parameter<double>("loop_rate_hz", 1.0);
        this->declare_parameter<double>("merge_timeout_sec", 10.0);
        this->declare_parameter<bool>("use_sim_time", true);

        // ==================== パラメータ取得 ====================
        num_robots_ = this->get_parameter("num_robots").as_int();
        loop_rate_hz_ = this->get_parameter("loop_rate_hz").as_double();
        merge_timeout_sec_ = this->get_parameter("merge_timeout_sec").as_double();

        // ==================== ロボット名リスト作成 ====================
        for (int i = 1; i <= num_robots_; i++) {
            std::string robot_name = "robot_" + std::to_string(i);
            robot_names_.push_back(robot_name);
            robot_map_updates_[robot_name] = rclcpp::Clock().now();
        }

        // ==================== Subscriber 作成 ====================
        // 各ロボットの地図を購読
        for (const auto &robot_name : robot_names_) {
            std::string map_topic = "/" + robot_name + "/map";
            auto map_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                map_topic,
                rclcpp::SensorDataQoS(),
                [this, robot_name](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                    this->robot_map_callback(robot_name, msg);
                }
            );
            map_subscriptions_[robot_name] = map_subscription;
            RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", map_topic.c_str());
        }

        // 統合地図を購読
        merged_map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::SensorDataQoS(),
            std::bind(&ExploreSupervisor::merged_map_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to merged map: /map");

        // ==================== Service Server 作成 ====================
        // 探索制御サービス
        start_exploration_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_exploration",
            std::bind(&ExploreSupervisor::start_exploration_callback, this,
                std::placeholders::_1, std::placeholders::_2)
        );

        stop_exploration_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_exploration",
            std::bind(&ExploreSupervisor::stop_exploration_callback, this,
                std::placeholders::_1, std::placeholders::_2)
        );

        // ==================== Publisher 作成 ====================
        status_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "supervisor_status",
            10
        );

        // ==================== タイマー作成 ====================
        // 監視ループ
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / loop_rate_hz_),
            std::bind(&ExploreSupervisor::supervision_loop, this)
        );

        exploration_state_ = ExplorationState::IDLE;

        RCLCPP_INFO(this->get_logger(),
            "ExploreSupervisor initialized with %d robots at %.1f Hz",
            num_robots_, loop_rate_hz_);
    }

private:
    // ==================== メンバ変数 ====================

    // パラメータ
    int num_robots_;
    double loop_rate_hz_;
    double merge_timeout_sec_;

    // ロボット情報
    std::vector<std::string> robot_names_;
    std::map<std::string, rclcpp::Time> robot_map_updates_;
    std::map<std::string, size_t> robot_map_occupancy_;

    // 探索状態
    enum class ExplorationState {
        IDLE,         // 待機中
        EXPLORING,    // 探索中
        PAUSED,       // 一時停止
        COMPLETED     // 完了
    };
    ExplorationState exploration_state_;

    // 統合地図の状態
    size_t merged_map_last_size_ = 0;
    rclcpp::Time last_merge_update_;

    // Subscriber / Publisher / Service
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subscriptions_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr status_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_exploration_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_exploration_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==================== コールバック関数 ====================

    /**
     * @brief 各ロボットの地図更新コールバック
     * @param robot_name ロボット名
     * @param msg OccupancyGrid メッセージ
     */
    void robot_map_callback(const std::string &robot_name,
                          const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // 最終更新時刻を記録
        robot_map_updates_[robot_name] = this->now();

        // 占有セルの数をカウント
        size_t occupied_count = 0;
        for (auto cell : msg->data) {
            if (cell > 50) occupied_count++;
        }
        robot_map_occupancy_[robot_name] = occupied_count;

        RCLCPP_DEBUG(this->get_logger(),
            "%s map updated: %zu occupied cells",
            robot_name.c_str(), occupied_count);
    }

    /**
     * @brief 統合地図更新コールバック
     * @param msg OccupancyGrid メッセージ
     */
    void merged_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        merged_map_last_size_ = msg->data.size();
        last_merge_update_ = this->now();

        RCLCPP_DEBUG(this->get_logger(),
            "Merged map updated: %zu cells, resolution: %.3f m/cell",
            msg->data.size(), msg->info.resolution);
    }

    /**
     * @brief 探索開始サービスコールバック
     */
    void start_exploration_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;

        if (exploration_state_ == ExplorationState::EXPLORING) {
            response->success = false;
            response->message = "Already exploring";
            return;
        }

        exploration_state_ = ExplorationState::EXPLORING;
        response->success = true;
        response->message = "Exploration started";

        RCLCPP_WARN(this->get_logger(), "Exploration STARTED");
    }

    /**
     * @brief 探索停止サービスコールバック
     */
    void stop_exploration_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;

        exploration_state_ = ExplorationState::IDLE;
        response->success = true;
        response->message = "Exploration stopped";

        RCLCPP_WARN(this->get_logger(), "Exploration STOPPED");
    }

    // ==================== ヘルパー関数 ====================

    /**
     * @brief ロボットが活動しているかチェック
     * @param robot_name ロボット名
     * @return true: 活動中, false: 非活動
     */
    bool is_robot_active(const std::string &robot_name) {
        auto it = robot_map_updates_.find(robot_name);
        if (it == robot_map_updates_.end()) return false;

        auto time_since_update = this->now() - it->second;
        return time_since_update.seconds() < merge_timeout_sec_;
    }

    /**
     * @brief 全ロボットの統計情報を出力
     */
    void print_statistics() {
        RCLCPP_INFO(this->get_logger(), "=== Exploration Status ===");
        RCLCPP_INFO(this->get_logger(), "State: %s", state_to_string().c_str());

        for (const auto &robot_name : robot_names_) {
            bool active = is_robot_active(robot_name);
            size_t occupancy = robot_map_occupancy_[robot_name];
            auto time_since = (this->now() - robot_map_updates_[robot_name]).seconds();

            RCLCPP_INFO(this->get_logger(),
                "  %s: %s | occupancy=%zu | last_update=%.1fs ago",
                robot_name.c_str(),
                active ? "ACTIVE" : "INACTIVE",
                occupancy,
                time_since);
        }

        RCLCPP_INFO(this->get_logger(),
            "Merged map: %zu cells, last_update=%.1fs ago",
            merged_map_last_size_,
            (this->now() - last_merge_update_).seconds());
    }

    /**
     * @brief 探索状態を文字列に変換
     */
    std::string state_to_string() const {
        switch (exploration_state_) {
            case ExplorationState::IDLE:      return "IDLE";
            case ExplorationState::EXPLORING: return "EXPLORING";
            case ExplorationState::PAUSED:    return "PAUSED";
            case ExplorationState::COMPLETED: return "COMPLETED";
            default:                          return "UNKNOWN";
        }
    }

    // ==================== メインループ ====================

    /**
     * @brief 監視ループ（定期的に実行）
     */
    void supervision_loop() {
        // ロボットの活動状況をチェック
        int active_robots = 0;
        for (const auto &robot_name : robot_names_) {
            if (is_robot_active(robot_name)) {
                active_robots++;
            }
        }

        // 定期的に統計を出力
        static int loop_count = 0;
        if (++loop_count >= static_cast<int>(loop_rate_hz_)) {
            print_statistics();
            loop_count = 0;

            // 全ロボットが非活動になった場合
            if (active_robots == 0 && exploration_state_ == ExplorationState::EXPLORING) {
                RCLCPP_WARN(this->get_logger(),
                    "All robots inactive. Exploration may be complete.");
            }
        }

        // 各ロボットの map トピックを確認
        for (const auto &robot_name : robot_names_) {
            if (!is_robot_active(robot_name)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                    "%s has not published map for %.1f seconds",
                    robot_name.c_str(), merge_timeout_sec_);
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExploreSupervisor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}