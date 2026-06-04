#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <deque>
#include <random>

class SensorExploreNode : public rclcpp::Node {
public:
    SensorExploreNode() : Node("sensor_explore"), rng_(std::random_device{}()) {
        // ==================== パラメータ宣言 ====================
        this->declare_parameter<double>("linear_speed", 0.2);
        this->declare_parameter<double>("angular_speed", 0.6);
        this->declare_parameter<double>("safe_distance", 0.5);
        this->declare_parameter<double>("front_half_deg", 30.0);
        this->declare_parameter<double>("side_center_deg", 90.0);
        this->declare_parameter<double>("side_half_deg", 40.0);
        // use_sim_time は rclcpp が自動宣言するため declare 不要
        // launch 側の parameters=[{'use_sim_time': True}] で値が渡される

        // --- 脱出パラメータ ---
        this->declare_parameter<double>("escape_back_speed", 0.15);
        this->declare_parameter<double>("escape_turn_speed", 0.8);
        this->declare_parameter<int>("escape_back_ticks", 15);
        this->declare_parameter<int>("escape_turn_ticks_min", 10);
        this->declare_parameter<int>("escape_turn_ticks_max", 30);
        this->declare_parameter<int>("oscillation_window", 20);
        this->declare_parameter<int>("oscillation_threshold", 6);
        this->declare_parameter<int>("turn_hold_ticks", 8);

        // ==================== パラメータ取得 ====================
        linear_speed_       = this->get_parameter("linear_speed").as_double();
        angular_speed_      = this->get_parameter("angular_speed").as_double();
        safe_distance_      = this->get_parameter("safe_distance").as_double();
        front_half_deg_     = this->get_parameter("front_half_deg").as_double();
        side_center_deg_    = this->get_parameter("side_center_deg").as_double();
        side_half_deg_      = this->get_parameter("side_half_deg").as_double();

        escape_back_speed_  = this->get_parameter("escape_back_speed").as_double();
        escape_turn_speed_  = this->get_parameter("escape_turn_speed").as_double();
        escape_back_ticks_  = this->get_parameter("escape_back_ticks").as_int();
        escape_turn_ticks_min_ = this->get_parameter("escape_turn_ticks_min").as_int();
        escape_turn_ticks_max_ = this->get_parameter("escape_turn_ticks_max").as_int();
        oscillation_window_    = this->get_parameter("oscillation_window").as_int();
        oscillation_threshold_ = this->get_parameter("oscillation_threshold").as_int();
        turn_hold_ticks_       = this->get_parameter("turn_hold_ticks").as_int();

        // ==================== Subscriber ====================
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS(),
            std::bind(&SensorExploreNode::scan_callback, this, std::placeholders::_1)
        );

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            rclcpp::SensorDataQoS(),
            std::bind(&SensorExploreNode::odom_callback, this, std::placeholders::_1)
        );

        // ==================== Publisher ====================
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // ==================== タイマー (10 Hz) ====================
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorExploreNode::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(),
            "SensorExploreNode initialized | v=%.2f m/s, w=%.2f rad/s, "
            "osc_window=%d, osc_thresh=%d, hold=%d ticks",
            linear_speed_, angular_speed_,
            oscillation_window_, oscillation_threshold_, turn_hold_ticks_);
    }

private:
    // ==================== 状態定義 ====================
    //
    //  通常状態:
    //    FORWARD      前方が開いている → 直進
    //    TURN_LEFT    左へ回転（hold_ticks 間は方向を維持）
    //    TURN_RIGHT   右へ回転（同上）
    //
    //  脱出状態 (振動 or 全方向閉塞で遷移):
    //    ESCAPE_BACK  後退して障害物から距離を取る
    //    ESCAPE_TURN  ランダム方向へ大きく回転して向きを変える
    //
    enum State {
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        ESCAPE_BACK,
        ESCAPE_TURN,
    };

    // ==================== パラメータ ====================
    double linear_speed_;
    double angular_speed_;
    double safe_distance_;
    double front_half_deg_;
    double side_center_deg_;
    double side_half_deg_;

    // 脱出パラメータ
    double escape_back_speed_;       // 後退速度 (m/s)
    double escape_turn_speed_;       // 脱出回転速度 (rad/s)
    int    escape_back_ticks_;       // 後退を続けるtick数
    int    escape_turn_ticks_min_;   // 回転の最小tick数
    int    escape_turn_ticks_max_;   // 回転の最大tick数
    int    oscillation_window_;      // 振動検出ウィンドウ (直近N tick)
    int    oscillation_threshold_;   // このウィンドウ内で状態変化がN回以上 → 振動
    int    turn_hold_ticks_;         // 回転方向を維持する最小tick数

    // ==================== センサーデータ ====================
    sensor_msgs::msg::LaserScan latest_scan_;
    nav_msgs::msg::Odometry latest_odom_;
    bool has_scan_data_ = false;
    bool has_odom_data_ = false;

    // ==================== 状態管理 ====================
    State current_state_      = FORWARD;
    int   escape_ticks_left_  = 0;      // 脱出行動の残りtick
    double escape_direction_  = 1.0;    // 脱出回転方向 (+1 左, -1 右)
    int   hold_ticks_left_    = 0;      // 回転維持の残りtick

    // ==================== 振動検出 ====================
    std::deque<State> state_history_;   // 直近の状態履歴

    // ==================== 乱数 ====================
    std::mt19937 rng_;

    // ==================== ROS ====================
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ================================================================
    //  コールバック
    // ================================================================

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = *msg;
        has_scan_data_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_data_ = true;
    }

    // ================================================================
    //  LiDAR ヘルパー
    // ================================================================

    double get_min_distance(double angle_min, double angle_max) {
        if (!has_scan_data_) return safe_distance_ + 1.0;

        double min_dist = latest_scan_.range_max;
        int start_idx = angle_to_index(angle_min);
        int end_idx   = angle_to_index(angle_max);
        if (start_idx > end_idx) std::swap(start_idx, end_idx);

        int n = static_cast<int>(latest_scan_.ranges.size());
        for (int i = std::max(0, start_idx); i <= std::min(n - 1, end_idx); i++) {
            float r = latest_scan_.ranges[i];
            if (r >= latest_scan_.range_min && r <= latest_scan_.range_max) {
                min_dist = std::min(min_dist, static_cast<double>(r));
            }
        }
        return min_dist;
    }

    int angle_to_index(double angle) {
        if (!has_scan_data_) return 0;
        return static_cast<int>(
            (angle - latest_scan_.angle_min) / latest_scan_.angle_increment);
    }

    double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

    // ================================================================
    //  振動検出
    // ================================================================

    /**
     * @brief 直近 oscillation_window_ tick の状態遷移回数を数える
     *        TURN_LEFT ↔ TURN_RIGHT の切り替わりのみカウント
     * @return 振動回数
     */
    int count_oscillations() {
        int flips = 0;
        for (size_t i = 1; i < state_history_.size(); i++) {
            State prev = state_history_[i - 1];
            State curr = state_history_[i];
            bool is_flip =
                (prev == TURN_LEFT  && curr == TURN_RIGHT) ||
                (prev == TURN_RIGHT && curr == TURN_LEFT);
            if (is_flip) flips++;
        }
        return flips;
    }

    /**
     * @brief 状態履歴に記録し、ウィンドウを維持する
     */
    void record_state(State s) {
        state_history_.push_back(s);
        while (static_cast<int>(state_history_.size()) > oscillation_window_) {
            state_history_.pop_front();
        }
    }

    // ================================================================
    //  脱出シーケンス開始
    // ================================================================

    /**
     * @brief 脱出行動を開始する
     *
     *   ESCAPE_BACK (後退)
     *       ↓ escape_back_ticks_ 経過
     *   ESCAPE_TURN (ランダム方向へ回転)
     *       ↓ escape_turn_ticks (ランダム) 経過
     *   通常状態に復帰
     */
    void start_escape(const std::string &reason) {
        current_state_ = ESCAPE_BACK;
        escape_ticks_left_ = escape_back_ticks_;

        // 回転方向をランダムに決定
        std::uniform_int_distribution<int> coin(0, 1);
        escape_direction_ = coin(rng_) ? 1.0 : -1.0;

        // 履歴をクリアして脱出直後の再検出を防ぐ
        state_history_.clear();

        RCLCPP_WARN(this->get_logger(),
            "ESCAPE triggered (%s) | direction=%s",
            reason.c_str(),
            escape_direction_ > 0 ? "LEFT" : "RIGHT");
    }

    // ================================================================
    //  通常の行動判定
    // ================================================================

    /**
     * @brief LiDAR を使った通常の方向判定
     *
     *  旧版との違い:
     *    - 左右の差が僅かな場合はランダムに選ぶ（振動の根本原因を減らす）
     *    - 全方向閉塞の場合は脱出に入る（STOP で固まらない）
     *
     * @return 遷移先の State。脱出が必要なときは ESCAPE_BACK を返す。
     */
    State decide_normal_state() {
        double front_angle_half = deg_to_rad(front_half_deg_);
        double front_dist = get_min_distance(-front_angle_half, front_angle_half);

        double left_half  = deg_to_rad(side_half_deg_);
        double left_angle = deg_to_rad(side_center_deg_);
        double left_dist  = get_min_distance(left_angle - left_half, left_angle + left_half);

        double right_angle = -deg_to_rad(side_center_deg_);
        double right_dist  = get_min_distance(right_angle - left_half, right_angle + left_half);

        RCLCPP_DEBUG(this->get_logger(),
            "F=%.2f L=%.2f R=%.2f | safe=%.2f",
            front_dist, left_dist, right_dist, safe_distance_);

        // --- 前方が開いている → 直進 ---
        if (front_dist > safe_distance_) {
            return FORWARD;
        }

        bool left_ok  = (left_dist  > safe_distance_);
        bool right_ok = (right_dist > safe_distance_);

        // --- 片方だけ開いている → そちらへ ---
        if (left_ok && !right_ok)  return TURN_LEFT;
        if (!left_ok && right_ok)  return TURN_RIGHT;

        // --- 両方開いている → 距離差が小さければランダム、大きければ遠い方 ---
        if (left_ok && right_ok) {
            double diff = std::abs(left_dist - right_dist);
            // 距離差が 0.3m 未満なら「ほぼ同じ」とみなしてランダム
            if (diff < 0.3) {
                std::uniform_int_distribution<int> coin(0, 1);
                return coin(rng_) ? TURN_LEFT : TURN_RIGHT;
            }
            return (left_dist > right_dist) ? TURN_LEFT : TURN_RIGHT;
        }

        // --- 全方向閉塞 → 脱出 ---
        return ESCAPE_BACK;
    }

    // ================================================================
    //  メイン制御ループ
    // ================================================================

    void control_loop() {
        if (!has_scan_data_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for LiDAR scan data...");
            return;
        }

        geometry_msgs::msg::Twist cmd_vel;

        // -----------------------------------------------------------
        //  脱出中の処理（後退 → 回転）
        // -----------------------------------------------------------
        if (current_state_ == ESCAPE_BACK) {
            if (escape_ticks_left_ > 0) {
                cmd_vel = create_twist(-escape_back_speed_, 0.0);
                escape_ticks_left_--;
            } else {
                // 後退完了 → 回転フェーズへ
                current_state_ = ESCAPE_TURN;
                // 回転tick数をランダムに決定
                std::uniform_int_distribution<int> dist(
                    escape_turn_ticks_min_, escape_turn_ticks_max_);
                escape_ticks_left_ = dist(rng_);
                RCLCPP_INFO(this->get_logger(),
                    "ESCAPE_BACK done -> ESCAPE_TURN (%d ticks)", escape_ticks_left_);
            }
            cmd_vel_publisher_->publish(cmd_vel);
            return;
        }

        if (current_state_ == ESCAPE_TURN) {
            if (escape_ticks_left_ > 0) {
                cmd_vel = create_twist(0.0, escape_direction_ * escape_turn_speed_);
                escape_ticks_left_--;
            } else {
                // 脱出完了 → 通常状態に復帰
                current_state_ = FORWARD;
                hold_ticks_left_ = 0;
                RCLCPP_INFO(this->get_logger(), "ESCAPE complete -> FORWARD");
            }
            cmd_vel_publisher_->publish(cmd_vel);
            return;
        }

        // -----------------------------------------------------------
        //  回転ホールド中（まだ保持tick が残っている場合はスキャンの再判定をしない）
        // -----------------------------------------------------------
        if (hold_ticks_left_ > 0 &&
            (current_state_ == TURN_LEFT || current_state_ == TURN_RIGHT))
        {
            hold_ticks_left_--;
            double w = (current_state_ == TURN_LEFT) ? angular_speed_ : -angular_speed_;
            cmd_vel = create_twist(0.0, w);
            cmd_vel_publisher_->publish(cmd_vel);
            return;
        }

        // -----------------------------------------------------------
        //  通常の判定
        // -----------------------------------------------------------
        State next = decide_normal_state();

        // 全方向閉塞の場合
        if (next == ESCAPE_BACK) {
            start_escape("all_blocked");
            cmd_vel = create_twist(-escape_back_speed_, 0.0);
            escape_ticks_left_--;
            cmd_vel_publisher_->publish(cmd_vel);
            return;
        }

        // 状態遷移の記録
        record_state(next);

        // 振動検出 → 脱出
        if (count_oscillations() >= oscillation_threshold_) {
            start_escape("oscillation_detected");
            cmd_vel = create_twist(-escape_back_speed_, 0.0);
            escape_ticks_left_--;
            cmd_vel_publisher_->publish(cmd_vel);
            return;
        }

        // 状態遷移ログ
        if (next != current_state_) {
            RCLCPP_INFO(this->get_logger(), "%s -> %s",
                state_to_string(current_state_).c_str(),
                state_to_string(next).c_str());
            current_state_ = next;

            // 回転に入ったらホールドタイマーを起動
            if (current_state_ == TURN_LEFT || current_state_ == TURN_RIGHT) {
                hold_ticks_left_ = turn_hold_ticks_;
            }
        }

        // 速度指令
        switch (current_state_) {
            case FORWARD:
                cmd_vel = create_twist(linear_speed_, 0.0);
                break;
            case TURN_LEFT:
                cmd_vel = create_twist(0.0, angular_speed_);
                break;
            case TURN_RIGHT:
                cmd_vel = create_twist(0.0, -angular_speed_);
                break;
            default:
                cmd_vel = create_twist(0.0, 0.0);
                break;
        }

        cmd_vel_publisher_->publish(cmd_vel);
    }

    // ================================================================
    //  ユーティリティ
    // ================================================================

    geometry_msgs::msg::Twist create_twist(double linear_x, double angular_z) {
        geometry_msgs::msg::Twist t;
        t.linear.x  = linear_x;
        t.angular.z = angular_z;
        return t;
    }

    std::string state_to_string(State s) {
        switch (s) {
            case FORWARD:      return "FORWARD";
            case TURN_LEFT:    return "TURN_LEFT";
            case TURN_RIGHT:   return "TURN_RIGHT";
            case ESCAPE_BACK:  return "ESCAPE_BACK";
            case ESCAPE_TURN:  return "ESCAPE_TURN";
            default:           return "UNKNOWN";
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorExploreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}