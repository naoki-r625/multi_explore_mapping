#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>

class SensorExploreNode : public rclcpp::Node {
public:
    SensorExploreNode() : Node("sensor_explore") {
        // ==================== パラメータ宣言 ====================
        this->declare_parameter<double>("linear_speed", 0.2);
        this->declare_parameter<double>("angular_speed", 0.5); // sbe_node.pyのデフォルト
        this->declare_parameter<double>("safe_distance", 0.5); // sbe_node.pyのデフォルト
        this->declare_parameter<double>("front_half_deg", 30.0);
        this->declare_parameter<double>("side_center_deg", 90.0);
        this->declare_parameter<double>("side_half_deg", 40.0);

        // ==================== パラメータ取得 ====================
        linear_speed_   = this->get_parameter("linear_speed").as_double();
        angular_speed_  = this->get_parameter("angular_speed").as_double();
        safe_distance_  = this->get_parameter("safe_distance").as_double();
        front_half_deg_ = this->get_parameter("front_half_deg").as_double();
        side_center_deg_= this->get_parameter("side_center_deg").as_double();
        side_half_deg_  = this->get_parameter("side_half_deg").as_double();

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
            "Simple SBE Node initialized | v=%.2f m/s, w=%.2f rad/s, safe=%.2f m",
            linear_speed_, angular_speed_, safe_distance_);
    }

private:
    // ==================== パラメータ ====================
    double linear_speed_;
    double angular_speed_;
    double safe_distance_;
    double front_half_deg_;
    double side_center_deg_;
    double side_half_deg_;

    // ==================== センサーデータ ====================
    sensor_msgs::msg::LaserScan latest_scan_;
    nav_msgs::msg::Odometry latest_odom_;
    bool has_scan_data_ = false;
    bool has_odom_data_ = false;

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
    //  安全なLiDAR距離取得ヘルパー（角度範囲ベース）
    // ================================================================
    /**
     * @brief 指定した角度範囲（ラジアン）の最小距離を取得する
     * 角度は正面を0とし、左が正(+), 右が負(-)
     */
    double get_min_distance_in_range(double angle_min_rad, double angle_max_rad) {
        if (!has_scan_data_ || latest_scan_.ranges.empty()) {
            return safe_distance_ + 1.0;
        }

        double min_dist = latest_scan_.range_max;
        double current_angle = latest_scan_.angle_min;

        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
            // インクリメントしながら全セルの角度をチェック
            if (current_angle >= angle_min_rad && current_angle <= angle_max_rad) {
                float r = latest_scan_.ranges[i];
                if (std::isfinite(r) && r >= latest_scan_.range_min && r <= latest_scan_.range_max) {
                    min_dist = std::min(min_dist, static_cast<double>(r));
                }
            }
            current_angle += latest_scan_.angle_increment;
        }
        return min_dist;
    }

    double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

    // ================================================================
    //  メイン制御ループ (10Hz)
    // ================================================================
    // ================================================================
    //  メイン制御ループ (10Hz) - 斜め衝突対策版
    // ================================================================
    void control_loop() {
        if (!has_scan_data_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for LiDAR scan data...");
            return;
        }

        geometry_msgs::msg::Twist cmd_vel;

        // --- 【対策1】直進を止めるための「前方＋斜め前方」の監視範囲を定義 ---
        // 元の30°から45°〜50°に広げることで、斜めからの接近を100%捕捉します
        double front_block_rad = deg_to_rad(45.0); 
        double front_dist = get_min_distance_in_range(-front_block_rad, front_block_rad);

        // --- 【対策2】回避方向（左右）を決めるエリアを真横から「斜め前方」にシフト ---
        // 旋回して逃げる先のスペースが本当にあるかを、より前寄りの角度で評価します
        double left_scan_min = deg_to_rad(30.0);
        double left_scan_max = deg_to_rad(150.0);
        double left_dist = get_min_distance_in_range(left_scan_min, left_scan_max);

        double right_scan_min = deg_to_rad(-150.0);
        double right_scan_max = deg_to_rad(-30.0);
        double right_dist = get_min_distance_in_range(right_scan_min, right_scan_max);

        // --- 危険度に応じた速度の減速（実機向けの隠し味） ---
        // 障害物に近づくほど直進速度を落とすと、さらに安定します
        double current_linear_speed = linear_speed_;
        if (front_dist < (safe_distance_ * 1.5)) {
            // 安全距離の1.5倍以内に近づいたら速度を半分にする
            current_linear_speed = linear_speed_ * 0.5;
        }

        // --- 回避アルゴリズム ---
        if (front_dist < safe_distance_) {
            // 斜め前方を含むどこかに壁が食い込んだら即座に直進を停止
            cmd_vel.linear.x = 0.0;
            
            if (left_dist > right_dist) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Obstacle detected! Turning LEFT");
                cmd_vel.angular.z = angular_speed_;  // 左斜め前の方が広いので左回転
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Obstacle detected! Turning RIGHT");
                cmd_vel.angular.z = -angular_speed_; // 右斜め前の方が広いので右回転
            }
        } else {
            // 完全に安全なら直進
            cmd_vel.linear.x = current_linear_speed;
            cmd_vel.angular.z = 0.0;
        }

        cmd_vel_publisher_->publish(cmd_vel);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorExploreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}