#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "multi_explore_mapping/icp.hpp"
#include <algorithm>
#include <functional>
#include <limits>
#include <random>

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

class ICPMapMatchingNode : public rclcpp::Node {
public:
    ICPMapMatchingNode() : rclcpp::Node("icp_map_matching") {
        declare_parameter("robot_1_init_x",   0.0);
        declare_parameter("robot_1_init_y",   0.0);
        declare_parameter("robot_1_init_yaw", 0.0);
        declare_parameter("robot_2_init_x",  0.0);
        declare_parameter("robot_2_init_y",   0.0);
        declare_parameter("robot_2_init_yaw", 0.0);
        declare_parameter("max_icp_iterations",           50);
        declare_parameter("icp_convergence_threshold",    0.05);
        declare_parameter("max_correspondence_distance",  0.5);   // メートル
        declare_parameter("overlap_filter_margin",        2.0);   // メートル
        declare_parameter("max_feature_points",           1500);

        map_pub_ = create_publisher<OccupancyGrid>(
            "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

        sub_map_1_ = create_subscription<OccupancyGrid>(
            "/robot_1/map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(maps_mutex_);
                maps_["robot_1"] = std::move(msg);
            });

        sub_map_2_ = create_subscription<OccupancyGrid>(
            "/robot_2/map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(maps_mutex_);
                maps_["robot_2"] = std::move(msg);
            });

        timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() { icp_matching(); });

        RCLCPP_INFO(get_logger(), "ICP Map Matching Node started");
    }

private:
    struct Pose2D { float x, y, yaw; };

    // OccupancyGrid のエッジ点を世界座標(メートル)で抽出
    // world = R(yaw) * local + (init_x, init_y)
    // local = (origin_x + col*res, origin_y + row*res)
    Eigen::MatrixXf extract_world_points(
        const OccupancyGrid::SharedPtr& grid,
        const Pose2D& pose)
    {
        const int W   = (int)grid->info.width;
        const int H   = (int)grid->info.height;
        const float res = grid->info.resolution;
        const float ox  = grid->info.origin.position.x;
        const float oy  = grid->info.origin.position.y;
        const float c   = std::cos(pose.yaw);
        const float s   = std::sin(pose.yaw);

        // OccupancyGrid → CV_8U
        cv::Mat img(H, W, CV_8U);
        for (int row = 0; row < H; ++row)
            for (int col = 0; col < W; ++col) {
                int8_t v = grid->data[row * W + col];
                img.at<uint8_t>(row, col) =
                    (v < 0) ? 128u : (v > 50) ? 255u : 0u;
            }

        cv::Mat edges;
        cv::Canny(img, edges, 50, 150);

        std::vector<Eigen::Vector2f> pts;
        pts.reserve(4096);
        for (int row = 0; row < H; ++row)
            for (int col = 0; col < W; ++col)
                if (edges.at<uint8_t>(row, col) > 0) {
                    // pixel → ローカルmap座標 → 世界座標
                    float lx = ox + col * res;
                    float ly = oy + row * res;
                    pts.push_back({c*lx - s*ly + pose.x,
                                   s*lx + c*ly + pose.y});
                }

        const int max_pts = (int)get_parameter("max_feature_points").as_int();
        if ((int)pts.size() > max_pts) {
            std::default_random_engine rng(42);
            std::shuffle(pts.begin(), pts.end(), rng);
            pts.resize(max_pts);
        }

        Eigen::MatrixXf result((int)pts.size(), 2);
        for (int i = 0; i < (int)pts.size(); ++i)
            result.row(i) = pts[i].transpose();
        return result;
    }

    // reference の包含バウンディングボックス(+ margin)に入る pts だけを返す
    // これにより両マップの重複領域のみで ICP を実行できる
    Eigen::MatrixXf filter_to_bbox(
        const Eigen::MatrixXf& pts,
        const Eigen::MatrixXf& reference,
        float margin)
    {
        if (pts.rows() == 0 || reference.rows() == 0) return pts;

        const float min_x = reference.col(0).minCoeff() - margin;
        const float max_x = reference.col(0).maxCoeff() + margin;
        const float min_y = reference.col(1).minCoeff() - margin;
        const float max_y = reference.col(1).maxCoeff() + margin;

        std::vector<Eigen::Vector2f> valid;
        valid.reserve(pts.rows());
        for (int i = 0; i < pts.rows(); ++i) {
            float x = pts(i, 0), y = pts(i, 1);
            if (x >= min_x && x <= max_x && y >= min_y && y <= max_y)
                valid.push_back(pts.row(i));
        }

        Eigen::MatrixXf result((int)valid.size(), 2);
        for (int i = 0; i < (int)valid.size(); ++i)
            result.row(i) = valid[i].transpose();
        return result;
    }

    // 両マップを世界座標でマージして /map をパブリッシュ
    // map1 を基準、map2 に ICP 補正変換 (R_icp, t_icp) を適用
    void merge_and_publish(
        const OccupancyGrid::SharedPtr& map1,
        const OccupancyGrid::SharedPtr& map2,
        const Pose2D& init1, const Pose2D& init2,
        const Eigen::Matrix2f& R_icp, const Eigen::Vector2f& t_icp)
    {
        const float res = map1->info.resolution;
        const float c1 = std::cos(init1.yaw), s1 = std::sin(init1.yaw);
        const float c2 = std::cos(init2.yaw), s2 = std::sin(init2.yaw);

        // ローカルmap座標 → 世界座標 (map1: 補正なし)
        auto to_world1 = [&](float lx, float ly) -> std::pair<float,float> {
            return {c1*lx - s1*ly + init1.x,
                    s1*lx + c1*ly + init1.y};
        };

        // ローカルmap座標 → 初期世界座標 → ICP補正後世界座標 (map2)
        auto to_world2 = [&](float lx, float ly) -> std::pair<float,float> {
            float wx = c2*lx - s2*ly + init2.x;
            float wy = s2*lx + c2*ly + init2.y;
            Eigen::Vector2f p = R_icp * Eigen::Vector2f(wx, wy) + t_icp;
            return {p(0), p(1)};
        };

        // 両マップ4隅の世界座標バウンディングボックス
        const float ox1 = map1->info.origin.position.x;
        const float oy1 = map1->info.origin.position.y;
        const float w1  = map1->info.width  * res;
        const float h1  = map1->info.height * res;
        const float ox2 = map2->info.origin.position.x;
        const float oy2 = map2->info.origin.position.y;
        const float w2  = map2->info.width  * res;
        const float h2  = map2->info.height * res;

        const std::vector<std::pair<float,float>> corners = {
            to_world1(ox1,    oy1   ), to_world1(ox1+w1, oy1   ),
            to_world1(ox1,    oy1+h1), to_world1(ox1+w1, oy1+h1),
            to_world2(ox2,    oy2   ), to_world2(ox2+w2, oy2   ),
            to_world2(ox2,    oy2+h2), to_world2(ox2+w2, oy2+h2),
        };

        float min_x = 1e9f, min_y = 1e9f, max_x = -1e9f, max_y = -1e9f;
        for (const auto& c : corners) {
            min_x = std::min(min_x, c.first);  max_x = std::max(max_x, c.first);
            min_y = std::min(min_y, c.second); max_y = std::max(max_y, c.second);
        }

        const float margin  = res * 2.0f;
        const float mox     = min_x - margin;
        const float moy     = min_y - margin;
        const int   merged_W = (int)std::ceil((max_x - min_x + 2.0f*margin) / res) + 1;
        const int   merged_H = (int)std::ceil((max_y - min_y + 2.0f*margin) / res) + 1;

        OccupancyGrid merged;
        merged.header.stamp              = now();
        merged.header.frame_id           = "map";
        merged.info.resolution           = res;
        merged.info.width                = (uint32_t)merged_W;
        merged.info.height               = (uint32_t)merged_H;
        merged.info.origin.position.x    = mox;
        merged.info.origin.position.y    = moy;
        merged.info.origin.orientation.w = 1.0;
        merged.data.assign(merged_W * merged_H, -1);

        // ソースマップのピクセルを世界座標変換してマージグリッドへ書き込む
        // 占有(100) > 空き(0) > 未知(-1) の優先度でマージ
        auto write_map = [&](
            const OccupancyGrid::SharedPtr& map,
            std::function<std::pair<float,float>(float,float)> tfm)
        {
            const int   mW   = (int)map->info.width;
            const int   mH   = (int)map->info.height;
            const float mres = map->info.resolution;
            const float mox_ = map->info.origin.position.x;
            const float moy_ = map->info.origin.position.y;

            for (int row = 0; row < mH; ++row) {
                for (int col = 0; col < mW; ++col) {
                    const int8_t val = map->data[row * mW + col];
                    if (val < 0) continue;

                    float lx = mox_ + col * mres;
                    float ly = moy_ + row * mres;
                    auto [wx, wy] = tfm(lx, ly);

                    const int mx = (int)std::round((wx - mox) / res);
                    const int my = (int)std::round((wy - moy) / res);
                    if (mx < 0 || mx >= merged_W || my < 0 || my >= merged_H) continue;

                    const int idx = my * merged_W + mx;
                    if (merged.data[idx] < val)
                        merged.data[idx] = val;
                }
            }
        };

        write_map(map1, [&](float lx, float ly){ return to_world1(lx, ly); });
        write_map(map2, [&](float lx, float ly){ return to_world2(lx, ly); });

        map_pub_->publish(merged);
        RCLCPP_INFO(get_logger(),
            "Published merged /map: %dx%d cells, origin=(%.2f, %.2f)",
            merged_W, merged_H, mox, moy);
    }

    void icp_matching() {
        OccupancyGrid::SharedPtr map1, map2;
        {
            std::lock_guard<std::mutex> lk(maps_mutex_);
            if (maps_.count("robot_1") == 0 || maps_.count("robot_2") == 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Waiting for both maps...");
                return;
            }
            map1 = maps_["robot_1"];
            map2 = maps_["robot_2"];
        }

        const Pose2D init1{
            (float)get_parameter("robot_1_init_x").as_double(),
            (float)get_parameter("robot_1_init_y").as_double(),
            (float)get_parameter("robot_1_init_yaw").as_double()};
        const Pose2D init2{
            (float)get_parameter("robot_2_init_x").as_double(),
            (float)get_parameter("robot_2_init_y").as_double(),
            (float)get_parameter("robot_2_init_yaw").as_double()};
        const int   max_iter       = (int)get_parameter("max_icp_iterations").as_int();
        const float conv_thresh    = (float)get_parameter("icp_convergence_threshold").as_double();
        const float max_corr_dist  = (float)get_parameter("max_correspondence_distance").as_double();
        const float overlap_margin = (float)get_parameter("overlap_filter_margin").as_double();

        // 両マップを世界座標に変換してエッジ特徴点を抽出
        Eigen::MatrixXf pts1 = extract_world_points(map1, init1);
        Eigen::MatrixXf pts2 = extract_world_points(map2, init2);

        RCLCPP_INFO(get_logger(),
            "World-frame feature points: robot_1=%ld, robot_2=%ld",
            pts1.rows(), pts2.rows());

        if (pts1.rows() < 10 || pts2.rows() < 10) {
            RCLCPP_WARN(get_logger(), "Insufficient features — merging with initial poses only");
            merge_and_publish(map1, map2, init1, init2,
                              Eigen::Matrix2f::Identity(),
                              Eigen::Vector2f::Zero());
            return;
        }

        // 重複領域のみに絞り込む:
        //   pts2 のうち pts1 のバウンディングボックス内にある点
        //   pts1 のうち pts2 のバウンディングボックス内にある点
        Eigen::MatrixXf pts2_overlap = filter_to_bbox(pts2, pts1, overlap_margin);
        Eigen::MatrixXf pts1_overlap = filter_to_bbox(pts1, pts2, overlap_margin);

        RCLCPP_INFO(get_logger(),
            "Overlap region: robot_1=%ld, robot_2=%ld pts",
            pts1_overlap.rows(), pts2_overlap.rows());

        if (pts1_overlap.rows() < 10 || pts2_overlap.rows() < 10) {
            RCLCPP_WARN(get_logger(),
                "Maps do not overlap sufficiently — merging with initial poses only");
            merge_and_publish(map1, map2, init1, init2,
                              Eigen::Matrix2f::Identity(),
                              Eigen::Vector2f::Zero());
            return;
        }

        // ICP: pts2_overlap(robot_2) を pts1_overlap(robot_1) に位置合わせ
        // max_correspondence_distance で遠すぎる対応を除外
        icp::ICP icp_algo(max_iter, conv_thresh, max_corr_dist);
        icp::ICPResult result = icp_algo.align(pts2_overlap, pts1_overlap);

        const float angle_deg =
            std::atan2(result.R(1,0), result.R(0,0)) * 180.0f / M_PI;

        RCLCPP_INFO(get_logger(),
            "ICP: iter=%d, error=%.4f m, valid_corr=%d, dx=%.4f m, dy=%.4f m, rot=%.2f deg",
            result.iterations, result.final_error, result.valid_correspondences,
            result.t(0), result.t(1), angle_deg);

        // ICP の信頼性チェック: 誤差が大きいか対応が少ない場合は初期位置のみで統合
        if (result.final_error > 0.5f || result.valid_correspondences < 10) {
            RCLCPP_WARN(get_logger(),
                "ICP result unreliable (error=%.4f m, valid=%d) — merging with initial poses only",
                result.final_error, result.valid_correspondences);
            merge_and_publish(map1, map2, init1, init2,
                              Eigen::Matrix2f::Identity(),
                              Eigen::Vector2f::Zero());
            return;
        }

        merge_and_publish(map1, map2, init1, init2, result.R, result.t);
    }

    rclcpp::Publisher<OccupancyGrid>::SharedPtr     map_pub_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr  sub_map_1_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr  sub_map_2_;
    rclcpp::TimerBase::SharedPtr                    timer_;
    std::map<std::string, OccupancyGrid::SharedPtr> maps_;
    std::mutex                                      maps_mutex_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPMapMatchingNode>());
    rclcpp::shutdown();
    return 0;
}
