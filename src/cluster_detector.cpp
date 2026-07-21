#include <multi_explore_mapping/cluster_detector.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <cmath>
#include <limits>

namespace cluster {

std::vector<Cluster> detect(
    const sensor_msgs::msg::LaserScan& scan,
    double thresh_base,
    double /*thresh_factor*/,
    int    min_pts)
{
    // Step 1: Convert valid scan beams to PCL point cloud (laser frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const double r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max * 0.99) continue;
        const double a = scan.angle_min + i * scan.angle_increment;
        cloud->push_back({static_cast<float>(r * std::cos(a)),
                          static_cast<float>(r * std::sin(a)), 0.0f});
    }

    if (cloud->empty()) return {};

    // Step 2: PCL Euclidean cluster extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(thresh_base);
    ec.setMinClusterSize(min_pts);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Step 3: Compute stats and wall classification
    std::vector<Cluster> result;
    result.reserve(cluster_indices.size());

    for (const auto& seg : cluster_indices) {
        double sx = 0.0, sy = 0.0;
        double min_d = std::numeric_limits<double>::max();

        for (int idx : seg.indices) {
            const double x = cloud->points[idx].x;
            const double y = cloud->points[idx].y;
            sx   += x;
            sy   += y;
            min_d = std::min(min_d, std::hypot(x, y));
        }

        const int    cnt = static_cast<int>(seg.indices.size());
        const double cx  = sx / cnt;
        const double cy  = sy / cnt;

        double radius = 0.0;
        for (int idx : seg.indices) {
            radius = std::max(radius, std::hypot(
                cloud->points[idx].x - cx,
                cloud->points[idx].y - cy));
        }

        // Adaptive wall threshold: close obstacles fill more beams for the same size.
        // At 1m range LDS-01 has ~1° spacing so a 15-beam cluster ≈ 26cm arc (typical wall).
        const double mean_range = std::hypot(cx, cy);
        int wall_thresh;
        if      (mean_range < 1.5) wall_thresh = 30;
        else if (mean_range < 2.5) wall_thresh = 20;
        else                        wall_thresh = 5;

        result.push_back({cx, cy, std::atan2(cy, cx), radius, min_d, cnt,
                          /*is_wall=*/(cnt >= wall_thresh)});
    }

    return result;
}

} // namespace cluster
