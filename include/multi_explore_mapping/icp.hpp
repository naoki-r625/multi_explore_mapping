#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>

namespace icp {

struct ICPResult {
    Eigen::Matrix2f R;
    Eigen::Vector2f t;
    float final_error;
    int iterations;
    int valid_correspondences;
};

class ICP {
public:
    ICP(int max_iterations = 50,
        float convergence_threshold = 0.001f,
        float max_correspondence_distance = std::numeric_limits<float>::max());

    ICPResult align(const Eigen::MatrixXf& source,
                    const Eigen::MatrixXf& target);

    void set_max_iterations(int v) { max_iterations_ = v; }
    void set_convergence_threshold(float v) { convergence_threshold_ = v; }
    void set_max_correspondence_distance(float v) { max_correspondence_distance_ = v; }

private:
    int   max_iterations_;
    float convergence_threshold_;
    float max_correspondence_distance_;

    // Returns target index for each source point, or -1 if no match within max distance
    std::vector<int> find_closest_points(
        const Eigen::MatrixXf& source,
        const Eigen::MatrixXf& target);

    // Only uses pairs where closest_indices[i] != -1
    void compute_transform(
        const Eigen::MatrixXf& source,
        const Eigen::MatrixXf& target,
        const std::vector<int>& closest_indices,
        Eigen::Matrix2f& R,
        Eigen::Vector2f& t);

    float compute_error(
        const Eigen::MatrixXf& source,
        const Eigen::MatrixXf& target,
        const std::vector<int>& closest_indices);
};

} // namespace icp
