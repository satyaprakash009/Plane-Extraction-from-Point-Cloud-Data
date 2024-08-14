#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

// Function to compute the plane from three points
Eigen::Vector4d ComputePlane(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p1;
    Eigen::Vector3d normal = v1.cross(v2);
    double a = normal[0], b = normal[1], c = normal[2];
    double d = -normal.dot(p1);
    return Eigen::Vector4d(a, b, c, d);
}

// RANSAC for plane fitting
std::tuple<Eigen::Vector4d, int> RANSACPlane(const std::vector<Eigen::Vector3d>& points, int max_iterations = 1000, double distance_threshold = 0.01) {
    Eigen::Vector4d best_plane;
    int max_inliers = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    for (int i = 0; i < max_iterations; ++i) {
        Eigen::Vector3d sample[3] = {points[dis(gen)], points[dis(gen)], points[dis(gen)]};
        Eigen::Vector4d plane = ComputePlane(sample[0], sample[1], sample[2]);

        Eigen::Vector3d normal = plane.head<3>();
        double d = plane[3];
        int inliers = 0;

        for (const auto& point : points) {
            double distance = std::abs(point.dot(normal) + d) / normal.norm();
            if (distance < distance_threshold) {
                inliers++;
            }
        }

        if (inliers > max_inliers) {
            max_inliers = inliers;
            best_plane = plane;
        }
    }

    return {best_plane, max_inliers};
}

int main() {
    // Example usage of RANSACPlane
    std::vector<Eigen::Vector3d> points = { /* Your point cloud data here */ };
    Eigen::Vector4d best_plane;
    int num_inliers;
    std::tie(best_plane, num_inliers) = RANSACPlane(points);

    std::cout << "Best plane: " << best_plane.transpose() << std::endl;
    std::cout << "Number of inliers: " << num_inliers << std::endl;

    return 0;
}
