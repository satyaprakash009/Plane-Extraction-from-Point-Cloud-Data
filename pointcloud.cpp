#include <iostream>
#include <Eigen/Dense>
#include <Open3D/Open3D.h>
#include <random>
#include <vector>

// Function to implement RANSAC for plane fitting
std::tuple<Eigen::Vector4d, int> RANSACPlane(const std::vector<Eigen::Vector3d>& points, int max_iterations = 1000, double distance_threshold = 0.01) {
    Eigen::Vector4d best_plane;
    int max_inliers = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    for (int i = 0; i < max_iterations; ++i) {
        // Randomly sample 3 points
        Eigen::Vector3d sample[3] = {points[dis(gen)], points[dis(gen)], points[dis(gen)]};
        Eigen::Vector3d v1 = sample[1] - sample[0];
        Eigen::Vector3d v2 = sample[2] - sample[0];
        Eigen::Vector3d normal = v1.cross(v2);
        double a = normal[0], b = normal[1], c = normal[2];
        double d = -normal.dot(sample[0]);

        // Calculate distances of all points to the plane
        int inliers = 0;
        for (const auto& point : points) {
            double distance = std::abs(point.dot(normal) + d) / normal.norm();
            if (distance < distance_threshold) {
                inliers++;
            }
        }

        if (inliers > max_inliers) {
            max_inliers = inliers;
            best_plane = Eigen::Vector4d(a, b, c, d);
        }
    }

    return {best_plane, max_inliers};
}

// Function to calculate the surface area of the plane using convex hull
double CalculatePlaneArea(const std::vector<Eigen::Vector3d>& inlier_points, const Eigen::Vector4d& plane) {
    Eigen::Vector3d normal = plane.head<3>();
    double d = plane[3];
    std::vector<Eigen::Vector3d> projected_points;

    for (const auto& point : inlier_points) {
        Eigen::Vector3d projected_point = point - (point.dot(normal) + d) / normal.dot(normal) * normal;
        projected_points.push_back(projected_point);
    }

    // Calculate the convex hull of the projected points in 2D (XY plane)
    std::vector<Eigen::Vector2d> projected_2d_points;
    for (const auto& point : projected_points) {
        projected_2d_points.push_back(Eigen::Vector2d(point[0], point[1]));
    }

    open3d::geometry::ConvexHull2D convex_hull(projected_2d_points);
    convex_hull.ComputeConvexHull();
    return convex_hull.Area();
}

int main() {
    try {
        std::string filename = "C:/Users/Aadii/Downloads/CSite1_orig-utm.pcd";

        // Read the point cloud
        auto pcd = std::make_shared<open3d::geometry::PointCloud>();
        if (open3d::io::ReadPointCloud(filename, *pcd)) {
            std::cout << "Successfully read " << filename << std::endl;
        } else {
            std::cerr << "Failed to read " << filename << std::endl;
            return 1;
        }

        std::vector<Eigen::Vector3d> points = pcd->points_;

        // Apply RANSAC to find the largest plane
        Eigen::Vector4d best_plane;
        int num_inliers;
        std::tie(best_plane, num_inliers) = RANSACPlane(points);

        std::cout << "Best plane equation: " << best_plane.transpose() << std::endl;
        std::cout << "Number of inliers: " << num_inliers << std::endl;

        // Extract inlier points
        Eigen::Vector3d normal = best_plane.head<3>();
        double d = best_plane[3];
        std::vector<Eigen::Vector3d> inlier_points;

        for (const auto& point : points) {
            double distance = std::abs(point.dot(normal) + d) / normal.norm();
            if (distance < 0.01) {
                inlier_points.push_back(point);
            }
        }

        // Calculate the surface area of the plane
        double plane_area = CalculatePlaneArea(inlier_points, best_plane);
        std::cout << "Plane surface area: " << plane_area << " square units" << std::endl;

        // Visualization using Open3D
        auto inlier_cloud = std::make_shared<open3d::geometry::PointCloud>();
        inlier_cloud->points_ = inlier_points;
        inlier_cloud->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));  // Red for plane

        open3d::visualization::Visualizer vis;
        vis.CreateVisualizerWindow("Point Cloud with Largest Plane", 1280, 720);
        vis.AddGeometry(pcd);
        vis.AddGeometry(inlier_cloud);
        vis.Run();
        vis.DestroyVisualizerWindow();

    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
