#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace trajectory_generation {

struct PlannerConfig {
  double resolution{0.05};
  double lower_x{-13.394};
  double lower_y{-12.079};
  double robot_radius{0.35};
  double desired_speed{2.0};
  int expected_map_width{400};
  int expected_map_height{400};
  int random_seed{-1};
};

struct PolynomialTrajectory {
  std::vector<float> coef_x;
  std::vector<float> coef_y;
  std::vector<float> duration;
};

struct PlanResult {
  std::vector<Eigen::Vector3d> raw_path;
  std::vector<Eigen::Vector3d> topological_path;
  std::vector<Eigen::Vector3d> smoothed_path;
  PolynomialTrajectory trajectory;
};

class PlannerCore {
 public:
  void configure(const PlannerConfig& config);
  void load_maps(const std::string& occupancy_path, const std::string& bev_path,
                 const std::string& topology_path);
  void set_dynamic_obstacles(const std::vector<Eigen::Vector3d>& points);
  [[nodiscard]] PlanResult plan(const Eigen::Vector3d& start,
                                const Eigen::Vector3d& goal) const;
  [[nodiscard]] bool path_collides(const std::vector<Eigen::Vector3d>& path) const;
  [[nodiscard]] bool is_ready() const;
  [[nodiscard]] double resolution() const { return config_.resolution; }

 private:
  [[nodiscard]] cv::Mat combined_occupancy() const;
  [[nodiscard]] std::optional<cv::Point> world_to_grid(const Eigen::Vector3d& point) const;
  [[nodiscard]] Eigen::Vector3d grid_to_world(const cv::Point& point) const;
  [[nodiscard]] std::optional<cv::Point> nearest_free(const cv::Mat& occupancy,
                                                       const cv::Point& point) const;
  [[nodiscard]] std::vector<Eigen::Vector3d> astar(const cv::Mat& occupancy,
                                                    const cv::Point& start,
                                                    const cv::Point& goal) const;
  [[nodiscard]] std::vector<Eigen::Vector3d> topological_search(
      const cv::Mat& occupancy, const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal) const;
  [[nodiscard]] std::vector<Eigen::Vector3d> shortcut_path(
      const cv::Mat& occupancy, const std::vector<Eigen::Vector3d>& path) const;
  [[nodiscard]] PolynomialTrajectory generate_reference(
      const std::vector<Eigen::Vector3d>& path) const;
  [[nodiscard]] bool line_free(const cv::Mat& occupancy, const cv::Point& first,
                               const cv::Point& second) const;

  PlannerConfig config_;
  cv::Mat static_occupancy_;
  cv::Mat bev_map_;
  cv::Mat topology_map_;
  cv::Mat dynamic_occupancy_;
};

}  // namespace trajectory_generation
