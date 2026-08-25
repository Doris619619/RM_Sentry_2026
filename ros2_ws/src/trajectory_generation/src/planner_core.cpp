#include "trajectory_generation/planner_core.hpp"

#include <Eigen/LU>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>

namespace trajectory_generation {
namespace {

constexpr int kOccupied = 255;

struct QueueItem {
  double score;
  cv::Point point;
  bool operator<(const QueueItem& other) const { return score > other.score; }
};

cv::Mat first_channel(const cv::Mat& input) {
  if (input.empty()) return {};
  if (input.channels() == 1) return input.clone();
  std::vector<cv::Mat> channels;
  cv::split(input, channels);
  return channels.front();
}

double distance(const cv::Point& first, const cv::Point& second) {
  return std::hypot(static_cast<double>(first.x - second.x),
                    static_cast<double>(first.y - second.y));
}

}  // namespace

void PlannerCore::configure(const PlannerConfig& config) {
  if (config.resolution <= 0.0 || config.robot_radius < 0.0 || config.desired_speed <= 0.0 ||
      config.expected_map_width <= 0 || config.expected_map_height <= 0) {
    throw std::invalid_argument("planner resolution, radius, and desired speed must be valid");
  }
  config_ = config;
}

void PlannerCore::load_maps(const std::string& occupancy_path, const std::string& bev_path,
                            const std::string& topology_path) {
  const auto occupancy = first_channel(cv::imread(occupancy_path, cv::IMREAD_UNCHANGED));
  bev_map_ = first_channel(cv::imread(bev_path, cv::IMREAD_UNCHANGED));
  topology_map_ = first_channel(cv::imread(topology_path, cv::IMREAD_UNCHANGED));
  if (occupancy.empty() || bev_map_.empty() || topology_map_.empty()) {
    throw std::runtime_error("failed to load one or more global-planning maps");
  }
  if (occupancy.size() != bev_map_.size() || occupancy.size() != topology_map_.size()) {
    throw std::runtime_error("occupancy, BEV, and topology maps must have identical dimensions");
  }
  if (occupancy.cols != config_.expected_map_width || occupancy.rows != config_.expected_map_height) {
    throw std::runtime_error("global-planning map dimensions do not match configured expected width/height");
  }
  cv::threshold(occupancy, static_occupancy_, 10, kOccupied, cv::THRESH_BINARY);
  const int radius_cells = static_cast<int>(std::ceil(config_.robot_radius / config_.resolution));
  if (radius_cells > 0) {
    const int diameter = radius_cells * 2 + 1;
    const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {diameter, diameter});
    cv::dilate(static_occupancy_, static_occupancy_, kernel);
  }
  dynamic_occupancy_ = cv::Mat::zeros(static_occupancy_.size(), CV_8UC1);
}

void PlannerCore::set_dynamic_obstacles(const std::vector<Eigen::Vector3d>& points) {
  if (!is_ready()) return;
  dynamic_occupancy_ = cv::Mat::zeros(static_occupancy_.size(), CV_8UC1);
  for (const auto& point : points) {
    const auto index = world_to_grid(point);
    if (index) dynamic_occupancy_.at<std::uint8_t>(index->y, index->x) = kOccupied;
  }
  const int radius_cells = static_cast<int>(std::ceil(config_.robot_radius / config_.resolution));
  if (radius_cells > 0) {
    const int diameter = radius_cells * 2 + 1;
    const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {diameter, diameter});
    cv::dilate(dynamic_occupancy_, dynamic_occupancy_, kernel);
  }
}

bool PlannerCore::is_ready() const { return !static_occupancy_.empty(); }

cv::Mat PlannerCore::combined_occupancy() const {
  cv::Mat combined;
  cv::bitwise_or(static_occupancy_, dynamic_occupancy_, combined);
  return combined;
}

std::optional<cv::Point> PlannerCore::world_to_grid(const Eigen::Vector3d& point) const {
  if (!is_ready()) return std::nullopt;
  const int x = static_cast<int>(std::floor((point.x() - config_.lower_x) / config_.resolution));
  const int world_y = static_cast<int>(std::floor((point.y() - config_.lower_y) / config_.resolution));
  const cv::Point grid{x, static_occupancy_.rows - 1 - world_y};
  if (grid.x < 0 || grid.y < 0 || grid.x >= static_occupancy_.cols || grid.y >= static_occupancy_.rows) {
    return std::nullopt;
  }
  return grid;
}

Eigen::Vector3d PlannerCore::grid_to_world(const cv::Point& point) const {
  return {config_.lower_x + (point.x + 0.5) * config_.resolution,
          config_.lower_y + (static_occupancy_.rows - 1 - point.y + 0.5) * config_.resolution,
          0.0};
}

std::optional<cv::Point> PlannerCore::nearest_free(const cv::Mat& occupancy,
                                                    const cv::Point& point) const {
  if (point.x >= 0 && point.y >= 0 && point.x < occupancy.cols && point.y < occupancy.rows &&
      occupancy.at<std::uint8_t>(point.y, point.x) == 0) {
    return point;
  }
  const int maximum = std::max(2, static_cast<int>(std::ceil(1.0 / config_.resolution)));
  for (int radius = 1; radius <= maximum; ++radius) {
    for (int y = point.y - radius; y <= point.y + radius; ++y) {
      for (int x = point.x - radius; x <= point.x + radius; ++x) {
        if (std::abs(x - point.x) != radius && std::abs(y - point.y) != radius) continue;
        if (x >= 0 && y >= 0 && x < occupancy.cols && y < occupancy.rows &&
            occupancy.at<std::uint8_t>(y, x) == 0) {
          return cv::Point{x, y};
        }
      }
    }
  }
  return std::nullopt;
}

std::vector<Eigen::Vector3d> PlannerCore::astar(const cv::Mat& occupancy, const cv::Point& start,
                                                 const cv::Point& goal) const {
  const int width = occupancy.cols;
  const int total = width * occupancy.rows;
  auto key = [width](const cv::Point& point) { return point.y * width + point.x; };
  std::vector<double> cost(total, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total, -1);
  std::priority_queue<QueueItem> open;
  cost[key(start)] = 0.0;
  open.push({distance(start, goal), start});
  constexpr int directions[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1},
                                    {1, -1}, {-1, 1}, {-1, -1}};
  while (!open.empty()) {
    const auto current = open.top().point;
    open.pop();
    if (current == goal) break;
    const auto current_cost = cost[key(current)];
    for (const auto& direction : directions) {
      const cv::Point next{current.x + direction[0], current.y + direction[1]};
      if (next.x < 0 || next.y < 0 || next.x >= width || next.y >= occupancy.rows ||
          occupancy.at<std::uint8_t>(next.y, next.x) != 0) continue;
      if (direction[0] != 0 && direction[1] != 0 &&
          (occupancy.at<std::uint8_t>(current.y, next.x) != 0 ||
           occupancy.at<std::uint8_t>(next.y, current.x) != 0)) continue;
      const double next_cost = current_cost + std::hypot(direction[0], direction[1]);
      if (next_cost < cost[key(next)]) {
        cost[key(next)] = next_cost;
        parent[key(next)] = key(current);
        open.push({next_cost + distance(next, goal), next});
      }
    }
  }
  if (start != goal && parent[key(goal)] < 0) return {};
  std::vector<Eigen::Vector3d> path;
  for (int current = key(goal); current >= 0; current = parent[current]) {
    path.push_back(grid_to_world({current % width, current / width}));
    if (current == key(start)) break;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<Eigen::Vector3d> PlannerCore::topological_search(const cv::Mat& occupancy,
                                                              const Eigen::Vector3d& start,
                                                              const Eigen::Vector3d& goal) const {
  const auto start_index = world_to_grid(start);
  const auto goal_index = world_to_grid(goal);
  if (!start_index || !goal_index) return {};
  const auto valid_start = nearest_free(occupancy, *start_index);
  const auto valid_goal = nearest_free(occupancy, *goal_index);
  if (!valid_start || !valid_goal) return {};
  // The legacy planner creates alternative topological candidates before choosing one.
  // Keep a deterministic candidate set here: direct A* plus obstacle-biased A* maps.
  std::vector<std::vector<Eigen::Vector3d>> candidates;
  candidates.push_back(astar(occupancy, *valid_start, *valid_goal));
  std::mt19937 generator(config_.random_seed >= 0 ? config_.random_seed : std::random_device{}());
  for (int attempt = 0; attempt < 3; ++attempt) {
    cv::Mat biased = occupancy.clone();
    const int count = std::max(1, biased.rows * biased.cols / 2000);
    for (int i = 0; i < count; ++i) {
      const int x = static_cast<int>(generator() % static_cast<unsigned>(biased.cols));
      const int y = static_cast<int>(generator() % static_cast<unsigned>(biased.rows));
      if (biased.at<std::uint8_t>(y, x) == 0 && distance({x, y}, *valid_start) > 8.0 &&
          distance({x, y}, *valid_goal) > 8.0) biased.at<std::uint8_t>(y, x) = kOccupied;
    }
    const auto candidate = astar(biased, *valid_start, *valid_goal);
    if (!candidate.empty()) candidates.push_back(candidate);
  }
  auto length = [](const std::vector<Eigen::Vector3d>& path) {
    double value = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) value += (path[i] - path[i - 1]).head<2>().norm();
    return value;
  };
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [](const auto& candidate) { return candidate.empty(); }),
                   candidates.end());
  if (candidates.empty()) return {};
  return *std::min_element(candidates.begin(), candidates.end(),
                           [&length](const auto& first, const auto& second) {
                             return length(first) < length(second);
                           });
}

bool PlannerCore::line_free(const cv::Mat& occupancy, const cv::Point& first,
                            const cv::Point& second) const {
  const int steps = std::max(std::abs(second.x - first.x), std::abs(second.y - first.y));
  for (int step = 0; step <= steps; ++step) {
    const double ratio = steps == 0 ? 0.0 : static_cast<double>(step) / steps;
    const cv::Point point{static_cast<int>(std::lround(first.x + ratio * (second.x - first.x))),
                          static_cast<int>(std::lround(first.y + ratio * (second.y - first.y)))};
    if (point.x < 0 || point.y < 0 || point.x >= occupancy.cols || point.y >= occupancy.rows ||
        occupancy.at<std::uint8_t>(point.y, point.x) != 0) return false;
  }
  return true;
}

std::vector<Eigen::Vector3d> PlannerCore::shortcut_path(
    const cv::Mat& occupancy, const std::vector<Eigen::Vector3d>& path) const {
  if (path.size() < 3) return path;
  std::vector<Eigen::Vector3d> result{path.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t furthest = anchor + 1;
    const auto anchor_index = world_to_grid(path[anchor]);
    for (std::size_t candidate = anchor + 2; candidate < path.size(); ++candidate) {
      const auto candidate_index = world_to_grid(path[candidate]);
      if (anchor_index && candidate_index && line_free(occupancy, *anchor_index, *candidate_index)) {
        furthest = candidate;
      } else {
        break;
      }
    }
    result.push_back(path[furthest]);
    anchor = furthest;
  }
  return result;
}

PolynomialTrajectory PlannerCore::generate_reference(const std::vector<Eigen::Vector3d>& path) const {
  PolynomialTrajectory result;
  if (path.size() < 2) return result;
  const Eigen::Index count = static_cast<Eigen::Index>(path.size() - 1);
  Eigen::VectorXd time(count);
  for (Eigen::Index index = 0; index < count; ++index) {
    time(index) = std::max(0.01, (path[index + 1] - path[index]).head<2>().norm() / config_.desired_speed);
    result.duration.push_back(static_cast<float>(time(index)));
  }
  Eigen::MatrixXd system = Eigen::MatrixXd::Zero(count + 1, count + 1);
  Eigen::VectorXd rhs_x = Eigen::VectorXd::Zero(count + 1);
  Eigen::VectorXd rhs_y = Eigen::VectorXd::Zero(count + 1);
  system(0, 0) = 1.0;
  system(count, count) = 1.0;
  for (Eigen::Index index = 1; index < count; ++index) {
    system(index, index - 1) = time(index - 1);
    system(index, index) = 2.0 * (time(index - 1) + time(index));
    system(index, index + 1) = time(index);
    rhs_x(index) = 6.0 * ((path[index + 1].x() - path[index].x()) / time(index) -
                          (path[index].x() - path[index - 1].x()) / time(index - 1));
    rhs_y(index) = 6.0 * ((path[index + 1].y() - path[index].y()) / time(index) -
                          (path[index].y() - path[index - 1].y()) / time(index - 1));
  }
  const Eigen::VectorXd second_x = system.fullPivLu().solve(rhs_x);
  const Eigen::VectorXd second_y = system.fullPivLu().solve(rhs_y);
  for (Eigen::Index index = 0; index < count; ++index) {
    const double duration = time(index);
    result.coef_x.push_back(static_cast<float>((second_x(index + 1) - second_x(index)) / (6.0 * duration)));
    result.coef_x.push_back(static_cast<float>(second_x(index) / 2.0));
    result.coef_x.push_back(static_cast<float>((path[index + 1].x() - path[index].x()) / duration -
                                               duration * (2.0 * second_x(index) + second_x(index + 1)) / 6.0));
    result.coef_x.push_back(static_cast<float>(path[index].x()));
    result.coef_y.push_back(static_cast<float>((second_y(index + 1) - second_y(index)) / (6.0 * duration)));
    result.coef_y.push_back(static_cast<float>(second_y(index) / 2.0));
    result.coef_y.push_back(static_cast<float>((path[index + 1].y() - path[index].y()) / duration -
                                               duration * (2.0 * second_y(index) + second_y(index + 1)) / 6.0));
    result.coef_y.push_back(static_cast<float>(path[index].y()));
  }
  return result;
}

PlanResult PlannerCore::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) const {
  if (!is_ready()) throw std::logic_error("planner maps are not loaded");
  const auto occupancy = combined_occupancy();
  PlanResult result;
  const auto start_index = world_to_grid(start);
  const auto goal_index = world_to_grid(goal);
  if (!start_index || !goal_index) return result;
  const auto valid_start = nearest_free(occupancy, *start_index);
  const auto valid_goal = nearest_free(occupancy, *goal_index);
  if (!valid_start || !valid_goal) return result;
  result.raw_path = astar(occupancy, *valid_start, *valid_goal);
  result.topological_path = topological_search(occupancy, grid_to_world(*valid_start), grid_to_world(*valid_goal));
  if (result.topological_path.empty()) result.topological_path = result.raw_path;
  result.smoothed_path = shortcut_path(occupancy, result.topological_path);
  result.trajectory = generate_reference(result.smoothed_path);
  return result;
}

bool PlannerCore::path_collides(const std::vector<Eigen::Vector3d>& path) const {
  if (path.empty()) return false;
  const auto occupancy = combined_occupancy();
  for (const auto& point : path) {
    const auto index = world_to_grid(point);
    if (!index || occupancy.at<std::uint8_t>(index->y, index->x) != 0) return true;
  }
  return false;
}

}  // namespace trajectory_generation
