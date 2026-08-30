#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <hdl_global_localization/srv/query_global_localization.hpp>
#include <hdl_global_localization/srv/set_global_localization_engine.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/engines/global_localization_bbs.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>
#include <hdl_global_localization/ros_compat.hpp>

namespace hdl_global_localization {
class GlobalLocalizationNode final : public rclcpp::Node {
public:
  GlobalLocalizationNode() : rclcpp::Node("hdl_global_localization"), params_(this) {
    if (!set_engine(params_.param<std::string>("global_localization_engine", "FPFH_RANSAC"))) {
      throw std::runtime_error("unsupported global localization engine");
    }
    set_engine_service_ = create_service<srv::SetGlobalLocalizationEngine>(
      "/hdl_global_localization/set_engine",
      [this](const std::shared_ptr<srv::SetGlobalLocalizationEngine::Request> request,
             std::shared_ptr<srv::SetGlobalLocalizationEngine::Response>) {
        if (!set_engine(request->engine_name.data)) {
          RCLCPP_ERROR(get_logger(), "unsupported global localization engine: %s", request->engine_name.data.c_str());
        }
      });
    set_map_service_ = create_service<srv::SetGlobalMap>(
      "/hdl_global_localization/set_global_map",
      [this](const std::shared_ptr<srv::SetGlobalMap::Request> request,
             std::shared_ptr<srv::SetGlobalMap::Response>) { set_map(*request); });
    query_service_ = create_service<srv::QueryGlobalLocalization>(
      "/hdl_global_localization/query",
      [this](const std::shared_ptr<srv::QueryGlobalLocalization::Request> request,
             std::shared_ptr<srv::QueryGlobalLocalization::Response> response) { query(*request, *response); });
  }

private:
  static pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double resolution) {
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(static_cast<float>(resolution), static_cast<float>(resolution), static_cast<float>(resolution));
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*filtered);
    return filtered;
  }

  bool set_engine(const std::string& name) {
    if (name == "BBS") {
      engine_ = std::make_unique<GlobalLocalizationBBS>(params_);
    } else if (name == "FPFH_RANSAC") {
      engine_ = std::make_unique<GlobalLocalizationEngineFPFH_RANSAC>(params_);
    } else if (name == "FPFH_TEASER") {
      RCLCPP_ERROR(get_logger(), "FPFH_TEASER is unavailable: TEASER is not part of the production closure");
      return false;
    } else {
      return false;
    }
    if (global_map_) engine_->set_global_map(global_map_);
    return true;
  }

  void set_map(const srv::SetGlobalMap::Request& request) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(request.global_map, *cloud);
    global_map_ = downsample(cloud, params_.param<double>("globalmap_downsample_resolution", 0.5));
    globalmap_header_ = request.global_map.header;
    engine_->set_global_map(global_map_);
  }

  void query(const srv::QueryGlobalLocalization::Request& request,
             srv::QueryGlobalLocalization::Response& response) {
    if (!global_map_) {
      RCLCPP_WARN(get_logger(), "query rejected: no global map");
      return;
    }
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(request.cloud, *cloud);
    cloud = downsample(cloud, params_.param<double>("query_downsample_resolution", 0.5));
    const auto results = engine_->query(cloud, std::max<int64_t>(0, request.max_num_candidates));
    response.header = request.cloud.header;
    response.globalmap_header = globalmap_header_;
    for (const auto& result : results.results) {
      if (!result) continue;
      Eigen::Quaternionf quaternion(result->pose.linear());
      geometry_msgs::msg::Pose pose;
      pose.position.x = result->pose.translation().x();
      pose.position.y = result->pose.translation().y();
      pose.position.z = result->pose.translation().z();
      pose.orientation.x = quaternion.x(); pose.orientation.y = quaternion.y();
      pose.orientation.z = quaternion.z(); pose.orientation.w = quaternion.w();
      response.inlier_fractions.push_back(result->inlier_fraction);
      response.errors.push_back(result->error);
      response.poses.push_back(pose);
    }
  }

  ros::NodeHandle params_;
  std_msgs::msg::Header globalmap_header_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr global_map_;
  std::unique_ptr<GlobalLocalizationEngine> engine_;
  rclcpp::Service<srv::SetGlobalLocalizationEngine>::SharedPtr set_engine_service_;
  rclcpp::Service<srv::SetGlobalMap>::SharedPtr set_map_service_;
  rclcpp::Service<srv::QueryGlobalLocalization>::SharedPtr query_service_;
};
}  // namespace hdl_global_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hdl_global_localization::GlobalLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
