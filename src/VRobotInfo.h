#pragma once

#include <chrono>
#include <drogon/drogon.h>
#include <fstream>
#include <functional>
#include <json/value.h>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/convert.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vrobot_map_manager/msg/detail/map_manager_state__struct.hpp>
#include <vrobot_map_manager/msg/map_manager_state.hpp>

constexpr char kScanMatcherTopic[]     = "/scan_matched_points2";
constexpr char kTrackedPoseTopic[]     = "/tracked_pose";
constexpr char kMapManagerStateTopic[] = "/map_manager/state";
constexpr char kCurrentMapTopic[]      = "/map/current_map/info";

const rclcpp::Logger kLogger = rclcpp::get_logger("robot_info");

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::MapMetaData;
using sensor_msgs::msg::PointCloud2;
using vrobot_map_manager::msg::MapManagerState;

using namespace drogon;

namespace vrobot_backend {

class VRobotInfo {
public:
  VRobotInfo(rclcpp::Node::SharedPtr                         node,
             std::function<void(const std::string &message)> callback);
  ~VRobotInfo() = default;

  void Initialize();

public:
  Json::Value get_scan_json();
  Json::Value get_robot_pose_json();
  std::string get_map_manager_state();
  Json::Value get_current_map_metadata();

private:
  void scan_callback(const PointCloud2::SharedPtr msg);
  void tracked_pose_callback(const PoseStamped::SharedPtr msg);
  void map_manager_state_callback(const MapManagerState::SharedPtr msg);
  void timer_callback();
  void current_map_callback(const MapMetaData::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr                      node_;
  rclcpp::TimerBase::SharedPtr                 timer_;
  rclcpp::Subscription<PointCloud2>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<PoseStamped>::SharedPtr tracked_pose_subscriber_;
  rclcpp::Subscription<MapManagerState>::SharedPtr
                                               map_manager_state_subscriber_;
  rclcpp::Subscription<MapMetaData>::SharedPtr current_map_subscriber_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::function<void(const std::string &message)> timer_callback_;

private:
  std::mutex mutex_scan_;
  std::mutex mutex_robot_pose_;
  std::mutex mutex_map_manager_state_;

  Json::Value scan_json_;
  Json::Value robot_pose_json_;
  std::string map_manager_state_;
  MapMetaData current_map_metadata_;
};

} // namespace vrobot_backend