#pragma once

#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros/time_conversion.h"
#include <cartographer_ros_msgs/msg/status_code.hpp>
#include <cartographer_ros_msgs/msg/submap_list.hpp>
#include <cartographer_ros_msgs/srv/submap_query.hpp>
#include <chrono>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <nav2_util/service_client.hpp>
#include <nav_msgs/msg/detail/map_meta_data__struct.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <vrobot_map_manager/msg/detail/map_manager_state__struct.hpp>
#include <vrobot_map_manager/msg/map_manager_state.hpp>
#include <vrobot_map_manager/srv/load_map.hpp>
#include <vrobot_map_manager/srv/patch_map.hpp>
#include <vrobot_map_manager/srv/save_map.hpp>

namespace vrobot_backend {

constexpr char kDefaultTopicInitialPose[]         = "/initialpose";
constexpr char kDefaultServiceStartMapping[]      = "/start_mapping";
constexpr char kDefaultServiceSwitchIdle[]        = "/switch_idle";
constexpr char kDefaultServiceStartLocalization[] = "/start_localization";
constexpr char kDefaultServiceStartPatchMapping[] = "/start_patch_mapping";
constexpr char kDefaultServiceSaveMap[]           = "/save_map";
constexpr char kSubmapListTopic[]                 = "/submap_list";
constexpr char kSubmapQueryServiceName[]          = "/submap_query";
constexpr char kMapManagerStateTopic[]            = "/map_manager/state";
constexpr char kCurrentMapTopic[]                 = "/map/current_map/info";

const rclcpp::Logger kDefaultLogger =
    rclcpp::get_logger("vrobot_cartographer_cli");

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;
using cartographer_ros_msgs::msg::StatusCode;
using cartographer_ros_msgs::msg::SubmapList;
using cartographer_ros_msgs::srv::SubmapQuery;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav2_util::ServiceClient;
using nav_msgs::msg::MapMetaData;
using std_srvs::srv::Trigger;
using vrobot_map_manager::msg::MapManagerState;
using vrobot_map_manager::srv::LoadMap;
using vrobot_map_manager::srv::PatchMap;
using vrobot_map_manager::srv::SaveMap;

using SubmapIdCartographer = ::cartographer::mapping::SubmapId;

class VMapManager {

public:
  VMapManager(rclcpp::Node::SharedPtr node);
  ~VMapManager();

public:
  cv::Mat get_current_map();

public:
  void init_service();

  bool initial_pose(const double x, const double y, const double theta);
  bool start_mapping();
  bool start_patch_mapping(std::string map_name);
  bool save_map(std::string map_name);
  bool switch_idle();
  bool switch_localization(std::string map_name);

private:
  void submap_list_callback(const SubmapList::SharedPtr msg);
  void map_manager_state_callback(const MapManagerState::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr
                                            initial_pose_publisher_{nullptr};
  rclcpp::Publisher<MapMetaData>::SharedPtr map_metadata_publisher_{nullptr};

  std::shared_ptr<ServiceClient<Trigger>> srv_start_mapping_client_{nullptr};
  std::shared_ptr<ServiceClient<Trigger>> srv_switch_idle_client_{nullptr};
  std::shared_ptr<ServiceClient<LoadMap>> srv_start_localization_client_{
      nullptr};
  std::shared_ptr<ServiceClient<PatchMap>> srv_start_patch_mapping_client_{
      nullptr};
  std::shared_ptr<ServiceClient<SaveMap>> srv_save_map_client_{nullptr};

  rclcpp::Subscription<MapManagerState>::SharedPtr
      map_manager_state_subscriber_{nullptr};

private:
  std::mutex                                           map_mutex_;
  rclcpp::Client<SubmapQuery>::SharedPtr               client_{nullptr};
  rclcpp::CallbackGroup::SharedPtr                     callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  rclcpp::Subscription<SubmapList>::SharedPtr submap_list_subscriber_{nullptr};
  std::map<SubmapId, SubmapSlice>             submap_slices_;
  std::string                                 last_frame_id_;
  cv::Mat                                     current_map_;
  MapManagerState                             map_manager_state_;
  MapMetaData                                 map_metadata_;
  double                                      resolution_ = 0.05;
};

} // namespace vrobot_backend