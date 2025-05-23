#include "src/VRobotInfo.h"
#include <mutex>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

namespace vrobot_backend {

VRobotInfo::VRobotInfo(rclcpp::Node::SharedPtr                         node,
                       std::function<void(const std::string &message)> callback)
    : node_(node), timer_callback_(callback) {
  Initialize();
}

void VRobotInfo::Initialize() {
  RCLCPP_INFO(kLogger, "VRobotInfo initialized.");
  scan_subscriber_ = node_->create_subscription<PointCloud2>(
      kScanMatcherTopic, rclcpp::SensorDataQoS(),
      std::bind(&VRobotInfo::scan_callback, this, std::placeholders::_1));
  tracked_pose_subscriber_ = node_->create_subscription<PoseStamped>(
      kTrackedPoseTopic, rclcpp::SensorDataQoS(),
      std::bind(&VRobotInfo::tracked_pose_callback, this,
                std::placeholders::_1));
  map_manager_state_subscriber_ = node_->create_subscription<MapManagerState>(
      kMapManagerStateTopic, rclcpp::SensorDataQoS(),
      std::bind(&VRobotInfo::map_manager_state_callback, this,
                std::placeholders::_1));
  current_map_subscriber_ = node_->create_subscription<MapMetaData>(
      kCurrentMapTopic, rclcpp::SensorDataQoS(),
      std::bind(&VRobotInfo::current_map_callback, this,
                std::placeholders::_1));

  timer_ = node_->create_wall_timer(
      1s, std::bind(&VRobotInfo::timer_callback, this));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void VRobotInfo::scan_callback(const PointCloud2::SharedPtr msg) {
  if (node_->count_publishers(kScanMatcherTopic) == 0) {
    return;
  }
  std::string x_data = "";
  std::string y_data = "";

  std::stringstream ss;

  for (size_t i = 0; i < msg->data.size(); i += msg->point_step * 3) {
    float x, y;
    memcpy(&x, &msg->data[i + msg->fields[0].offset], sizeof(float));
    memcpy(&y, &msg->data[i + msg->fields[1].offset], sizeof(float));
    ss.str("");
    ss << std::fixed << std::setprecision(2) << x;
    x_data += ss.str() + " ";
    ss.str("");
    ss << std::fixed << std::setprecision(2) << y;
    y_data += ss.str() + " ";
  }

  x_data.pop_back();
  y_data.pop_back();

  {
    std::lock_guard<std::mutex> lock(mutex_scan_);
    scan_json_["x"] = x_data;
    scan_json_["y"] = y_data;
  }
}

void VRobotInfo::tracked_pose_callback(const PoseStamped::SharedPtr msg) {
  if (node_->count_publishers(kTrackedPoseTopic) == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_robot_pose_);

  robot_pose_json_["position"]["x"]    = msg->pose.position.x;
  robot_pose_json_["position"]["y"]    = msg->pose.position.y;
  robot_pose_json_["position"]["z"]    = msg->pose.position.z;
  robot_pose_json_["orientation"]["w"] = msg->pose.orientation.w;
  robot_pose_json_["orientation"]["x"] = msg->pose.orientation.x;
  robot_pose_json_["orientation"]["y"] = msg->pose.orientation.y;
  robot_pose_json_["orientation"]["z"] = msg->pose.orientation.z;
}

void VRobotInfo::map_manager_state_callback(
    const MapManagerState::SharedPtr msg) {
  if (node_->count_publishers(kMapManagerStateTopic) == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_map_manager_state_);

  switch (msg->mode) {
  case MapManagerState::MODE_IDLE: map_manager_state_ = "idle"; break;
  case MapManagerState::MODE_MAPPING: map_manager_state_ = "mapping"; break;
  case MapManagerState::MODE_NAVIGATION:
    map_manager_state_ = "navigation";
    break;
  case MapManagerState::MODE_PATCH_MAPPING:
    map_manager_state_ = "patch_mapping";
    break;
  }
}

void VRobotInfo::current_map_callback(const MapMetaData::SharedPtr msg) {
  if (node_->count_publishers(kCurrentMapTopic) == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_map_manager_state_);
}

Json::Value VRobotInfo::get_robot_pose_json() {
  std::lock_guard<std::mutex> lock(mutex_robot_pose_);
  return robot_pose_json_;
}

Json::Value VRobotInfo::get_scan_json() {
  std::lock_guard<std::mutex> lock(mutex_scan_);
  return scan_json_;
}

void VRobotInfo::timer_callback() {
  auto robot_pose_json   = get_robot_pose_json();
  auto scan_json         = get_scan_json();
  auto map_manager_state = get_map_manager_state();

  Json::Value ret(Json::objectValue);
  ret["location"] = robot_pose_json;
  ret["laser"]    = scan_json;
  ret["mode"]     = map_manager_state;

  std::string message = ret.toStyledString();
  timer_callback_(message);
}

std::string VRobotInfo::get_map_manager_state() {
  std::lock_guard<std::mutex> lock(mutex_map_manager_state_);
  return map_manager_state_;
}

} // namespace vrobot_backend