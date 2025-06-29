#include "src/VMapManager.h"
#include "cartographer/transform/transform.h"

using namespace std::chrono_literals;

namespace vrobot_backend
{

  VMapManager::VMapManager(rclcpp::Node::SharedPtr node) : node_(node)
  {
    init_service();
  }

  VMapManager::~VMapManager()
  {
    RCLCPP_INFO(kDefaultLogger, "VMapManager destructor called");
  }

  void VMapManager::init_service()
  {
    RCLCPP_INFO(kDefaultLogger, "VMapManager object configure.");
    initial_pose_publisher_ = node_->create_publisher<PoseWithCovarianceStamped>(
        kDefaultTopicInitialPose, 10);
    map_metadata_publisher_ =
        node_->create_publisher<MapMetaData>(kCurrentMapTopic, 10);

    srv_start_mapping_client_ = std::make_shared<ServiceClient<Trigger>>(
        kDefaultServiceStartMapping, node_);
    srv_switch_idle_client_ = std::make_shared<ServiceClient<Trigger>>(
        kDefaultServiceSwitchIdle, node_);
    srv_start_localization_client_ = std::make_shared<ServiceClient<LoadMap>>(
        kDefaultServiceStartLocalization, node_);
    srv_start_patch_mapping_client_ = std::make_shared<ServiceClient<PatchMap>>(
        kDefaultServiceStartPatchMapping, node_);
    srv_save_map_client_ =
        std::make_shared<ServiceClient<SaveMap>>(kDefaultServiceSaveMap, node_);

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_ =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    callback_group_executor_->add_callback_group(
        callback_group_, node_->get_node_base_interface());
    client_ = node_->create_client<cartographer_ros_msgs::srv::SubmapQuery>(
        kSubmapQueryServiceName, rmw_qos_profile_services_default,
        callback_group_);

    submap_list_subscriber_ = node_->create_subscription<SubmapList>(
        kSubmapListTopic, 10,
        std::bind(&VMapManager::submap_list_callback, this,
                  std::placeholders::_1));

    map_manager_state_subscriber_ = node_->create_subscription<MapManagerState>(
        kMapManagerStateTopic, 10,
        std::bind(&VMapManager::map_manager_state_callback, this,
                  std::placeholders::_1));
  }

  bool VMapManager::initial_pose(const double x, const double y,
                                 const double theta)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node_->now();
    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));
    initial_pose_publisher_->publish(initial_pose);

    return true;
  }

  bool VMapManager::start_mapping()
  {
    auto request = std::make_shared<Trigger::Request>();
    auto result = srv_start_mapping_client_->invoke(request, 5000ms);

    return result->success;
  }

  bool VMapManager::start_patch_mapping(std::string map_name)
  {
    auto request = std::make_shared<PatchMap::Request>();
    request->filename = map_name;
    auto result = srv_start_patch_mapping_client_->invoke(request, 5000ms);

    return result->status.code == StatusCode::OK;
  }

  bool VMapManager::save_map(std::string map_name)
  {
    auto request = std::make_shared<SaveMap::Request>();
    request->filename = map_name;
    auto result = srv_save_map_client_->invoke(request, 5000ms);

    return result->status.code == StatusCode::OK;
  }

  bool VMapManager::switch_idle()
  {
    auto request = std::make_shared<Trigger::Request>();
    auto result = srv_switch_idle_client_->invoke(request, 5000ms);

    return result->success;
  }

  bool VMapManager::switch_localization(std::string map_name)
  {
    auto request = std::make_shared<LoadMap::Request>();
    request->filename = map_name;
    auto result = srv_start_localization_client_->invoke(request, 5000ms);

    return result->status.code == StatusCode::OK;
  }

  void VMapManager::submap_list_callback(const SubmapList::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (node_->count_publishers(kSubmapListTopic) == 0 and
        map_manager_state_.mode != MapManagerState::MODE_MAPPING)
    {
      return;
    }

    // Keep track of submap IDs that don't appear in the message anymore.
    std::set<SubmapId> submap_ids_to_delete;
    for (const auto &pair : submap_slices_)
    {
      submap_ids_to_delete.insert(pair.first);
    }

    for (const auto &submap_msg : msg->submap)
    {
      const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
      submap_ids_to_delete.erase(id);
      if ((submap_msg.is_frozen && !true) || (!submap_msg.is_frozen && !true))
      {
        continue;
      }
      SubmapSlice &submap_slice = submap_slices_[id];
      submap_slice.pose = cartographer_ros::ToRigid3d(submap_msg.pose);
      submap_slice.metadata_version = submap_msg.submap_version;
      if (submap_slice.surface != nullptr &&
          submap_slice.version == submap_msg.submap_version)
      {
        continue;
      }

      auto fetched_textures = cartographer_ros::FetchSubmapTextures(
          id, client_, callback_group_executor_, 1s);
      if (fetched_textures == nullptr)
      {
        continue;
      }
      CHECK(!fetched_textures->textures.empty());
      submap_slice.version = fetched_textures->version;

      // We use the first texture only. By convention this is the highest
      // resolution texture and that is the one we want to use to construct the
      // map for ROS.
      const auto fetched_texture = fetched_textures->textures.begin();
      submap_slice.width = fetched_texture->width;
      submap_slice.height = fetched_texture->height;
      submap_slice.slice_pose = fetched_texture->slice_pose;
      submap_slice.resolution = fetched_texture->resolution;
      submap_slice.cairo_data.clear();
      submap_slice.surface = ::cartographer::io::DrawTexture(
          fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
          fetched_texture->width, fetched_texture->height,
          &submap_slice.cairo_data);
    }

    // Delete all submaps that didn't appear in the message.
    for (const auto &id : submap_ids_to_delete)
    {
      submap_slices_.erase(id);
    }

    last_frame_id_ = msg->header.frame_id;
  }

  cv::Mat VMapManager::get_current_map()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (submap_slices_.empty() || last_frame_id_.empty() ||
        map_manager_state_.mode != MapManagerState::MODE_MAPPING)
    {
      return cv::Mat();
    }
    auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);

    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height =
        cairo_image_surface_get_height(painted_slices.surface.get());
    const uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
        cairo_image_surface_get_data(painted_slices.surface.get()));

    map_metadata_.resolution = resolution_;
    map_metadata_.width = width;
    map_metadata_.height = height;
    map_metadata_.origin.position.x = -painted_slices.origin.x() * resolution_;
    map_metadata_.origin.position.y =
        (-height + painted_slices.origin.y()) * resolution_;
    map_metadata_publisher_->publish(map_metadata_);

    cv::Mat image(height, width, CV_8UC1);
    for (int y = height - 1; y >= 0; --y)
    {
      for (int x = 0; x < width; ++x)
      {
        const uint32_t packed = pixel_data[y * width + x];
        const unsigned char color = packed >> 16;
        const unsigned char observed = packed >> 8;
        const int value =
            observed == 0
                ? -1
                : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
        CHECK_LE(-1, value);
        CHECK_GE(100, value);
        if (value == -1)
        {
          image.at<uint8_t>(height - 1 - y, x) = 127;
        }
        else if (value == 0)
        {
          image.at<uint8_t>(height - 1 - y, x) = 255;
        }
        else
        {
          image.at<uint8_t>(height - 1 - y, x) = 255.0 - value / 100.0 * 255.0;
        }
      }
    }
    cv::Mat imflip;
    cv::flip(image, imflip, 0);
    return imflip;
  }

  void VMapManager::map_manager_state_callback(
      const MapManagerState::SharedPtr msg)
  {
    map_manager_state_ = *msg;
  }

  cv::Mat VMapManager::getMapFromDir(std::string pbstream_filename)
  {
    double resolution = 0.05;
    // Load the pbstream file
    cartographer::io::ProtoStreamReader reader(pbstream_filename);
    cartographer::io::ProtoStreamDeserializer deserializer(&reader);

    std::map<cartographer::mapping::SubmapId, cartographer::io::SubmapSlice>
        submap_slices;

    cartographer::mapping::ValueConversionTables conversion_tables;
    cartographer::io::DeserializeAndFillSubmapSlices(
        &deserializer, &submap_slices, &conversion_tables);

    auto painted_slices =
        cartographer::io::PaintSubmapSlices(submap_slices, resolution);

    // const auto &const_painted_slices = painted_slices;

    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height =
        cairo_image_surface_get_height(painted_slices.surface.get());
    const uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
        cairo_image_surface_get_data(painted_slices.surface.get()));

    map_metadata_.resolution = resolution_;
    map_metadata_.width = width;
    map_metadata_.height = height;
    map_metadata_.origin.position.x = -painted_slices.origin.x() * resolution_;
    map_metadata_.origin.position.y =
        (-height + painted_slices.origin.y()) * resolution_;
    map_metadata_publisher_->publish(map_metadata_);

    cv::Mat image(height, width, CV_8UC1);
    for (int y = height - 1; y >= 0; --y)
    {
      for (int x = 0; x < width; ++x)
      {
        const uint32_t packed = pixel_data[y * width + x];
        const unsigned char color = packed >> 16;
        const unsigned char observed = packed >> 8;
        const int value =
            observed == 0
                ? -1
                : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
        CHECK_LE(-1, value);
        CHECK_GE(100, value);
        if (value == -1)
        {
          image.at<uint8_t>(height - 1 - y, x) = 127;
        }
        else if (value == 0)
        {
          image.at<uint8_t>(height - 1 - y, x) = 255;
        }
        else
        {
          image.at<uint8_t>(height - 1 - y, x) = 255.0 - value / 100.0 * 255.0;
        }
      }
    }
    cv::Mat imflip;
    cv::flip(image, imflip, 0);
    return imflip;
  }
} // namespace vrobot_backend