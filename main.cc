#include "controllers/VHttp.h"
#include "controllers/VSocket.h"
#include <drogon/HttpAppFramework.h>

using namespace drogon;

int main(int argc, char *argv[]) {
  // --- 1. Khởi tạo ROS2 ---
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("vrobot_backend");

  // --- 2. Khởi tao VMapManager ---
  VHttp::Initialize(node);
  VSocket::Initialize(node);

  // -- 3. Khởi tạo ros_thread ---
  auto ros_thread = std::thread([&node]() { rclcpp::spin(node); });

  // --- 4. Khởi động WebSocket Server của Drogon ---
  drogon::app().loadConfigFile(
      "/home/boot/ros2_ws/src/vrobot_backend/config.json");
  drogon::app().run(); // Blocking call

  // --- 5. Dọn dẹp ---
  rclcpp::shutdown();

  return 0;
}
