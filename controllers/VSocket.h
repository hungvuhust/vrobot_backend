#pragma once

#include "src/VRobotInfo.h"
#include <drogon/WebSocketController.h>

using namespace drogon;

class VSocket : public drogon::WebSocketController<VSocket> {
public:
  void handleNewMessage(const WebSocketConnectionPtr &, std::string &&,
                        const WebSocketMessageType &) override;
  void handleNewConnection(const HttpRequestPtr &,
                           const WebSocketConnectionPtr &) override;
  void handleConnectionClosed(const WebSocketConnectionPtr &) override;

  WS_PATH_LIST_BEGIN
  WS_PATH_ADD("/ws");
  WS_PATH_LIST_END

  static void Initialize(rclcpp::Node::SharedPtr node) {
    robot_info_ =
        std::make_shared<vrobot_backend::VRobotInfo>(node, broadcastToClients);
  }

private:
  static void broadcastToClients(const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &conn : clients_) {
      if (conn->connected()) {
        conn->send(message);
      }
    }
  }

  static std::shared_ptr<vrobot_backend::VRobotInfo> robot_info_;
  static std::set<WebSocketConnectionPtr>            clients_;
  static std::mutex                                  mutex_;
};
