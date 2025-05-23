#include "VSocket.h"
#include <trantor/utils/Logger.h>

// Static member definitions
std::shared_ptr<vrobot_backend::VRobotInfo> VSocket::robot_info_ =
    nullptr; // Initialize the static member
std::set<WebSocketConnectionPtr> VSocket::clients_;
std::mutex                       VSocket::mutex_;

void VSocket::handleNewMessage(const WebSocketConnectionPtr &wsConnPtr,
                               std::string                 &&message,
                               const WebSocketMessageType   &type) {
  if (robot_info_ == nullptr) {
    LOG_ERROR << "robot_info_ is not initialized.";
    return;
  }
  // write your application logic here
  if (type == WebSocketMessageType::Text) {
    // Broadcast the message to all connected clients
    broadcastToClients(message);
  } else if (type == WebSocketMessageType::Binary) {
    LOG_INFO << "Received binary message";
  } else {
    LOG_WARN << "Unknown message type";
  }
}

void VSocket::handleNewConnection(const HttpRequestPtr         &req,
                                  const WebSocketConnectionPtr &wsConnPtr) {
  if (robot_info_ == nullptr) {
    LOG_ERROR << "robot_info_ is not initialized.";
    return;
  }
  // write your application logic here
  std::lock_guard<std::mutex> lock(mutex_);
  clients_.insert(wsConnPtr);
  LOG_INFO << "Client connected. Total: " << clients_.size();
}

void VSocket::handleConnectionClosed(const WebSocketConnectionPtr &wsConnPtr) {
  if (robot_info_ == nullptr) {
    LOG_ERROR << "robot_info_ is not initialized.";
    return;
  }
  // write your application logic here
  std::lock_guard<std::mutex> lock(mutex_);
  clients_.erase(wsConnPtr);
  LOG_INFO << "Client disconnected. Total: " << clients_.size();
}