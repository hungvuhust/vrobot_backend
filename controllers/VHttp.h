#pragma once

#include "src/VMapManager.h"
#include <drogon/HttpController.h>

using namespace drogon;
using namespace vrobot_backend;

class VHttp : public drogon::HttpController<VHttp> {
public:
  METHOD_LIST_BEGIN
  ADD_METHOD_TO(VHttp::mapping, "/mapping", Post);
  ADD_METHOD_TO(VHttp::save_map, "/save-map?map_name={1}", Post);
  ADD_METHOD_TO(VHttp::open_map, "/open-map?map_name={1}", Post);
  ADD_METHOD_TO(VHttp::idle, "/idle", Post);
  ADD_METHOD_TO(VHttp::set_initial_pose, "/initial-pose?x={1}&y={2}&theta={3}",
                Post);
  ADD_METHOD_TO(VHttp::get_map, "/get-map", Get);
  METHOD_LIST_END

  static void Initialize(rclcpp::Node::SharedPtr ros) {
    carto_cli_ = std::make_shared<VMapManager>(ros);
  }

  void idle(const HttpRequestPtr                          &req,
            std::function<void(const HttpResponsePtr &)> &&callback);

  void mapping(const HttpRequestPtr                          &req,
               std::function<void(const HttpResponsePtr &)> &&callback);

  void save_map(const HttpRequestPtr                          &req,
                std::function<void(const HttpResponsePtr &)> &&callback,
                std::string                                  &&map_name);

  void open_map(const HttpRequestPtr                          &req,
                std::function<void(const HttpResponsePtr &)> &&callback,
                std::string                                  &&map_name);

  void get_map(const HttpRequestPtr                          &req,
               std::function<void(const HttpResponsePtr &)> &&callback);

  void set_initial_pose(const HttpRequestPtr                          &req,
                        std::function<void(const HttpResponsePtr &)> &&callback,
                        double x, double y, double theta);

private:
  static std::shared_ptr<VMapManager> carto_cli_;
};
