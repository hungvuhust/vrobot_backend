#include "VHttp.h"

std::shared_ptr<VMapManager> VHttp::carto_cli_ = nullptr;

void VHttp::idle(const HttpRequestPtr                          &req,
                 std::function<void(const HttpResponsePtr &)> &&callback) {
  auto result = carto_cli_->switch_idle();
  auto resp   = HttpResponse::newHttpResponse();
  if (result) {
    resp->setStatusCode(k200OK);
    resp->setBody("Idle mode activated successfully.");
  } else {
    resp->setStatusCode(k500InternalServerError);
    resp->setBody("Failed to activate idle mode.");
  }
  callback(resp);
}

void VHttp::mapping(const HttpRequestPtr                          &req,
                    std::function<void(const HttpResponsePtr &)> &&callback) {

  auto result = carto_cli_->start_mapping();
  auto resp   = HttpResponse::newHttpResponse();
  if (result) {
    resp->setStatusCode(k200OK);
    resp->setBody("Mapping started successfully.");
  } else {
    resp->setStatusCode(k500InternalServerError);
    resp->setBody("Failed to start mapping.");
  }
  callback(resp);
}

void VHttp::save_map(const HttpRequestPtr                          &req,
                     std::function<void(const HttpResponsePtr &)> &&callback,
                     std::string                                  &&map_name) {
  auto result = carto_cli_->save_map(map_name);
  auto resp   = HttpResponse::newHttpResponse();
  if (result) {
    resp->setStatusCode(k200OK);
    resp->setBody("Map saved successfully.");
  } else {
    resp->setStatusCode(k500InternalServerError);
    resp->setBody("Failed to save map.");
  }
  callback(resp);
}

void VHttp::open_map(const HttpRequestPtr                          &req,
                     std::function<void(const HttpResponsePtr &)> &&callback,
                     std::string                                  &&map_name) {
  auto result = carto_cli_->switch_localization(map_name);
  auto resp   = HttpResponse::newHttpResponse();
  if (result) {
    resp->setStatusCode(k200OK);
    resp->setBody("Map opened successfully.");
  } else {
    resp->setStatusCode(k500InternalServerError);
    resp->setBody("Failed to open map.");
  }
  callback(resp);
}

void VHttp::get_map(const HttpRequestPtr                          &req,
                    std::function<void(const HttpResponsePtr &)> &&callback) {

  cv::Mat map  = carto_cli_->get_current_map();
  auto    resp = HttpResponse::newHttpResponse();

  if (map.empty()) {
    resp->setStatusCode(k500InternalServerError);
    resp->setBody("Failed to get map.");
    callback(resp);
    return;
  }

  // Convert the map to a PNG image
  std::vector<uchar> buf;
  cv::imencode(".png", map, buf);
  std::string str(buf.begin(), buf.end());
  resp->setContentTypeCode(drogon::CT_IMAGE_PNG);
  resp->setBody(str);
  resp->setStatusCode(k200OK);

  callback(resp);
}

void VHttp::set_initial_pose(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback, double x, double y,
    double theta) {
  auto result = carto_cli_->initial_pose(x, y, theta);
  auto resp   = HttpResponse::newHttpResponse();
  if (result) {
    resp->setStatusCode(k200OK);
    resp->setBody("Initial pose set successfully.");
  } else {
    resp->setStatusCode(k500InternalServerError);
    resp->setBody("Failed to set initial pose.");
  }
  callback(resp);
}
