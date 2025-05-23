#include "serivces/NodeService.h"

void NodeService::getAll(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback) {
  auto        nodes = nodeMapper_.findAll();
  Json::Value ret(Json::arrayValue);
  for (const auto &node : nodes) {
    ret.append(node.toJson());
  }
  auto resp = HttpResponse::newHttpJsonResponse(ret);
  callback(resp);
}

void NodeService::getById(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback, int id) {
  auto node = nodeMapper_.findByPrimaryKey(id);
  auto resp = HttpResponse::newHttpJsonResponse(node.toJson());
  callback(resp);
}

void NodeService::create(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback) {
  auto json = req->getJsonObject();
  if (!json) {
    auto resp = HttpResponse::newHttpResponse();
    resp->setStatusCode(k400BadRequest);
    resp->setBody("Invalid JSON");
    callback(resp);
    return;
  }
  Node node(*json);
  nodeMapper_.insert(node);
  auto resp = HttpResponse::newHttpJsonResponse(node.toJson());
  callback(resp);
}

void NodeService::update(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback, int id) {
  auto json = req->getJsonObject();
  if (!json) {
    auto resp = HttpResponse::newHttpResponse();
    resp->setStatusCode(k400BadRequest);
    resp->setBody("Invalid JSON");
    callback(resp);
    return;
  }
  Node node = nodeMapper_.findByPrimaryKey(id);
  node.updateByJson(*json);
  nodeMapper_.update(node);
  auto resp = HttpResponse::newHttpJsonResponse(node.toJson());
  callback(resp);
}

void NodeService::erase(const HttpRequestPtr                          &req,
                        std::function<void(const HttpResponsePtr &)> &&callback,
                        int                                            id) {
  nodeMapper_.deleteByPrimaryKey(id);
  auto resp = HttpResponse::newHttpResponse();
  resp->setStatusCode(k204NoContent);
  callback(resp);
}
