#include "serivces/CurvelinkService.h"

void CurvelinkService::getAll(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback) {
  auto        items = curvelinkMapper_.findAll();
  Json::Value ret(Json::arrayValue);
  for (const auto &item : items) {
    ret.append(item.toJson());
  }
  auto resp = HttpResponse::newHttpJsonResponse(ret);
  callback(resp);
}

void CurvelinkService::getById(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback, int id) {
  auto item = curvelinkMapper_.findByPrimaryKey(id);
  auto resp = HttpResponse::newHttpJsonResponse(item.toJson());
  callback(resp);
}

void CurvelinkService::create(
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
  Curvelink item(*json);
  curvelinkMapper_.insert(item);
  auto resp = HttpResponse::newHttpJsonResponse(item.toJson());
  callback(resp);
}

void CurvelinkService::update(
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
  Curvelink item = curvelinkMapper_.findByPrimaryKey(id);
  item.updateByJson(*json);
  curvelinkMapper_.update(item);
  auto resp = HttpResponse::newHttpJsonResponse(item.toJson());
  callback(resp);
}

void CurvelinkService::erase(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback, int id) {
  curvelinkMapper_.deleteByPrimaryKey(id);
  auto resp = HttpResponse::newHttpResponse();
  resp->setStatusCode(k204NoContent);
  callback(resp);
}
