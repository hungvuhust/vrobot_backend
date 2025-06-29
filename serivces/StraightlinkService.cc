#include "serivces/StraightlinkService.h"

void StraightlinkService::getAll(
    const HttpRequestPtr &req,
    std::function<void(const HttpResponsePtr &)> &&callback)
{
  auto items = straightlinkMapper_.findAll();
  Json::Value ret(Json::arrayValue);
  for (const auto &item : items)
  {
    ret.append(item.toJson());
  }
  auto resp = HttpResponse::newHttpJsonResponse(ret);
  callback(resp);
}

void StraightlinkService::getById(
    const HttpRequestPtr &req,
    std::function<void(const HttpResponsePtr &)> &&callback, int id)
{
  auto item = straightlinkMapper_.findByPrimaryKey(id);
  auto resp = HttpResponse::newHttpJsonResponse(item.toJson());
  callback(resp);
}

void StraightlinkService::create(
    const HttpRequestPtr &req,
    std::function<void(const HttpResponsePtr &)> &&callback)
{
  auto json = req->getJsonObject();
  if (!json)
  {
    auto resp = HttpResponse::newHttpResponse();
    resp->setStatusCode(k400BadRequest);
    resp->setBody("Invalid JSON");
    callback(resp);
    return;
  }
  Straightlink item(*json);
  straightlinkMapper_.insert(item);
  auto resp = HttpResponse::newHttpJsonResponse(item.toJson());
  callback(resp);
}

void StraightlinkService::update(
    const HttpRequestPtr &req,
    std::function<void(const HttpResponsePtr &)> &&callback, int id)
{
  auto json = req->getJsonObject();
  if (!json)
  {
    auto resp = HttpResponse::newHttpResponse();
    resp->setStatusCode(k400BadRequest);
    resp->setBody("Invalid JSON");
    callback(resp);
    return;
  }
  Straightlink item = straightlinkMapper_.findByPrimaryKey(id);
  item.updateByJson(*json);
  straightlinkMapper_.update(item);
  auto resp = HttpResponse::newHttpJsonResponse(item.toJson());
  callback(resp);
}

void StraightlinkService::erase(
    const HttpRequestPtr &req,
    std::function<void(const HttpResponsePtr &)> &&callback, int id)
{
  straightlinkMapper_.deleteByPrimaryKey(id);
  auto resp = HttpResponse::newHttpResponse();
  resp->setStatusCode(k204NoContent);
  callback(resp);
}
