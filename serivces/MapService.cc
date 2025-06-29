#include "serivces/MapService.h"
#include <drogon/drogon.h>
#include <models/Map.h>


using namespace drogon;
using namespace drogon::orm;

void MapService::getAll(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback) {
  // asynchronous query
  mapMapper_.findAll(
      [callback](const std::vector<Map> &rows) {
        Json::Value arr(Json::arrayValue);
        for (const auto &r : rows) {
          arr.append(r.toJson());
        }
        auto resp = HttpResponse::newHttpJsonResponse(arr);
        callback(resp);
      },
      [callback](const DrogonDbException &err) {
        auto resp = HttpResponse::newHttpResponse();
        resp->setStatusCode(HttpStatusCode::k500InternalServerError);
        resp->setBody(err.base().what());
        callback(resp);
      });
}

void MapService::getById(
    const HttpRequestPtr &,
    std::function<void(const HttpResponsePtr &)> &&callback, int id) {
  mapMapper_.findByPrimaryKey(
      id,
      [callback](const Map &row) {
        callback(HttpResponse::newHttpJsonResponse(row.toJson()));
      },
      [callback](const DrogonDbException &err) {
        // Check if the error message contains "not found"
        std::string msg = err.base().what();
        if (msg.find("not found") != std::string::npos ||
            msg.find("NotFound") != std::string::npos) {
          callback(HttpResponse::newNotFoundResponse());
        } else {
          auto resp = HttpResponse::newHttpResponse();
          resp->setStatusCode(k500InternalServerError);
          resp->setBody(err.base().what());
          callback(resp);
        }
      });
}

void MapService::create(
    const HttpRequestPtr                          &req,
    std::function<void(const HttpResponsePtr &)> &&callback) {
  auto json = req->getJsonObject();
  if (!json) {
    auto resp = HttpResponse::newHttpResponse();
    resp->setStatusCode(HttpStatusCode::k400BadRequest);
    resp->setBody("Invalid JSON");
    callback(resp);
    return;
  }
  Map m;
  m.setMapName((*json)["map_name"].asString());
  m.setX((*json)["x"].asFloat());
  m.setY((*json)["y"].asFloat());
  m.setTheta((*json)["theta"].asFloat());
  m.setImage((*json)["image"].asString());
  m.setWidth((*json)["width"].asInt());
  m.setHeight((*json)["height"].asInt());
  m.setResolution((*json)["resolution"].asFloat());


  mapMapper_.insert(
      m,
      [callback](const Map &inserted) {
        auto resp = HttpResponse::newHttpJsonResponse(inserted.toJson());
        resp->setStatusCode(k201Created);
        callback(resp);
      },
      [callback](const DrogonDbException &err) {
        auto resp = HttpResponse::newHttpResponse();
        resp->setStatusCode(k500InternalServerError);
        resp->setBody(err.base().what());
        callback(resp);
      });
}

void MapService::update(const HttpRequestPtr                          &req,
                        std::function<void(const HttpResponsePtr &)> &&callback,
                        int                                            id) {
  auto json = req->getJsonObject();
  if (!json) {
    auto resp = HttpResponse::newHttpResponse();
    resp->setStatusCode(HttpStatusCode::k400BadRequest);
    resp->setBody("Invalid JSON");
    callback(resp);
    return;
  }
  mapMapper_.findByPrimaryKey(
      id,
      [json, callback](const Map &existingMap) mutable {
        Map updatedMap = existingMap;
        if ((*json).isMember("map_name"))
          updatedMap.setMapName((*json)["map_name"].asString());
        if ((*json).isMember("x"))
          updatedMap.setX((*json)["x"].asFloat());
        if ((*json).isMember("y"))
          updatedMap.setY((*json)["y"].asFloat());
        if ((*json).isMember("theta"))
          updatedMap.setTheta((*json)["theta"].asFloat());
        if ((*json).isMember("image"))
          updatedMap.setImage((*json)["image"].asString());
        if ((*json).isMember("width"))
          updatedMap.setWidth((*json)["width"].asInt());
        if ((*json).isMember("height"))
          updatedMap.setHeight((*json)["height"].asInt());
        if ((*json).isMember("resolution"))
          updatedMap.setResolution((*json)["resolution"].asFloat());
        drogon::orm::Mapper<Map> updateMapper(app().getDbClient("default"));
        updateMapper.update(
            updatedMap,
            [callback](const size_t updatedRows) {
              if (updatedRows > 0)
                callback(HttpResponse::newHttpResponse());
              else
                callback(HttpResponse::newNotFoundResponse());
            },
            [callback](const DrogonDbException &err) {
              auto resp = HttpResponse::newHttpResponse();
              resp->setStatusCode(k500InternalServerError);
              resp->setBody(err.base().what());
              callback(resp);
            });
      },
      [callback](const DrogonDbException &err) {
        std::string msg = err.base().what();
        if (msg.find("not found") != std::string::npos ||
            msg.find("NotFound") != std::string::npos) {
          callback(HttpResponse::newNotFoundResponse());
        } else {
          auto resp = HttpResponse::newHttpResponse();
          resp->setStatusCode(k500InternalServerError);
          resp->setBody(err.base().what());
          callback(resp);
        }
      });
}

void MapService::erase(const HttpRequestPtr &,
                       std::function<void(const HttpResponsePtr &)> &&callback,
                       int                                            id) {
  mapMapper_.deleteByPrimaryKey(
      id,
      [callback](const size_t deletedRows) {
        if (deletedRows > 0)
          callback(HttpResponse::newHttpResponse());
        else
          callback(HttpResponse::newNotFoundResponse());
      },
      [callback](const DrogonDbException &err) {
        auto resp = HttpResponse::newHttpResponse();
        resp->setStatusCode(k500InternalServerError);
        resp->setBody(err.base().what());
        callback(resp);
      });
}