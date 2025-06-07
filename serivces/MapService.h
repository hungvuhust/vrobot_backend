#pragma once

#include <drogon/HttpController.h>
#include <drogon/orm/Mapper.h>
#include <models/Map.h>

using namespace drogon;
using namespace drogon::orm;
using namespace drogon_model::amr_01::amr_ros2;


class MapService : public drogon::HttpController<MapService> {
public:
  METHOD_LIST_BEGIN
  // GET    /maps
  ADD_METHOD_TO(MapService::getAll, "/maps", Get);
  // GET    /maps/{1}
  ADD_METHOD_TO(MapService::getById, "/maps/{1}", Get);
  // POST   /maps
  ADD_METHOD_TO(MapService::create, "/maps",Post);
  // PUT    /maps/{1}
  ADD_METHOD_TO(MapService::update, "/maps/{1}", Put);
  // DELETE /maps/{1}
  ADD_METHOD_TO(MapService::erase, "/maps/{1}", Delete);
  METHOD_LIST_END

  void getAll(const HttpRequestPtr                          &req,
              std::function<void(const HttpResponsePtr &)> &&callback);
  void getById(const HttpRequestPtr                          &req,
               std::function<void(const HttpResponsePtr &)> &&callback, int id);

  void create(const HttpRequestPtr                          &req,
              std::function<void(const HttpResponsePtr &)> &&callback);

  void update(const HttpRequestPtr                          &req,
              std::function<void(const HttpResponsePtr &)> &&callback, int id);

  void erase(const HttpRequestPtr                          &req,
             std::function<void(const HttpResponsePtr &)> &&callback, int id);

private:
  Mapper<Map> mapMapper_ = Mapper<Map>(app().getDbClient());
};
