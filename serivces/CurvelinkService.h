#pragma once

#include <drogon/HttpController.h>
#include <drogon/orm/Mapper.h>
#include <models/Curvelink.h>
using namespace drogon;
using namespace drogon::orm;
using namespace drogon_model::vrobot;

class CurvelinkService : public drogon::HttpController<CurvelinkService> {
public:
  METHOD_LIST_BEGIN
  // GET    /curvelinks
  ADD_METHOD_TO(CurvelinkService::getAll, "/curvelinks", Get);
  // GET    /curvelinks/{1}
  ADD_METHOD_TO(CurvelinkService::getById, "/curvelinks/{1}", Get);
  // POST   /curvelinks
  ADD_METHOD_TO(CurvelinkService::create, "/curvelinks", Post);
  // PUT    /curvelinks/{1}
  ADD_METHOD_TO(CurvelinkService::update, "/curvelinks/{1}", Put);
  // DELETE /curvelinks/{1}
  ADD_METHOD_TO(CurvelinkService::erase, "/curvelinks/{1}", Delete);
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
  Mapper<Curvelink> curvelinkMapper_ = Mapper<Curvelink>(app().getDbClient());
};
