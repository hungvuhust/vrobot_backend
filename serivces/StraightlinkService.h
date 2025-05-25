#pragma once

#include <drogon/HttpController.h>
#include <drogon/orm/Mapper.h>
#include <models/Straightlink.h>
using namespace drogon;
using namespace drogon::orm;
using namespace drogon_model::amr_01::amr_ros2;


class StraightlinkService : public drogon::HttpController<StraightlinkService> {
public:
  METHOD_LIST_BEGIN
  // GET    /straightlinks
  ADD_METHOD_TO(StraightlinkService::getAll, "/straightlinks", Get);
  // GET    /straightlinks/{1}
  ADD_METHOD_TO(StraightlinkService::getById, "/straightlinks/{1}", Get);
  // POST   /straightlinks
  ADD_METHOD_TO(StraightlinkService::create, "/straightlinks", Post);
  // PUT    /straightlinks/{1}
  ADD_METHOD_TO(StraightlinkService::update, "/straightlinks/{1}", Put);
  // DELETE /straightlinks/{1}
  ADD_METHOD_TO(StraightlinkService::erase, "/straightlinks/{1}", Delete);
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
  Mapper<Straightlink> straightlinkMapper_ =
      Mapper<Straightlink>(app().getDbClient());
};
