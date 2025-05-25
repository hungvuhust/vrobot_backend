#pragma once

#include <drogon/HttpController.h>
#include <drogon/orm/Mapper.h>
#include <models/Node.h>
using namespace drogon;
using namespace drogon::orm;
using namespace drogon_model::amr_01::amr_ros2;


class NodeService : public drogon::HttpController<NodeService> {
public:
  METHOD_LIST_BEGIN
  // GET    /nodes
  ADD_METHOD_TO(NodeService::getAll, "/nodes", Get);
  // GET    /nodes/{1}
  ADD_METHOD_TO(NodeService::getById, "/nodes/{1}", Get);
  // POST   /nodes
  ADD_METHOD_TO(NodeService::create, "/nodes", Post);
  // PUT    /nodes/{1}
  ADD_METHOD_TO(NodeService::update, "/nodes/{1}", Put);
  // DELETE /nodes/{1}
  ADD_METHOD_TO(NodeService::erase, "/nodes/{1}", Delete);
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
  Mapper<Node> nodeMapper_ = Mapper<Node>(app().getDbClient());
};
