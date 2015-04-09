#ifndef WORLD_UPDATE_SERVER_PLUGIN_H
#define WORLD_UPDATE_SERVER_PLUGIN_H

#include <ed/plugin.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <map>
#include <iterator>
#include <ed/uuid.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/datatypes.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "ed_cloud/WorldModelDelta.h"
#include "ed_cloud/WorldModelDelta.h"
#include "ed_cloud/EntityUpdateInfo.h"
#include "ed_cloud/GetWorldModel.h"

class SyncServer : public ed::Plugin
{
public:
    SyncServer();

    bool GetWorldModel(ed_cloud::GetWorldModel::Request &req, ed_cloud::GetWorldModel::Response &res);

    virtual ~SyncServer();

    void initialize(ed::InitData& init);

    void addDelta(const ed::UpdateRequest& req);

   // void createNewDelta();

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

    ed_cloud::WorldModelDelta combineDeltas(int rev_number);

private:

    // deltaModels is a circular buffer containing world model updates
    std::vector<ed::UpdateRequest> deltaModels;

    // i_delta_models_start_ stores the index to the *earliest* delta in the buffer
    unsigned int i_delta_models_start_;

    // Maximum delta model buffer size
    unsigned int max_num_delta_models_;

    int current_rev_number;
    ros::CallbackQueue cb_queue_;
    ros::ServiceServer srv_get_world_;

    // Pointer to the latest world model state
    const ed::WorldModel* world_;

    // Keeps track of the latest world revision in which an entity was changed.
    // For example, if index 10 has value 5, it means that the entity with index 10
    // was last changed in revision 5.
    std::vector<unsigned int> entity_server_revisions_;

    std::pair<ed_cloud::EntityUpdateInfo&, bool> addOrGetEntityUpdate(const ed::UUID& id, std::map<std::string, unsigned int>& ids, ed_cloud::WorldModelDelta& delta);
};

#endif // WORLD_UPDATE_SERVER_PLUGIN_H
