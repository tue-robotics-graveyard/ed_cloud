#ifndef WORLD_UPDATE_SERVER_PLUGIN_H
#define WORLD_UPDATE_SERVER_PLUGIN_H

#include <ed/plugin.h>
#include <ed/WorldModelDelta.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <map>
#include <iterator>
#include <ed/uuid.h>
#include <ed/WorldModelDelta.h>
#include <ed/EntityUpdateInfo.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/datatypes.h>
#include <ed/GetWorldModel.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

class SyncServer : public ed::Plugin
{
public:
    SyncServer();

    bool GetWorldModel(ed::GetWorldModel::Request &req, ed::GetWorldModel::Response &res);

    virtual ~SyncServer();

    void configure(tue::Configuration config);

    void initialize();

    void updateRequestCallback(const ed::UpdateRequest& req);

    void createNewDelta();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    ed::WorldModelDelta combineDeltas(int rev_number);

private:

//    std::set<ed::UUID> modified_entities_current_delta;
//    std::map<ed::UUID, geo::ShapeConstPtr> shapes_current_delta;
//    std::map<ed::UUID, ed::ConvexHull2D> convex_hulls_current_delta;
//    std::map<ed::UUID, std::string> types_current_delta;
//    std::map<ed::UUID, geo::Pose3D> poses_current_delta;
//    std::set<ed::UUID> removed_entities_current_delta;

    // Contains the delta from the latest revision to the current world model state
    // (which is not yet added to deltaModels). A double buffer is used because the
    // delta is filled by a different thread than it is used.
    ed::UpdateRequest latest_delta_[2];
    bool has_new_delta[2];

    // Index in the double buffer
    int i_latest_delta_;

    // Signals whether the delta is currently being read (and therefore should not be writter to)
    bool using_delta_;

    // deltaModels is a circular buffer containing world model updates
    std::vector<ed::WorldModelDelta> deltaModels;

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

    ed::EntityUpdateInfo& addOrGetEntityUpdate(const ed::UUID& id, std::map<std::string, unsigned int>& ids, ed::WorldModelDelta& delta);
};

#endif // WORLD_UPDATE_SERVER_PLUGIN_H
