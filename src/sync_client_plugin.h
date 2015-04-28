#ifndef WORLD_UPDATE_CLIENT_PLUGIN_H
#define WORLD_UPDATE_CLIENT_PLUGIN_H

#include <ros/callback_queue.h>
#include <ed/plugin.h>

#include "ed_cloud/WorldModelDelta.h"

#include "ros/service_client.h"

class SyncClient : public ed::Plugin
{

public:

    SyncClient();

    virtual ~SyncClient();

    void configure(tue::Configuration config);

    void initialize();

    void updateWithDelta(ed_cloud::WorldModelDelta& a, const ed::WorldModel &world, ed::UpdateRequest &req);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    int current_rev_number;

    ros::ServiceClient client;

    std::vector<ed::UUID> entity_index;

    std::map<ed::UUID, int64_t> index_map;

};

#endif // WORLD_UPDATE_CLIENT_PLUGIN_H
