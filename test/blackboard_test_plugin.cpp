#include "blackboard_test_plugin.h"
#include <ros/ros.h>
#include <ed/update_request.h>

BlackboardTestPlugin::BlackboardTestPlugin()
{
}

BlackboardTestPlugin::~BlackboardTestPlugin()
{

}

void BlackboardTestPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    bb::Key key = data.blackboard.findKey("test");

    if (key != -1) {
        if (reader) {
            const int *num = data.blackboard.getValue<int>(key, last_update_time.toSec());
            if (num != NULL) {
                std::cout << ros::this_node::getName()
                          << ": Value update: " << *num
                          << std::endl;
            }
            last_update_time = ros::Time::now();
        } else {
            int value = rand() % 10;
            std::cout << "Writing value " << value << std::endl;
            req.bb_update.setValue<int>(key, ros::Time::now().toSec(), value);
        }
    }
}

void BlackboardTestPlugin::initialize(ed::InitData& init)
{
    int rw;
    init.config.value<int>("reader", rw);
    if (rw == 0) {
        reader = false;
    } else  {
        reader = true;
    }
    init.blackboard.addKey("test", new IntSerializer());
}


ED_REGISTER_PLUGIN(BlackboardTestPlugin)
