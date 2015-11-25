#include "intruder_detection.h"
#include <ed/world_model.h>
#include <ed_perception/Classify.h>
#include <ed/update_request.h>


IntruderDetectionPlugin::IntruderDetectionPlugin()
{

}

IntruderDetectionPlugin::~IntruderDetectionPlugin()
{

}

void IntruderDetectionPlugin::initialize(ed::InitData &init)
{
    std::cout << "Plugin init!!" << std::endl;
}

void IntruderDetectionPlugin::process(const ed::PluginInput &data, ed::UpdateRequest &req)
{
    ros::NodeHandle nh;
    ed_perception::Classify clas_req;
    ros::ServiceClient cl_perception = nh.serviceClient<ed_perception::Classify>("ed/classify");

    for(unsigned int i = 0; i < data.deltas.size(); ++i) {
        ed::UpdateRequest req2 = *data.deltas[i];
        for (auto it = req2.types.begin(); it != req2.types.end(); it++) {
            ed::TYPE entity_type = it->second;
            std::cout << "Entity " << it->first.str() << std::endl;
            if (entity_type == "suspect") {
                clas_req.request.all_ids = false;
                clas_req.request.ids.push_back(it->first.str().c_str());
                clas_req.request.property = "color";
                clas_req.request.perception_models_path =
                        "/home/ubuntu/ros/indigo/system/src/ed_perception_models/models/robotics_testlabs";
                std::cout << "Calling perception with id " << it->first.str() << std::endl;
                if (cl_perception.call(clas_req) && !clas_req.response.expected_values.empty()) {
                    std::cout << "Found color:" << clas_req.response.expected_values[0] << std::endl;
                    if (clas_req.response.expected_values[0] == "red") {
                        std::cout << "Found intruder!!!" << std::endl;
                        req.setType(it->first, "intruder");
                    }
                }
            }
        }
    }
}


ED_REGISTER_PLUGIN(IntruderDetectionPlugin)
