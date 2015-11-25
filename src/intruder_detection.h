#ifndef INTRUDER_DETECTION_H
#define INTRUDER_DETECTION_H

#include <ros/ros.h>
#include <ed/plugin.h>
#include <ed/uuid.h>
#include <ed/types.h>
#include <string>

#include <map>

class IntruderDetectionPlugin : public ed::Plugin
{
public:
    IntruderDetectionPlugin();

    virtual ~IntruderDetectionPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    std::set<ed::UUID> safe_people;
    std::set<ed::UUID> intruders;
    std::set<ed::UUID> unclassified_people;

};

#endif // INTRUDER_DETECTION_H
