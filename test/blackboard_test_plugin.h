#ifndef BLACKBOARDTESTPLUGIN_H
#define BLACKBOARDTESTPLUGIN_H

#include <ed/plugin.h>
#include <ros/ros.h>
#include <blackboard/serializer.h>

class BlackboardTestPlugin : public ed::Plugin
{
public:
    BlackboardTestPlugin();

    virtual ~BlackboardTestPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    ros::Time last_update_time;
    bool reader;

};


class IntSerializer : public bb::Serializer
{

    void serialize(const bb::Variant& data, bb::WBytes& bytes)
    {
        int i = data.getValue<int>();
        bytes.resize(sizeof(i));
        memcpy(bytes.ptr(), (unsigned char*)&i, bytes.size());
    }

    void deserialize(const bb::RBytes& bytes, bb::Variant& v)
    {
        int* i = (int*)&bytes.ptr()[0];
        v.setValue<int>(*i);
    }
};

#endif // BLACKBOARDTESTPLUGIN_H
