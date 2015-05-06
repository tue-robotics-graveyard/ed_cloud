#ifndef HYPERTABLEREADERPLUGIN_H
#define HYPERTABLEREADERPLUGIN_H

#include <ed/plugin.h>
#include <Common/Compat.h>
#include <Common/Logger.h>
#include <Common/System.h>

#include <Hypertable/Lib/Key.h>
#include <Hypertable/Lib/KeySpec.h>

#include <ThriftBroker/Client.h>
#include <ThriftBroker/gen-cpp/Client_types.h>
#include <ThriftBroker/gen-cpp/HqlService.h>
#include <ThriftBroker/ThriftHelper.h>
#include <ThriftBroker/SerializedCellsReader.h>
#include <ThriftBroker/SerializedCellsWriter.h>
#include <ros/time.h>

class HypertableReaderPlugin : public ed::Plugin
{

public:

    HypertableReaderPlugin();

    virtual ~HypertableReaderPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

    std::string get_timestamp_string(ros::Time& time);

    void process_cells(std::vector<Hypertable::ThriftGen::CellAsArray> &cells, ros::Time& timestamp, ed::UpdateRequest &req);

    void add_to_world_model(Hypertable::ThriftGen::CellAsArray &cell, ed::UpdateRequest &req);

    void get_cell_publisher(Hypertable::ThriftGen::CellAsArray &cell, std::string& publisher);

    void query_cloud_world_model(const Hypertable::ThriftGen::Namespace& ns, const std::string &columns, const std::string &table, ros::Time& timestamp, ed::UpdateRequest &req);

private:

    std::string db_address;
    int db_port;
    std::string db_namespace;
    Hypertable::Thrift::Client *client;
    ros::Time max_timestamp_queried_entities;
    ros::Time max_timestamp_queried_measurements;
    int64_t total_elements;
    std::string world_text;
    std::set<std::string> elements_to_read;
    std::string select_columns;
    bool get_measurements;

    // workaround
    std::set<std::string> deleted_entities;
};

#endif
