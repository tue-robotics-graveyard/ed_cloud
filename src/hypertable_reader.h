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

class HypertableReaderPlugin : public ed::Plugin
{

public:

    HypertableReaderPlugin();

    virtual ~HypertableReaderPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

    std::string get_last_timestamp_string( );

    void process_cells(std::vector<Hypertable::ThriftGen::CellAsArray> &cells, ed::UpdateRequest &req);

    void add_to_world_model(Hypertable::ThriftGen::CellAsArray &cell, ed::UpdateRequest &req);

    void get_cell_publisher(Hypertable::ThriftGen::CellAsArray &cell, std::string& publisher);

private:

    std::string db_address;
    int db_port;
    std::string db_namespace;
    Hypertable::Thrift::Client *client;
    int64_t last_update_time;
    const std::string table_name = "entity_delta";
    int64_t total_elements;
    std::string world_text;
};

#endif
