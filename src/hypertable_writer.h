#ifndef HYPERTABLEPLUGIN_H
#define HYPERTABLEPLUGIN_H

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

class HypertableWriterPlugin : public ed::Plugin
{

public:

    HypertableWriterPlugin();

    virtual ~HypertableWriterPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    std::string db_address;
    int db_port;
    std::string db_namespace;
    Hypertable::Thrift::Client *client;
    int64_t total_elements;
    int stop;
    const std::string table_name = "entity_delta";

};

#endif
