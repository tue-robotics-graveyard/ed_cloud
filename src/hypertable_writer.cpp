#include "hypertable_writer.h"
#include <map>
#include <ros/ros.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <geolib/datatypes.h>
#include "world_writer.h"
#include <ed/io/json_writer.h>
#include <geolib/Shape.h>
#include <fstream>
#include "ed_hypertable.h"

HypertableWriterPlugin::HypertableWriterPlugin()
{
}

HypertableWriterPlugin::~HypertableWriterPlugin() {
}

void HypertableWriterPlugin::initialize(ed::InitData& init) {


    total_elements = 0;
    init.config.value("address", db_address);
    init.config.value("port", db_port);
    init.config.value("namespace", db_namespace);
    init.config.value("stop", stop);

    try {
        client = new Hypertable::Thrift::Client(db_address, db_port);

        // Create namespace and tables

        if (!client->namespace_exists(db_namespace)) {
            client->namespace_create(db_namespace);
        }

        Hypertable::ThriftGen::Namespace ns = client->namespace_open(db_namespace);
        if (!client->table_exists(ns, ed_hypertable::TABLE_NAME)){
            client->table_drop(ns, ed_hypertable::TABLE_NAME, true);

            Hypertable::ThriftGen::Schema schema;
            std::map<std::string, Hypertable::ThriftGen::ColumnFamilySpec> cf_specs;
            Hypertable::ThriftGen::ColumnFamilySpec cf;
            Hypertable::ThriftGen::ColumnFamilyOptions opts;

            opts.__set_max_versions(ed_hypertable::MAX_VERSIONS);
            cf.__set_options(opts);

            cf.__set_name(ed_hypertable::POSE_CELL);
            cf_specs[ed_hypertable::POSE_CELL] = cf;
            cf.__set_name(ed_hypertable::TYPE_CELL);
            cf_specs[ed_hypertable::TYPE_CELL] = cf;
            cf.__set_name(ed_hypertable::DELETED_CELL);
            cf_specs[ed_hypertable::DELETED_CELL] = cf;
            cf.__set_name(ed_hypertable::DATA_CELL);
            cf_specs[ed_hypertable::DATA_CELL] = cf;
            cf.__set_name(ed_hypertable::CONVEX_HULL_CELL);
            cf_specs[ed_hypertable::CONVEX_HULL_CELL] = cf;
            cf.__set_name(ed_hypertable::SHAPE_CELL);
            cf_specs[ed_hypertable::SHAPE_CELL] = cf;

            schema.__set_column_families(cf_specs);

            client->table_create(ns, ed_hypertable::TABLE_NAME, schema);
        }

        client->namespace_close(ns);
    }
    catch (Hypertable::ThriftGen::ClientException &e) {
      init.config.addError(e.message);
    }

}

void HypertableWriterPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req) {

    std::vector<Hypertable::ThriftGen::CellAsArray> cells_as_arrays;
    Hypertable::ThriftGen::CellAsArray cell_as_array;
    Hypertable::ThriftGen::Namespace ns = client->namespace_open(db_namespace);


    for(unsigned int i = 0; i < data.deltas.size(); ++i) {
        ed::UpdateRequest req = *data.deltas[i];

        // Do not write sync update requests to the database. These are deltas that were
        // generated to keep the local instance in sync with the database
        if (req.is_sync_update)
            continue;

        for (auto it = req.poses.begin();
             it != req.poses.end(); it++) {
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back(ed_hypertable::POSE_CELL);
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_publisher(ros::this_node::getName(), wr);
            ed_cloud::write_pose(it->second, wr);
            wr.endGroup();
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);


        }

        for (auto it = req.convex_hulls.begin();
             it != req.convex_hulls.end(); it++) {
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back(ed_hypertable::CONVEX_HULL_CELL);
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_publisher(ros::this_node::getName(), wr);
            ed_cloud::write_convex_hull(it->second, wr);
            wr.endGroup();
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);
        }

        for (auto it = req.shapes.begin();
             it != req.shapes.end(); it++) {
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back(ed_hypertable::SHAPE_CELL);
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_publisher(ros::this_node::getName(), wr);
            ed_cloud::write_shape((*it->second).getMesh(), wr);
            wr.endGroup();
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);

        }

        for (auto it = req.types.begin();
             it != req.types.end(); it++) {
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back(ed_hypertable::TYPE_CELL);
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_publisher(ros::this_node::getName(), wr);
            ed_cloud::write_type(it->second.c_str(), wr);
            wr.endGroup();
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);

        }


        for (auto it = req.removed_entities.begin();
             it != req.removed_entities.end(); it++) {
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((*it).c_str());
            cell_as_array.push_back(ed_hypertable::DELETED_CELL);
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_publisher(ros::this_node::getName(), wr);
            wr.endGroup();
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);

        }
    }

    if (!cells_as_arrays.empty()) {
        total_elements+=cells_as_arrays.size();
        ROS_INFO_STREAM("Publishing " << cells_as_arrays.size() << " Elements, "
                        << "Total = " << total_elements);
        client->set_cells_as_arrays(ns, ed_hypertable::TABLE_NAME,
                                    cells_as_arrays);
    }

    client->namespace_close(ns);

    if (stop > 0) {
        stop--;
    }

    if (stop == 0) {
        ROS_INFO("Shutting down!");

        std::stringstream fileName;
        fileName << "output-server-";
        fileName << "0.json";
        std::ofstream ofile(fileName.str().c_str());
        ed::io::JSONWriter writer(ofile);
        ed_cloud::world_write(data.world, 0, writer);
        ofile.close();

        ros::shutdown();
        exit(0);
    }
}

ED_REGISTER_PLUGIN(HypertableWriterPlugin)
