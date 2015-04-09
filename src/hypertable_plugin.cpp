#include "hypertable_plugin.h"
#include <map>
#include <ros/ros.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <geolib/datatypes.h>
#include "world_writer.h"
#include <ed/io/json_writer.h>
#include <geolib/Shape.h>

HyperTablePlugin::HyperTablePlugin()
{
}

HyperTablePlugin::~HyperTablePlugin() {
}

void HyperTablePlugin::initialize(ed::InitData& init) {

    init.config.value("address", db_address);
    init.config.value("port", db_port);
    init.config.value("namespace", db_namespace);

    try {
        client = new Hypertable::Thrift::Client(db_address, db_port);

        // Create namespace and tables

        if (!client->namespace_exists(db_namespace)) {
            client->namespace_create(db_namespace);
        }

        Hypertable::ThriftGen::Namespace ns = client->namespace_open(db_namespace);
        client->table_drop(ns, HyperTablePlugin::table_name, true);

        Hypertable::ThriftGen::Schema schema;
        std::map<std::string, Hypertable::ThriftGen::ColumnFamilySpec> cf_specs;
        Hypertable::ThriftGen::ColumnFamilySpec cf;

        cf.__set_name("pose");
        cf_specs["pose"] = cf;
        cf.__set_name("type");
        cf_specs["type"] = cf;
        cf.__set_name("deleted");
        cf_specs["deleted"] = cf;
        cf.__set_name("data");
        cf_specs["data"] = cf;
        cf.__set_name("type");
        cf_specs["type"] = cf;
        cf.__set_name("convex_hull");
        cf_specs["convex_hull"] = cf;
        cf.__set_name("shape");
        cf_specs["shape"] = cf;

        schema.__set_column_families(cf_specs);

        client->table_create(ns, HyperTablePlugin::table_name, schema);

        client->namespace_close(ns);
    }
    catch (Hypertable::ThriftGen::ClientException &e) {
      init.config.addError(e.message);
    }

}

void HyperTablePlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req) {

    std::vector<Hypertable::ThriftGen::CellAsArray> cells_as_arrays;
    Hypertable::ThriftGen::CellAsArray cell_as_array;
    Hypertable::ThriftGen::Namespace ns = client->namespace_open(db_namespace);


    for(unsigned int i = 0; i < data.deltas.size(); ++i) {
        ed::UpdateRequest req = *data.deltas[i];

        for (auto it = req.poses.begin();
             it != req.poses.end(); it++) {
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back("pose");
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_pose(it->second, wr);
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);
        }

        for (auto it = req.convex_hulls.begin();
             it != req.convex_hulls.end(); it++) {
            ROS_INFO("Convex!!!");
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back("convex_hull");
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_convex_hull(it->second, wr);
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);
        }

        for (auto it = req.shapes.begin();
             it != req.shapes.end(); it++) {
            std::stringstream str;
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back("shape");
            cell_as_array.push_back("");
            ed::io::JSONWriter wr(str);
            ed_cloud::write_shape((*it->second).getMesh(), wr);
            cell_as_array.push_back(str.str());
            cells_as_arrays.push_back(cell_as_array);
        }

        for (auto it = req.types.begin();
             it != req.types.end(); it++) {
            cell_as_array.clear();
            cell_as_array.push_back((it->first).c_str());
            cell_as_array.push_back("type");
            cell_as_array.push_back("");
            cell_as_array.push_back(it->second.c_str());
            cells_as_arrays.push_back(cell_as_array);
        }


        for (auto it = req.removed_entities.begin();
             it != req.removed_entities.end(); it++) {
            cell_as_array.clear();
            cell_as_array.push_back((*it).c_str());
            cell_as_array.push_back("deleted");
            cell_as_array.push_back("");
            cell_as_array.push_back("");
            cells_as_arrays.push_back(cell_as_array);
        }


    }

    if (!cells_as_arrays.empty()) {
        client->set_cells_as_arrays(ns, HyperTablePlugin::table_name,
                                    cells_as_arrays);
    }

    client->namespace_close(ns);
}

ED_REGISTER_PLUGIN(HyperTablePlugin)
