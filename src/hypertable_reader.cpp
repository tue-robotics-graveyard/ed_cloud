#include "hypertable_reader.h"
#include <map>
#include <ros/ros.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <geolib/datatypes.h>
#include "world_writer.h"
#include "world_reader.h"
#include <ed/io/json_writer.h>
#include <ed/io/json_reader.h>
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <fstream>
#include "ed_hypertable.h"

HypertableReaderPlugin::HypertableReaderPlugin()
{
}

HypertableReaderPlugin::~HypertableReaderPlugin() {
//    std::cout << "Writing file" << std::endl;
//    std::ofstream ofile("output-client-0.json");
//    ofile << world_text;
//    ofile.close();
}

void HypertableReaderPlugin::initialize(ed::InitData& init) {
    total_elements = 0;
    max_timestamp_queried_entities = max_timestamp_queried_entities.fromNSec(0);
    max_timestamp_queried_measurements = max_timestamp_queried_measurements.fromNSec(0);
    init.config.value("address", db_address);
    init.config.value("port", db_port);
    init.config.value("namespace", db_namespace);
    get_measurements = false;

    elements_to_read.insert(ed_hypertable::DELETED_CELL);

    std::stringstream select_columns_aux;
    select_columns_aux <<  ed_hypertable::DELETED_CELL;

    if (init.config.readArray("read"))
    {
        while(init.config.nextArrayItem())
        {
            std::string property_to_read;
            init.config.value("property", property_to_read);
            elements_to_read.insert(property_to_read);
            if (property_to_read != ed_hypertable::MEASUREMENT_CELL) {
                select_columns_aux << "," << property_to_read;
            }
        }
    }

    select_columns = select_columns_aux.str();
    if (elements_to_read.find(ed_hypertable::MEASUREMENT_CELL) != elements_to_read.end()) {
        get_measurements = true;
    }

    try {
        client = new Hypertable::Thrift::Client(db_address, db_port);
    }
    catch (Hypertable::ThriftGen::ClientException &e) {
      init.config.addError(e.message);
      ROS_ERROR_STREAM(e.message);
    }

}

void HypertableReaderPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req) {

//    std::stringstream ofile;
//    ed::io::JSONWriter writer(ofile);
//    ed_cloud::world_write(data.world, 0, writer);
//    world_text = ofile.str();

    try {
        Hypertable::ThriftGen::Namespace ns = client->namespace_open(db_namespace);

        query_cloud_world_model(ns, select_columns,
                                ed_hypertable::ENTITY_TABLE_NAME,
                                max_timestamp_queried_entities, req);


        if (get_measurements) {
            query_cloud_world_model(ns, ed_hypertable::MEASUREMENT_CELL,
                                    ed_hypertable::MEASUREMENT_TABLE_NAME,
                                    max_timestamp_queried_measurements, req);
        }

        req.setSyncUpdate(true);

        client->namespace_close(ns);
    } catch (Hypertable::ThriftGen::ClientException &e) {
        ROS_ERROR_STREAM(e.message);
    }
}

std::string HypertableReaderPlugin::get_timestamp_string(ros::Time &time)
{
    long  date_msb, date_lsb;
    char timestamp[100];
    date_msb = time.sec;
    date_lsb = time.nsec;

    struct tm *timestamp_tm = localtime(&date_msb);
    strftime (timestamp, 99, "%F %T",
                     timestamp_tm);
    sprintf(timestamp, "%s.%09ld", timestamp, date_lsb);

    return std::string(timestamp);

}

void HypertableReaderPlugin::process_cells(std::vector<Hypertable::ThriftGen::CellAsArray> &cells, ros::Time &timestamp, ed::UpdateRequest& req)
{
    ed::UUID current_entity;
    bool entity_removed = false;

    long timestamp_aux;
    long max_timestamp = 0;

    std::string publisher;
    for (auto & cell : cells) {

        timestamp_aux = atol(cell[4].c_str());

        if (timestamp_aux > max_timestamp) {
            max_timestamp = timestamp_aux;
        }

        get_cell_publisher(cell, publisher);

        if (deleted_entities.find(cell[0]) != deleted_entities.end()) {
            std::cout << "INFO ABOUT REMOVED ENTITY " << cell[0] << std::endl;
        }

        if (publisher == ros::this_node::getName() ||
                (current_entity == cell[0] && entity_removed)
                || deleted_entities.find(cell[0]) != deleted_entities.end()) {
            continue;
        } else {
            entity_removed = false;
            current_entity = cell[0];
            if (cell[1] == ed_hypertable::DELETED_CELL) {
                entity_removed = true;
                deleted_entities.insert(cell[0]);
                req.removeEntity(current_entity);
            } else {
                add_to_world_model(cell, req);
            }
        }
    }

    timestamp = timestamp.fromNSec(max_timestamp);
}

void HypertableReaderPlugin::add_to_world_model(Hypertable::ThriftGen::CellAsArray &cell, ed::UpdateRequest &req)
{
    ed::UUID entity_id = cell[0];
    std::string elem = cell[3] + "}";
    ed::io::JSONReader reader(elem.c_str());

    if (cell[1] == ed_hypertable::POSE_CELL) {
        geo::Pose3D pose;
        ed_cloud::read_pose(reader, pose);
        req.setPose(entity_id, pose);
    } else if (cell[1] == ed_hypertable::SHAPE_CELL) {
        geo::Mesh mesh;
        ed_cloud::read_shape(reader, mesh);
        geo::ShapePtr shape(new geo::Shape());
        shape->setMesh(mesh);
        req.setShape(entity_id, shape);
    }  else if (cell[1] == ed_hypertable::CONVEX_HULL_CELL) {
        if (reader.readArray("convex_hulls")) {
            while(reader.nextArrayItem())
            {
                std::string source_id;
                ed::MeasurementConvexHull ch;
                ed_cloud::read_convex_hull(reader, ch, source_id);
                req.setConvexHullNew(entity_id, ch.convex_hull, ch.pose, ch.timestamp, source_id);
            }
            reader.endArray();
        }
    } else if (cell[1] == ed_hypertable::TYPE_CELL) {
        ed::TYPE type;
        ed_cloud::read_type(reader, type);
        req.setType(entity_id, type);
    } else if (cell[1] == ed_hypertable::MEASUREMENT_CELL) {
        std::istringstream i_str(cell[3]);
        ed::MeasurementConstPtr measure = ed_cloud::read_measurement(i_str);
        req.addMeasurement(entity_id, measure);
    }
}

void HypertableReaderPlugin::get_cell_publisher(Hypertable::ThriftGen::CellAsArray &cell, std::string &publisher)
{
    if (cell[1] == ed_hypertable::MEASUREMENT_CELL) {
        std::istringstream iss(cell[3]);
        ed_cloud::read_publisher_binary(iss, publisher);
     } else {
        std::string elem = cell[3] + "}";
        ed::io::JSONReader reader(elem.c_str());
        ed_cloud::read_publisher(reader, publisher);
    }
}

void HypertableReaderPlugin::query_cloud_world_model(const Hypertable::ThriftGen::Namespace& ns, const std::string &columns, const std::string &table, ros::Time &timestamp, ed::UpdateRequest &req)
{
    std::stringstream query;

    query << "SELECT " << columns << " FROM "
          << table
          << " WHERE TIMESTAMP > "
          << "\"" << get_timestamp_string(timestamp) << "\"";

    if (columns != ed_hypertable::MEASUREMENT_CELL) {
        query << " MAX_VERSIONS = 1";
    }

    Hypertable::ThriftGen::HqlResultAsArrays result_as_arrays;

    client->hql_query_as_arrays(result_as_arrays, ns, query.str());

    if (!result_as_arrays.cells.empty()) {
        total_elements+=result_as_arrays.cells.size();
        ROS_INFO_STREAM("New data! " << result_as_arrays.cells.size()
                        << " Elements, Total " << total_elements);
        process_cells(result_as_arrays.cells, timestamp, req);
    }
}

ED_REGISTER_PLUGIN(HypertableReaderPlugin)
