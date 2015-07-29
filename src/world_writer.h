#ifndef WORLD_WRITER_H
#define WORLD_WRITER_H

#include <ed/types.h>
#include <ed/measurement_convex_hull.h>

#include <geolib/datatypes.h>
#include <geolib/Mesh.h>

#include <ed/io/writer.h>

namespace ed_cloud {

    void world_write(const ed::WorldModel& world,  int rev_number, ed::io::Writer& w);
    void write_entity(const ed::EntityConstPtr& ent, ed::io::Writer& w);
    void write_pose(const geo::Pose3D& pose, ed::io::Writer& w);
    void write_timestamp(double timestamp, ed::io::Writer& w);
    void write_shape(const geo::Mesh& mesh, ed::io::Writer& w);
    void write_convex_hull_map(const std::map<std::string, ed::MeasurementConvexHull>& convex_hull_map, ed::io::Writer& w);
    void write_convex_hull(const ed::MeasurementConvexHull& ch, const std::string& source_id, ed::io::Writer& w);
    void write_publisher(const std::string& node_name, ed::io::Writer& w);
    void write_measurements(const std::vector<ed::MeasurementConstPtr> &measurements, ed::io::Writer& w);
    void write_type(const ed::TYPE &type, ed::io::Writer& w);

    // Binary

    void write_publisher_binary(const std::string& node_name, std::ostream& out);
    void write_measurement_binary(const ed::Measurement& msr,const std::string& node_name, std::ostream& out);
}

#endif // WORLD_WRITER_H
