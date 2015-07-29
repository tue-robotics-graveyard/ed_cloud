#ifndef WORLD_READER_H
#define WORLD_READER_H

#include <ed/types.h>
#include <ed/measurement_convex_hull.h>

#include <geolib/datatypes.h>
#include <geolib/Mesh.h>

#include <ed/io/reader.h>

namespace ed_cloud
{

void read_pose(ed::io::Reader& r, geo::Pose3D& pose);
void read_timestamp(ed::io::Reader& r, double& timestamp);
void read_shape(ed::io::Reader& r, geo::Mesh& mesh);
void read_convex_hull(ed::io::Reader& r, ed::MeasurementConvexHull& ch, std::string& source_id);
void read_publisher(ed::io::Reader& r, std::string publisher);
void read_type(ed::io::Reader& r, ed::TYPE& type);

// Binary

ed::MeasurementConstPtr read_measurement(std::istream& in);
void read_publisher_binary(std::istream& in, std::string& publisher);

}

#endif // WORLD_WRITER_H
