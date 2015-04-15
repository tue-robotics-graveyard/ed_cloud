#ifndef WORLD_READER_H
#define WORLD_READER_H

#include "ed/types.h"
#include "geolib/datatypes.h"
#include "geolib/Mesh.h"

#include <ed/io/reader.h>

namespace ed_cloud
{

void read_pose(ed::io::Reader& r, geo::Pose3D& pose);
void read_shape(ed::io::Reader& r, geo::Mesh& mesh);
void read_convex_hull(ed::io::Reader& r, ed::ConvexHull2D& ch);
void read_publisher(ed::io::Reader& r, std::string publisher);
void read_type(ed::io::Reader& r, ed::TYPE& type);
}

#endif // WORLD_WRITER_H
