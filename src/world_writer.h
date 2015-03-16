#ifndef WORLD_WRITER_H
#define WORLD_WRITER_H

#include "ed/types.h"
#include "geolib/datatypes.h"
#include "geolib/Mesh.h"

#include <ed/io/writer.h>

namespace ed_cloud {

    void world_write(const ed::WorldModel& world,  int rev_number, ed::io::Writer& w);
    void write_entity(const ed::EntityConstPtr& ent, ed::io::Writer& w);
    void write_pose(const geo::Pose3D& pose, ed::io::Writer& w);
    void write_shape(const geo::Mesh& mesh, ed::io::Writer& w);
    void write_convex_hull(const ed::ConvexHull2D& ch, ed::io::Writer& w);
}

#endif // WORLD_WRITER_H
