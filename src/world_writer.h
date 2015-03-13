#ifndef WORLD_WRITER_H
#define WORLD_WRITER_H

#include "ed/types.h"
#include "geolib/datatypes.h"
#include "geolib/Mesh.h"

namespace ed_cloud {

    void world_write(const ed::WorldModel& world,  int rev_number, std::ostream& output);
    void write_entity(const ed::EntityConstPtr& ent, std::ostream &output);
    void write_pose(const geo::Pose3D& pose, std::ostream &output);
    void write_shape(const geo::Mesh& mesh, std::ostream &output);
    void write_convex_hull(const ed::ConvexHull2D& ch, std::ostream &output);
}

#endif // WORLD_WRITER_H
