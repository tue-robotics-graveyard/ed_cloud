#include <ed/world_model.h>
#include <ed/entity.h>
#include "world_writer.h"
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/datatypes.h>

#include <ed/io/json_writer.h>

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_shape(const geo::Mesh& mesh, ed::io::Writer& w)
{
    w.writeGroup("shape");

    // Vertices
    w.writeArray("vertices");
    for (std::vector<geo::Vector3>::const_iterator it2 = mesh.getPoints().begin();
         it2 != mesh.getPoints().end(); it2++)
    {
        w.addArrayItem();
        w.writeValue("x", it2->x);
        w.writeValue("y", it2->y);
        w.writeValue("z", it2->z);
        w.endArrayItem();
    }
    w.endArray();

    // Triangles
    w.writeArray("triangles");
    for (std::vector<geo::TriangleI>::const_iterator it2 = mesh.getTriangleIs().begin();
         it2 != mesh.getTriangleIs().end(); it2++)
    {
        w.addArrayItem();
        w.writeValue("i1", it2->i1_);
        w.writeValue("i2", it2->i2_);
        w.writeValue("i3", it2->i3_);
        w.endArrayItem();
    }
    w.endArray();

    w.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_entity(const ed::EntityConstPtr& ent, ed::io::Writer& w)
{
    w.writeValue("id", ent->id().str());
    w.writeValue("type", ent->type());

    if (ent->has_pose())
        write_pose(ent->pose(), w);

    if (ent->shape())
        write_shape(ent->shape()->getMesh(), w);

    if (!ent->convexHull().chull.empty())
        write_convex_hull(ent->convexHull(), w);
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::world_write(const ed::WorldModel &world, int rev_number, ed::io::Writer& w)
{
    w.writeValue("revision", rev_number);

    w.writeArray("entities");

    for (ed::WorldModel::const_iterator it = world.begin();
         it != world.end(); it ++)
    {
        w.addArrayItem();
        write_entity((*it), w);
        w.endArrayItem();
    }

    w.endArray();
    w.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_pose(const geo::Pose3D &pose, ed::io::Writer& w)
{
    w.writeGroup("pose");

    w.writeValue("pos.x", pose.t.x);
    w.writeValue("pos.y", pose.t.y);
    w.writeValue("pos.z", pose.t.z);

    w.writeValue("rot.xx", pose.R.xx);
    w.writeValue("rot.xy", pose.R.xy);
    w.writeValue("rot.xz", pose.R.xz);
    w.writeValue("rot.yx", pose.R.yx);
    w.writeValue("rot.yy", pose.R.yy);
    w.writeValue("rot.yz", pose.R.yz);
    w.writeValue("rot.zx", pose.R.zx);
    w.writeValue("rot.zy", pose.R.zy);
    w.writeValue("rot.zz", pose.R.zz);

    w.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_convex_hull(const ed::ConvexHull2D &ch, ed::io::Writer& w)
{
    w.writeGroup("convex_hull");

    w.writeGroup("center");
    w.writeValue("x", ch.center_point.x);
    w.writeValue("y", ch.center_point.y);
    w.writeValue("z", ch.center_point.z);
    w.endGroup();

    w.writeValue("z_min", ch.min_z);
    w.writeValue("z_max", ch.max_z);

    w.writeArray("points");
    for(unsigned int i = 0; i <  ch.chull.size(); ++i)
    {
        w.addArrayItem();
        w.writeValue("x", ch.chull[i].x);
        w.writeValue("y", ch.chull[i].y);
        w.endArrayItem();
    }
    w.endArray();

    w.endGroup();
}
