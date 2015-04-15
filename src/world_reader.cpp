#include "world_reader.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/datatypes.h>

namespace ed_cloud
{

// ----------------------------------------------------------------------------------------------------

void read_pose(ed::io::Reader& r, geo::Pose3D& pose)
{
    if (!r.readGroup("pose"))
        return;

    r.readValue("pos.x", pose.t.x);
    r.readValue("pos.y", pose.t.y);
    r.readValue("pos.z", pose.t.z);

    r.readValue("rot.xx", pose.R.xx);
    r.readValue("rot.xy", pose.R.xy);
    r.readValue("rot.xz", pose.R.xz);
    r.readValue("rot.yx", pose.R.yx);
    r.readValue("rot.yy", pose.R.yy);
    r.readValue("rot.yz", pose.R.yz);
    r.readValue("rot.zx", pose.R.zx);
    r.readValue("rot.zy", pose.R.zy);
    r.readValue("rot.zz", pose.R.zz);

    r.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void read_shape(ed::io::Reader& r, geo::Mesh& mesh)
{
    if (!r.readGroup("shape"))
        return;

    // Vertices
    if (r.readArray("vertices"))
    {
        while(r.nextArrayItem())
        {
            geo::Vector3 p;
            r.readValue("x", p.x);
            r.readValue("y", p.y);
            r.readValue("z", p.z);
            mesh.addPoint(p);
        }

        r.endArray();
    }

    // Triangles
    if (r.readArray("triangles"))
    {
        while(r.nextArrayItem())
        {
            int i1, i2, i3;
            r.readValue("i1", i1);
            r.readValue("i2", i2);
            r.readValue("i3", i3);
            mesh.addTriangle(i1, i2, i3);
        }

        r.endArray();
    }

    r.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void read_convex_hull(ed::io::Reader& r, ed::ConvexHull2D& ch)
{
    if (!r.readGroup("convex_hull"))
        return;

    if (r.readGroup("center"))
    {
        r.readValue("x", ch.center_point.x);
        r.readValue("y", ch.center_point.y);
        r.readValue("z", ch.center_point.z);
        r.endGroup();
    }

    r.readValue("z_min", ch.min_z);
    r.readValue("z_max", ch.max_z);

    if (r.readArray("points"))
    {
        while(r.nextArrayItem())
        {
            double x, y;
            r.readValue("x", x);
            r.readValue("y", y);
            ch.chull.push_back(pcl::PointXYZ(x, y, 0));
        }
        r.endArray();
    }

    r.endGroup();
}

void read_publisher(ed::io::Reader &r, std::string publisher)
{
    r.readValue("publisher", publisher);
}

void read_type(ed::io::Reader &r, ed::TYPE& type)
{
    r.readValue("type", type);
}

}
