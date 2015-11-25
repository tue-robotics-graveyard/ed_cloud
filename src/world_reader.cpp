#include "world_reader.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/datatypes.h>

#include <rgbd/serialization.h>
#include <ed/serialization/serialization.h>
#include <ed/mask.h>
#include <ed/measurement.h>

namespace ed_cloud
{

// ----------------------------------------------------------------------------------------------------

void read_pose(ed::io::Reader& r, geo::Pose3D& pose)
{
    if (!r.readGroup("pose")) {
        return;
    }

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

void read_timestamp(ed::io::Reader& r, double& timestamp)
{
    int sec, nsec;
    r.readValue("sec", sec);
    r.readValue("nsec", nsec);

    timestamp = sec + (double)nsec / 1e9;
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

void read_convex_hull(ed::io::Reader& r, ed::MeasurementConvexHull& ch, std::string& source_id)
{
    // Read source id
    r.readValue("source_id", source_id);

    // Read convex hull pose
    read_pose(r, ch.pose);

    // Read timestamp
    r.readGroup("timestamp");
    read_timestamp(r, ch.timestamp);
    r.endGroup();

    // Read convex hull itself
    r.readValue("z_min", ch.convex_hull.z_min);
    r.readValue("z_max", ch.convex_hull.z_max);

    if (r.readArray("points"))
    {
        while(r.nextArrayItem())
        {
            double x, y;
            r.readValue("x", x);
            r.readValue("y", y);
            ch.convex_hull.points.push_back(geo::Vec2f(x, y));
        }
        r.endArray();
    }

}

// ----------------------------------------------------------------------------------------------------

void read_publisher(ed::io::Reader &r, std::string& publisher)
{
    r.readValue("publisher", publisher);
}

// ----------------------------------------------------------------------------------------------------

void read_type(ed::io::Reader &r, ed::TYPE& type)
{
    r.readValue("type", type);
}

// ----------------------------------------------------------------------------------------------------

ed::MeasurementConstPtr read_measurement(std::istream& in)
{
    std::string publisher;

    tue::serialization::InputArchive a_in(in);
    a_in >> publisher;
    rgbd::ImagePtr image(new rgbd::Image);
    rgbd::deserialize(a_in, *image);

    ed::ImageMask mask;
    ed::deserialize(a_in, mask);

    geo::Pose3D p;
    a_in >> p.t.x >> p.t.y >> p.t.z
         >> p.R.xx >> p.R.xy >> p.R.xz
         >> p.R.yx >> p.R.yy >> p.R.yz
         >> p.R.zx >> p.R.zy >> p.R.zz;

    return ed::MeasurementConstPtr(new ed::Measurement(image, mask, p));
}

void read_publisher_binary(std::istream &in, std::string &publisher)
{
    tue::serialization::InputArchive a_in(in);
    a_in >> publisher;
}

}

