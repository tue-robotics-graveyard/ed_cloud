#include <ed/world_model.h>
#include <ed/entity.h>
#include "world_writer.h"
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/datatypes.h>

void ed_cloud::write_shape(const geo::Mesh& mesh, std::ostream &output) {

    output << ",";
    output << "\"shape\":"  <<  "{" <<
              "\"vertices\":"
           << "[";


    for (std::vector<geo::Vector3>::const_iterator it2 = mesh.getPoints().begin();
         it2 != mesh.getPoints().end(); it2++)
    {
        if (it2 != mesh.getPoints().begin()) {
          output << ",";
        }

        output << "{";

        output <<  "\"x\":" << "\"" << it2->getX() << "\"" << ",";
        output <<  "\"y\":" << "\"" << it2->getX() << "\"" << ",";
        output <<  "\"z\":" << "\"" << it2->getX() << "\"";

        output << "}";
    }

    output  << "]" << ",";

    output  << "\"triangles\":"
            << "[";


    for (std::vector<geo::TriangleI>::const_iterator it2 = mesh.getTriangleIs().begin();
         it2 != mesh.getTriangleIs().end(); it2++)
    {
        if (it2 != mesh.getTriangleIs().begin()) {
          output << ",";
        }
         output << "{";
         output << "\"i1\"" << ":" << "\"" << it2->i1_ << "\"" << ",";
         output << "\"i2\"" << ":" << "\"" << it2->i2_ << "\"" << ",";
         output << "\"i3\"" << ":" << "\"" << it2->i3_ << "\"";
         output << "}";
    }


    output << "]";
    output << "}";

}

void ed_cloud::write_entity(const ed::EntityConstPtr& ent, std::ostream &output) {

    output << "{";
    output << "\"id\":"     << "\"" << ent->id()   << "\"";
    output << "," << "\"type\":"   << "\"" << ent->type() << "\"";

    write_pose(ent->pose(), output);


    if (ent->shape()) {
      write_shape(ent->shape()->getMesh(), output);

    }

    if (!ent->convexHull().chull.empty()) {
      write_convex_hull(ent->convexHull(), output);
    }

    output   << "}";

}

void ed_cloud::world_write(const ed::WorldModel &world, int rev_number, std::ostream &output)
{
    output << "{";
    output << "\"revision\":\"" << rev_number << "\",";
    output << "\"entities\":";
    output << "[";


    for (std::vector<ed::EntityConstPtr>::const_iterator it = world.entities().begin();
         it != world.entities().end(); it ++) {

        if (it != world.entities().begin()) {
            output << ",";
        }

        write_entity((*it), output);
    }

    output << "]";
    output << "}";
}


void ed_cloud::write_pose(const geo::Pose3D &pose, std::ostream &output)
{

    output << ","<< "\"pose\":"   <<
              "{" <<
                  "\"pos.x\":"  << "\"" << pose.t.getX() << "\""  << "," <<
                  "\"pos.y\":"  << "\"" << pose.t.getY() << "\""  << "," <<
                  "\"pos.z\":"  << "\"" << pose.t.getZ() << "\""  << "," <<
                  "\"rot.xx\":" << "\"" << pose.R.xx     << "\""  << "," <<
                  "\"rot.xy\":" << "\"" << pose.R.xy     << "\""  << "," <<
                  "\"rot.xz\":" << "\"" << pose.R.xz     << "\""  << "," <<
                  "\"rot.yx\":" << "\"" << pose.R.yx     << "\""  << "," <<
                  "\"rot.yy\":" << "\"" << pose.R.yy     << "\""  << "," <<
                  "\"rot.yz\":" << "\"" << pose.R.yz     << "\""  << "," <<
                  "\"rot.zx\":" << "\"" << pose.R.yx     << "\""  << "," <<
                  "\"rot.zy\":" << "\"" << pose.R.yy     << "\""  << "," <<
                  "\"rot.zz\":" << "\"" << pose.R.yz     << "\""  <<
              "}";

}


void ed_cloud::write_convex_hull(const ed::ConvexHull2D &ch, std::ostream &output)
{
    output << ",";
    output << "\"convex_hull\":" << "{";

    output << "\"points\":" << "[";
    for(unsigned int i = 0; i <  ch.chull.size(); ++i)
    {
        if (i != 0) {
            output << ",";
        }
        output << "{";
        output << "\"x\":" << ch.chull[i].x << ",";
        output << "\"y\":" << ch.chull[i].y;
        output << "}";
    }
    output << "]";

    output << "}";

}
