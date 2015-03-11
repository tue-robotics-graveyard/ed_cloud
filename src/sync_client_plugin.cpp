#include "sync_client_plugin.h"

#include <ed/WorldModelDelta.h>
#include <ed/EntityUpdateInfo.h>
#include <ed/GetWorldModel.h>
#include <ed/update_request.h>
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/Polygon.h>
#include <ed/Mesh.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/Mesh.h>
#include <geolib/Shape.h>
#include "ros/ros.h"

// ----------------------------------------------------------------------------------------------------

SyncClient::SyncClient()
{
    current_rev_number = 0;
}

// ----------------------------------------------------------------------------------------------------

SyncClient::~SyncClient()
{
}

// ----------------------------------------------------------------------------------------------------

void SyncClient::configure(tue::Configuration config)
{
}

// ----------------------------------------------------------------------------------------------------

void SyncClient::initialize()
{
    current_rev_number = 0;
}

// ----------------------------------------------------------------------------------------------------

void SyncClient::process(const ed::WorldModel &world, ed::UpdateRequest &req)
{
    ros::NodeHandle n;
    ed::GetWorldModel srv;
    ros::ServiceClient client = n.serviceClient<ed::GetWorldModel>("get_world_model");

    srv.request.rev_number = this->current_rev_number;

    if (client.call(srv)) {
            updateWithDelta(srv.response.world, world, req);
            this->current_rev_number = srv.response.rev_number;
    } else {
        ROS_WARN("Cannot obtain world model updates from the server");
    }
}

// ----------------------------------------------------------------------------------------------------

void SyncClient::updateWithDelta(ed::WorldModelDelta& a,
                                        const ed::WorldModel &world,
                                        ed::UpdateRequest &req)
{

    for (std::vector<ed::EntityUpdateInfo>::const_iterator it = a.update_entities.begin();
        it != a.update_entities.end(); it++) {

        req.setType(it->id, it->type);

        if (it->new_pose) {
            geo::Pose3D pose;
            geo::convert(it->pose, pose);
            req.setPose(it->id, pose);
        }

        if (it->new_shape_or_convex) {
            if (it->is_convex_hull) {
                ed::ConvexHull2D ch;
                geo::Pose3D center;
                geo::convert(it->center, center);
                ch.center_point = center.t;
                ch.max_z = it->polygon.z_max;
                ch.min_z = it->polygon.z_min;
                for (int i = 0; i < it->polygon.xs.size(); i ++) {
                    pcl::PointXYZ p (it->polygon.xs[i], it->polygon.ys[i], 0);
                    ch.chull.points.push_back(p);
                }

                req.setConvexHull(it->id, ch);
            } else {
                geo::Mesh m;

                for (int i = 0; i < it->mesh.vertices.size(); i += 3) {

                    m.addPoint(it->mesh.vertices[i],
                               it->mesh.vertices[i + 1],
                               it->mesh.vertices[i + 2]);
                }

                for (int i = 0; i < it->mesh.triangles.size(); i += 3) {
                    m.addTriangle(it->mesh.triangles[i],
                                  it->mesh.triangles[i + 1],
                                  it->mesh.triangles[i + 2]);
                }

                geo::ShapePtr shape(new geo::Shape());
                shape->setMesh(m);
                req.setShape(it->id, shape);
            }
        }
    }

    for (std::vector<std::string>::const_iterator it = a.remove_entities.begin();
         it != a.remove_entities.end(); it++) {
        req.removeEntity(*it);
    }
}


ED_REGISTER_PLUGIN(SyncClient)
