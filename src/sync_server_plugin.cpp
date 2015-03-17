#include "sync_server_plugin.h"

#include <ed/entity.h>
#include <fstream>
#include <sstream>
#include "world_writer.h"

#include <ed/io/json_writer.h>

// ----------------------------------------------------------------------------------------------------

void shapeToMsg(const geo::Shape& shape, ed_cloud::Mesh& msg)
{
    const geo::Mesh& mesh = shape.getMesh();

    for (std::vector<geo::Vector3>::const_iterator it2 = mesh.getPoints().begin();
         it2 != mesh.getPoints().end(); it2++)
    {
         msg.vertices.push_back(it2->getX());
         msg.vertices.push_back(it2->getY());
         msg.vertices.push_back(it2->getZ());
    }

    for (std::vector<geo::TriangleI>::const_iterator it2 = mesh.getTriangleIs().begin();
         it2 != mesh.getTriangleIs().end(); it2++)
    {
         msg.triangles.push_back(it2->i1_);
         msg.triangles.push_back(it2->i2_);
         msg.triangles.push_back(it2->i3_);
    }
}

// ----------------------------------------------------------------------------------------------------

void polygonToMsg(const ed::ConvexHull2D& ch, ed_cloud::Polygon& msg)
{
    msg.z_min = ch.min_z;
    msg.z_max = ch.max_z;

    msg.xs.resize(ch.chull.size());
    msg.ys.resize(ch.chull.size());

    for(unsigned int i = 0; i <  ch.chull.size(); ++i)
    {
        msg.xs[i] = ch.chull[i].x;
        msg.ys[i] = ch.chull[i].y;
    }
}

// ----------------------------------------------------------------------------------------------------

void entityToMsg(const ed::Entity& e, ed_cloud::EntityUpdateInfo& msg)
{
    // id
    msg.id = e.id().str();

    // type
    msg.type = e.type();
    msg.new_type = true;

    // pose
    geo::convert(e.pose(), msg.pose);
    msg.new_pose = true;

    // shape
    if (e.shape())
    {
        // Mesh
        shapeToMsg(*e.shape(), msg.mesh);
        msg.new_shape_or_convex = true;
    }
    else if (!e.convexHull().chull.empty())
    {
        // Polygon
        polygonToMsg(e.convexHull(), msg.polygon);
        msg.new_shape_or_convex = true;
    }
}

// ----------------------------------------------------------------------------------------------------

SyncServer::SyncServer()
{
    current_rev_number = 0;
    i_delta_models_start_ = 0;

    max_num_delta_models_ = 10;
}

// ----------------------------------------------------------------------------------------------------

SyncServer::~SyncServer()
{
}

// ----------------------------------------------------------------------------------------------------

bool SyncServer::GetWorldModel(ed_cloud::GetWorldModel::Request &req, ed_cloud::GetWorldModel::Response &res)
{
    ROS_INFO_STREAM("Queried revision " << req.rev_number
                    << " , " << "Current rev number = "
                    << current_rev_number << std::endl);

    if (req.rev_number > current_rev_number)
    {
        std::stringstream ss;
        ss << "Requested revision is in the future: client asks revision '" << req.rev_number << "' but current revision is '" << current_rev_number << "'.";
        res.error = ss.str();
    }
    else if (current_rev_number > req.rev_number)
    {
        if (req.rev_number + deltaModels.size() < current_rev_number)
        {
            // The revision is too old for the number of deltas stored. Therefore, send full entity info, but only
            // for the entities that changed since the requested revision number

            ROS_INFO_STREAM("Requested revision too old for sending small delta revision; will send full entity info instead.");

            for(unsigned int i = 0; i < world_->entities().size(); ++i)
            {
                const ed::EntityConstPtr& e = world_->entities()[i];
                if (req.rev_number < entity_server_revisions_[i])
                {
                    res.world.update_entities.push_back(ed_cloud::EntityUpdateInfo());
                    ed_cloud::EntityUpdateInfo& info = res.world.update_entities.back();
                    info.index = i;

                    if (e)
                        entityToMsg(*e, info);
                    // else: do nothing. An empty entity info message (no id, no type) will tell the client that this entity
                    // does not longer exist, and the client will delete the entity
                }
            }
        }
        else
        {
            res.world = combineDeltas(req.rev_number);
        }
    }

    if (res.error.empty())
        res.rev_number = current_rev_number;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void SyncServer::configure(tue::Configuration config)
{
}

// ----------------------------------------------------------------------------------------------------

void SyncServer::initialize()
{
    ros::NodeHandle nh;

    current_rev_number = 0;

    ROS_INFO("Advertising 'get_world' service");

    ros::AdvertiseServiceOptions get_world_model =
            ros::AdvertiseServiceOptions::create<ed_cloud::GetWorldModel>(
                "/ed/get_world", boost::bind(&SyncServer::GetWorldModel, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);
    srv_get_world_ = nh.advertiseService(get_world_model);
}

// ----------------------------------------------------------------------------------------------------

void SyncServer::addDelta(const ed::UpdateRequest &req)
{
    current_rev_number++;

    if (deltaModels.size() < max_num_delta_models_)
    {
        // If we have not yet reached the max number of delta models, simply push the delta
        deltaModels.push_back(req);
    }
    else
    {
        // Otherwise, calculate the new index (deltaModels is a circular buffer)
        deltaModels[i_delta_models_start_] = req;
        i_delta_models_start_ = (i_delta_models_start_ + 1) % max_num_delta_models_;
    }

    for(std::set<ed::UUID>::const_iterator it = req.updated_entities.begin(); it != req.updated_entities.end(); ++it)
    {
        const ed::UUID& id = *it;
        ed::Idx idx;
        if (world_->findEntityIdx(id, idx))
        {
            for(ed::Idx i = entity_server_revisions_.size(); i < idx + 1; ++i)
                entity_server_revisions_.push_back(0);
            entity_server_revisions_[idx] = current_rev_number;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

ed_cloud::EntityUpdateInfo& SyncServer::addOrGetEntityUpdate(
        const ed::UUID& id, std::map<std::string, unsigned int>& ids, ed_cloud::WorldModelDelta& delta)
{
    std::map<std::string, unsigned int>::const_iterator it = ids.find(id.str());
    if (it != ids.end())
        return delta.update_entities[it->second];

    // Remember the entity index for next time
    ids[id.str()] = delta.update_entities.size();

    // Add a fresh new entity update
    delta.update_entities.push_back(ed_cloud::EntityUpdateInfo());

    // Get a reference to the new entity update
    ed_cloud::EntityUpdateInfo& new_info = delta.update_entities.back();

    // Set id
    new_info.id = id.str();

    // Find entity index
    if (world_->findEntityIdx(new_info.id, new_info.index))
    {

        // Update entity server revision list
        for(unsigned int i = entity_server_revisions_.size(); i < new_info.index + 1; ++i)
            entity_server_revisions_.push_back(0);
        entity_server_revisions_[new_info.index] = current_rev_number;
    }

    return new_info;
}

// -----------------------------------------------------------------------  -----------------------------

void SyncServer::process(const ed::PluginInput& data, ed::UpdateRequest &req)
{
    world_ = &data.world;

    for(unsigned int i = 0; i < data.deltas.size(); ++i)
        this->addDelta(*data.deltas[i]);

    if (this->current_rev_number >= 98 && this->current_rev_number <= 102)
    {
        std::stringstream fileName;
        ROS_INFO("Writing file");
        fileName << "output-server-";
        fileName << this->current_rev_number;
        fileName << ".json";
        std::ofstream ofile(fileName.str().c_str());
        ed::io::JSONWriter writer(ofile);
        ed_cloud::world_write(data.world, this->current_rev_number, writer);
        ofile.close();
    }


    this->cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

ed_cloud::WorldModelDelta SyncServer::combineDeltas(int rev_number)
{
    ed_cloud::WorldModelDelta res_delta;

    std::map<std::string, unsigned int> updated_ids;

    // Merge information starting with the latest delta
    for (int i = this->current_rev_number - 1; i >= rev_number; i--)
    {
        const ed::UpdateRequest& delta = deltaModels[(i + deltaModels.size() + i_delta_models_start_ - current_rev_number) % max_num_delta_models_];

        // Types
        for (std::map<ed::UUID, std::string>::const_iterator it = delta.types.begin(); it != delta.types.end(); it ++)
        {
            ed_cloud::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, res_delta);
            if (!info.new_type)
            {
                info.new_type = true;
                info.type = it->second;
            }
        }

        // Poses
        for (std::map<ed::UUID, geo::Pose3D>::const_iterator it = delta.poses.begin(); it != delta.poses.end(); it ++)
        {
            ed_cloud::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, res_delta);
            if (!info.new_pose)
            {
                info.new_pose = true;
                geo::convert(it->second, info.pose);
            }
        }

        // Shapes
        for (std::map<ed::UUID, geo::ShapeConstPtr>::const_iterator it = delta.shapes.begin(); it != delta.shapes.end(); it ++)
        {
            ed_cloud::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, res_delta);
            if (!info.new_shape_or_convex)
            {
                info.new_shape_or_convex = true;
                info.is_convex_hull = false;
                shapeToMsg(*it->second, info.mesh);
            }
        }

        // Convex hulls
        for (std::map<ed::UUID, ed::ConvexHull2D>::const_iterator it = delta.convex_hulls.begin(); it != delta.convex_hulls.end(); it ++)
        {
            ed_cloud::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, res_delta);
            if (!info.new_shape_or_convex)
            {
                info.new_shape_or_convex = true;
                info.is_convex_hull = true;
                polygonToMsg(it->second, info.polygon);
            }
        }

        // Removed entities
        for (std::set<ed::UUID>::iterator it = delta.removed_entities.begin(); it != delta.removed_entities.end(); it++)
        {
            res_delta.remove_entities.push_back(it->str());
        }
    }

    return res_delta;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(SyncServer)
