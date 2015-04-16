#ifndef BOUNCING_CUBES_PLUGIN_H
#define BOUNCING_CUBES_PLUGIN_H

#include <ed/plugin.h>
#include <ed/uuid.h>

#include <geolib/datatypes.h>

// ----------------------------------------------------------------------------------------------------

struct Cube
{
    ed::UUID id;
    geo::Pose3D pose;
    geo::Vector3 vel;
};

// ----------------------------------------------------------------------------------------------------

class BouncingCubes : public ed::Plugin
{
public:
    BouncingCubes();

    virtual ~BouncingCubes();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::vector<Cube> cubes;
    std::set<ed::UUID> removed_entities;
    unsigned long newId;
    int num_cubes_;
    std::string ns;
};

#endif
