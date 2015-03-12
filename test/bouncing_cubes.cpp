#include "bouncing_cubes.h"

#include <ed/update_request.h>

#include <geolib/Box.h>

#include <sstream>

// ----------------------------------------------------------------------------------------------------

BouncingCubes::BouncingCubes()
{
}

// ----------------------------------------------------------------------------------------------------

BouncingCubes::~BouncingCubes()
{
}

// ----------------------------------------------------------------------------------------------------

void BouncingCubes::configure(tue::Configuration config)
{
}

// ----------------------------------------------------------------------------------------------------

void BouncingCubes::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void BouncingCubes::process(const ed::WorldModel &world, ed::UpdateRequest &req)
{
    if (cubes.empty())
    {
        // Initialize cubes

        for(unsigned int i = 0; i < 1; ++i)
            {

            Cube cube;
            cube.pose = geo::Pose3D::identity();
            cube.vel = geo::Vector3(0.2, 0.1, -0.05);

            std::stringstream ss_id;
            ss_id << "cube-" << i;

            cube.id = ss_id.str();

            cubes.push_back(cube);

            // Set the shape
            req.setShape(cube.id, geo::ShapePtr(new geo::Box(geo::Vector3(-0.25, -0.25, -0.25), geo::Vector3(0.25, 0.25, 0.25))));
        }
    }

    // Update cube positions
    for(std::vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); ++it)
    {
        Cube& cube = *it;
        cube.pose.t += cube.vel;

        // Check if cube reached boundary. If so, bounce
        for(unsigned int d = 0; d < 3; ++d)
        {
            if (cube.pose.t.m[d] < -2 || cube.pose.t.m[d] > 2)
                cube.vel.m[d] = -cube.vel.m[d];
        }

        req.setPose(cube.id, cube.pose);
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(BouncingCubes)
