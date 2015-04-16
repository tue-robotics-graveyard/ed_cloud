#include "bouncing_cubes.h"

#include <ed/update_request.h>

#include <geolib/Box.h>

#include <sstream>

#include <set>

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
    config.value("num_cubes", num_cubes_);
    config.value("ns", ns);
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

        newId = 0;
        // Initialize cubes

        for(unsigned int i = 0; i < num_cubes_; ++i)
            {

                Cube cube;
                cube.pose = geo::Pose3D::identity();
                cube.vel = geo::Vector3(0.2, 0.1, -0.05);

                std::stringstream ss_id;
                ss_id << ns << "cube-" << i;

                cube.id = ss_id.str();

                cubes.push_back(cube);

                // Set the shape
                req.setShape(cube.id, geo::ShapePtr(new geo::Box(geo::Vector3(-0.25, -0.25, -0.25), geo::Vector3(0.25, 0.25, 0.25))));
                newId++;
            }
    } else {

        // Random delete: added by javsalgar

        for(std::vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); ++it)
        {
            Cube& cube = *it;
            std::set<ed::UUID>::iterator pos = removed_entities.find(cube.id);

            int v1 = rand() % 100;
            if (v1 > 70) {
                if (pos == removed_entities.end()) {
                    req.removeEntity(cube.id);
                    removed_entities.insert(cube.id);
                }
            } else {
                if (pos != removed_entities.end()) {
                    removed_entities.erase(pos);
                    std::stringstream ss_id;
                    ss_id << ns << "cube-" << newId;
                    cube.id = ss_id.str();
                    newId++;
                }
            }
        }
    }

    // Update cube positions
    for(std::vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); ++it)
    {
        Cube& cube = *it;
        cube.pose.t += cube.vel;

        if (removed_entities.find(cube.id) == removed_entities.end()) {

            // Check if cube reached boundary. If so, bounce
            for(unsigned int d = 0; d < 3; ++d)
            {
                if (cube.pose.t.m[d] < -2 || cube.pose.t.m[d] > 2)
                    cube.vel.m[d] = -cube.vel.m[d];
            }

            req.setPose(cube.id, cube.pose);
        }
    }

    // Random change shape: added by javsalgar
    for(std::vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); ++it)
    {
        Cube& cube = *it;
        if (removed_entities.find(cube.id) == removed_entities.end()) {
            float new_shape_x = rand() / 100.0;
            float new_shape_y = rand() / 100.0;
            float new_shape_z = rand() / 100.0;

            req.setShape(cube.id, geo::ShapePtr(new geo::Box(geo::Vector3(-new_shape_x, -new_shape_y, -new_shape_z),
                                                             geo::Vector3(new_shape_x, new_shape_y, new_shape_z))));
        }
     }

}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(BouncingCubes)
