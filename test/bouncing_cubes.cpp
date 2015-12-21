#include "bouncing_cubes.h"

#include <ed/update_request.h>

#include <geolib/Box.h>
#include <geolib/sensors/DepthCamera.h>
#include <geolib/datatypes.h>
#include <sstream>
#include <rgbd/Image.h>
#include <ed/mask.h>
#include <ed/measurement.h>

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
    config.value("stop", stop);
    config.value("die", die);
    config.value("measurement_width", measurement_width);
    config.value("measurement_height", measurement_height);

}

// ----------------------------------------------------------------------------------------------------

void BouncingCubes::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void BouncingCubes::process(const ed::WorldModel &world, ed::UpdateRequest &req)
{
    if (die <= 0) {
        exit(0);
    }

    if (stop == 0) {
	die--;
        return;
    }

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

    for (Cube& cube: cubes) {

        if (removed_entities.find(cube.id) == removed_entities.end()) {
            // Create rgb image
            cv::Mat rgb_image(measurement_height, measurement_width, CV_8UC3, cv::Scalar(0,0,255));

            for(int x = 0; x < rgb_image.cols; ++x)

            {
                for(int y = 0; y < rgb_image.rows; ++y)

                {
                    rgb_image.at<cv::Vec3b>(y, x) = cv::Vec3b(rand() % 256,
                                                              rand() % 256,
                                                              rand() % 256);  // BGR
                }
            }

            // Create depth image

            cv::Mat depth_image(measurement_height, measurement_width, CV_32FC1, 0.0);
            for(int x = 0; x < rgb_image.cols; ++x)

            {
                for(int y = 0; y < rgb_image.rows; ++y)
                {
                    depth_image.at<float>(y, x) = (rand() % 1000)/100;// Depth is in meters

                }
            }

            // Create camera model

            geo::DepthCamera cam_model;
            cam_model.setFocalLengths(rand() % 500, rand() % 500);
            cam_model.setOpticalCenter(rand() % 500, rand() % 500);
            cam_model.setOpticalTranslation(rand() % 100, rand() % 100);

            // Set frame id and timestamp

            std::string frame_id = "/amigo/top_kinect";

            double timestamp = 123;

            // construct RGBD image

            rgbd::ImageConstPtr image(new rgbd::Image(rgb_image, depth_image, cam_model, frame_id, timestamp));

            // Create mask (add pixels that belong to the mask)

            ed::ImageMask mask(rgb_image.cols, rgb_image.rows);

            for (int x = 0; x < rgb_image.cols; x ++) {
                for (int y = 0; y < rgb_image.rows; y ++) {
                    if (rand() % 70 <= 70) {
                        mask.addPoint(x, y);
                    }
                }
            }

            // Create sensor pose

            geo::Pose3D sensor_pose = geo::Pose3D::identity();

            sensor_pose.t.x = (float)(rand() % 10)/10 + cube.pose.t.x;
            sensor_pose.t.y = (float)(rand() % 10)/10 + cube.pose.t.y;

            // construct measurement

            ed::MeasurementConstPtr msr(new ed::Measurement(image, mask, sensor_pose));

            req.addMeasurement(cube.id, msr);
        }
    }

    if (stop > 0) {
        stop--;
    }

    if (die > 0) {
        die--;
    }

}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(BouncingCubes)
