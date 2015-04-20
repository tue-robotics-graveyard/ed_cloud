#include <ed/measurement.h>
#include <geolib/sensors/DepthCamera.h>
#include <rgbd/Image.h>

#include "../src/world_reader.h"
#include "../src/world_writer.h"

// ----------------------------------------------------------------------------------------------------

ed::MeasurementConstPtr createMeasurement()
{
    // Create rgb image

    cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));

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

    cv::Mat depth_image(480, 640, CV_32FC1, 0.0);
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

    sensor_pose.t.x = 10;
    sensor_pose.t.y = 11;

    // construct measurement

    ed::MeasurementConstPtr msr(new ed::Measurement(image, mask, sensor_pose));

    return msr;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // WRITING

    std::string data;

    {
        ed::MeasurementConstPtr m = createMeasurement();

        std::vector<ed::MeasurementConstPtr> msrs;
        msrs.push_back(m);

        std::stringstream ss;
        ed_cloud::write_measurement_binary(*m, "Blaaaa", ss);

        data = ss.str();
    }

    // READING

    {
        std::stringstream ss1(data);

        std::string publisher;
        ed_cloud::read_publisher_binary(ss1, publisher);

        std::cout << "Publisher: " << publisher << std::endl;

        std::stringstream ss2(data);

        ed::MeasurementConstPtr m2 = ed_cloud::read_measurement(ss2);

        std::cout << m2->image()->getFrameId() << std::endl;
    }

    return 0;
}
