#include <ed/world_model.h>
#include <ed/entity.h>
#include "world_writer.h"
#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/datatypes.h>

#include <ed/io/json_writer.h>

#include <ed/serialization/serialization.h>
#include <rgbd/serialization.h>
#include <rgbd/View.h>
#include <ed/measurement.h>

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_publisher(const std::string &node_name, ed::io::Writer& w) {
    w.writeValue("publisher", node_name);
}

void ed_cloud::write_shape(const geo::Mesh& mesh, ed::io::Writer& w)
{
    w.writeGroup("shape");

    // Vertices
    w.writeArray("vertices");
    for (std::vector<geo::Vector3>::const_iterator it2 = mesh.getPoints().begin();
         it2 != mesh.getPoints().end(); it2++)
    {
        w.addArrayItem();
        w.writeValue("x", it2->x);
        w.writeValue("y", it2->y);
        w.writeValue("z", it2->z);
        w.endArrayItem();
    }
    w.endArray();

    // Triangles
    w.writeArray("triangles");
    for (std::vector<geo::TriangleI>::const_iterator it2 = mesh.getTriangleIs().begin();
         it2 != mesh.getTriangleIs().end(); it2++)
    {
        w.addArrayItem();
        w.writeValue("i1", it2->i1_);
        w.writeValue("i2", it2->i2_);
        w.writeValue("i3", it2->i3_);
        w.endArrayItem();
    }
    w.endArray();

    w.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_entity(const ed::EntityConstPtr& ent, ed::io::Writer& w)
{
    w.writeValue("id", ent->id().str());
    write_type(ent->type(), w);

    if (ent->has_pose())
        write_pose(ent->pose(), w);

    if (ent->shape())
        write_shape(ent->shape()->getMesh(), w);

    if (!ent->convexHullMap().empty())
        write_convex_hull_map(ent->convexHullMap(), w);

    std::vector<ed::MeasurementConstPtr> measurements;
    ent->measurements(measurements);

    if (!measurements.empty()) {
        write_measurements(measurements, w);
    }
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::world_write(const ed::WorldModel &world, int rev_number, ed::io::Writer& w)
{
    w.writeValue("revision", rev_number);

    w.writeArray("entities");

    for (ed::WorldModel::const_iterator it = world.begin();
         it != world.end(); it ++)
    {
        w.addArrayItem();
        write_entity((*it), w);
        w.endArrayItem();
    }

    w.endArray();
    w.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_pose(const geo::Pose3D &pose, ed::io::Writer& w)
{
    w.writeGroup("pose");

    w.writeValue("pos.x", pose.t.x);
    w.writeValue("pos.y", pose.t.y);
    w.writeValue("pos.z", pose.t.z);

    w.writeValue("rot.xx", pose.R.xx);
    w.writeValue("rot.xy", pose.R.xy);
    w.writeValue("rot.xz", pose.R.xz);
    w.writeValue("rot.yx", pose.R.yx);
    w.writeValue("rot.yy", pose.R.yy);
    w.writeValue("rot.yz", pose.R.yz);
    w.writeValue("rot.zx", pose.R.zx);
    w.writeValue("rot.zy", pose.R.zy);
    w.writeValue("rot.zz", pose.R.zz);

    w.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_timestamp(double timestamp, ed::io::Writer& w)
{
    w.writeValue("sec", (int)timestamp);
    w.writeValue("nsec", (int)((timestamp - (int)timestamp) * 1e9));
    w.endGroup();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_convex_hull_map(const std::map<std::string, ed::MeasurementConvexHull>& convex_hull_map, ed::io::Writer& w)
{
    w.writeArray("convex_hulls");
    for(const auto& it : convex_hull_map)
    {
        w.addArrayItem();

        const std::string& source_id = it.first;
        const ed::MeasurementConvexHull& m = it.second;
        write_convex_hull(m, source_id, w);

        w.endArrayItem();
    }
    w.endArray();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_convex_hull(const ed::MeasurementConvexHull& ch, const std::string& source_id, ed::io::Writer& w)
{
    // Write source id
    w.writeValue("source_id", source_id);

    // Write convex hull pose
    write_pose(ch.pose, w);

    // Write timestamp
    w.writeGroup("timestamp");
    write_timestamp(ch.timestamp, w);
    w.endGroup();

    // Write convex hull itself
    w.writeValue("z_min", ch.convex_hull.z_min);
    w.writeValue("z_max", ch.convex_hull.z_max);

    w.writeArray("points");
    for(unsigned int i = 0; i < ch.convex_hull.points.size(); ++i)
    {
        w.addArrayItem();
        w.writeValue("x", ch.convex_hull.points[i].x);
        w.writeValue("y", ch.convex_hull.points[i].y);
        w.endArrayItem();
    }
    w.endArray();
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_type(const ed::TYPE &type, ed::io::Writer &w)
{
    w.writeValue("type", type);
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_measurement_binary(const ed::Measurement& msr, const std::string& node_name, std::ostream& out)
{
    tue::serialization::OutputArchive a_out(out);

    a_out << node_name;

    // save image
    rgbd::serialize(*msr.image(), a_out, rgbd::RGB_STORAGE_LOSSLESS, rgbd::DEPTH_STORAGE_LOSSLESS);

    // save mask
    ed::serialize(msr.imageMask(), a_out);

    const geo::Pose3D& p = msr.sensorPose();
    a_out << p.t.x << p.t.y << p.t.z
          << p.R.xx << p.R.xy << p.R.xz
          << p.R.yx << p.R.yy << p.R.yz
          << p.R.zx << p.R.zy << p.R.zz;
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_publisher_binary(const std::string &node_name, std::ostream &out)
{
     out << node_name;
}

// ----------------------------------------------------------------------------------------------------

void ed_cloud::write_measurements(const std::vector<ed::MeasurementConstPtr>& measurements, ed::io::Writer &w)
{
    w.writeArray("measurements");

    for (auto& measurement: measurements) {
        w.addArrayItem();
        w.writeGroup("measurement");

        w.writeValue("frame_id", measurement->image()->getFrameId());

        w.writeGroup("rgb_image");
        const cv::Mat& rgb_image = measurement->image()->getRGBImage();
        w.writeValue("cols", rgb_image.cols);
        w.writeValue("rows", rgb_image.rows);

        std::stringstream data_rgbd;

        for (int i = 0; i < rgb_image.rows; i ++) {
            for (int j = 0; j < rgb_image.cols; j ++) {
                cv::Vec3b pixel = rgb_image.at<cv::Vec3b>(i, j);
                data_rgbd << std::setw(2) << std::hex << std::setfill('0') << (int)pixel[0];
                data_rgbd << std::setw(2) << std::hex << std::setfill('0') << (int)pixel[1];
                data_rgbd << std::setw(2) << std::hex << std::setfill('0') << (int)pixel[2];
            }
        }

        w.writeValue("data", data_rgbd.str());
        w.endGroup();

        w.writeGroup("depth_image");
        const cv::Mat& depth_image = measurement->image()->getDepthImage();
        w.writeValue("cols", depth_image.cols);
        w.writeValue("rows", depth_image.rows);

        std::stringstream data_depth;

        for (int i = 0; i < depth_image.rows; i ++) {
            for (int j = 0; j < depth_image.cols; j ++) {
                float pixel = depth_image.at<float>(i, j);
                data_depth << std::setw(4) << std::hex << std::setfill('0') << pixel;
            }
        }

        w.writeValue("data", data_depth.str());
        w.endGroup();

        w.writeGroup("camera_model");

        rgbd::View view(*measurement->image(), measurement->image()->getDepthImage().cols);
        const geo::DepthCamera& cam_model = view.getRasterizer();

        w.writeValue("fx", cam_model.getFocalLengthX());
        w.writeValue("fy", cam_model.getFocalLengthY());
        w.writeValue("cx", cam_model.getOpticalCenterX());
        w.writeValue("cy", cam_model.getOpticalCenterY());
        w.writeValue("tx", cam_model.getOpticalTranslationX());
        w.writeValue("ty", cam_model.getOpticalTranslationY());
        w.endGroup();

        w.writeGroup("sensor_pose");
        const geo::Pose3D& sensor_pose = measurement->sensorPose();
        w.writeValue("x", sensor_pose.t.x);
        w.writeValue("y", sensor_pose.t.y);
        w.endGroup();

        w.writeGroup("mask");
        const ed::ImageMask& image_mask = measurement->imageMask();
        w.writeValue("height", image_mask.height());
        w.writeValue("width", image_mask.width());

        std::stringstream mask_str;

        for (cv::Point2i mask_point: image_mask) {
            mask_str << std::setw(4) << std::hex << std::setfill('0')
                     << mask_point.x << mask_point.y;
        }

        w.writeValue("points", mask_str.str());
        w.endGroup();

        w.endGroup();

        w.endArrayItem();
    }
    w.endArray();
}
