

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/PointCloud2.h>
#include "System.h"
#include "Converter.h"
#include <geometry_msgs/PoseStamped.h>
#include "depthai/depthai.hpp"
#include "sophus/se3.hpp"
#include "Eigen/Core"
#include <iostream>
using namespace std;
//#include "Node.h"

/* -------------------------------------------------------------------------
 * STRUCTS
 * ------------------------------------------------------------------------- */

// A simple pose structure containing position vector and rotation matrix.
/*typedef struct _Pose
{
    cv::Mat position;
    cv::Mat rotation;
} Pose;*/

typedef struct _Pose
{
    Eigen::Vector3f position;
    Eigen::Quaternionf rotation;
} Pose;

ros::Time current_frame_time_;
// function to convert between cv mat and  dai img frames
cv::Mat imgframe_to_mat(std::shared_ptr<dai::ImgFrame> frame, int data_type = CV_8UC1)
{
    return cv::Mat(
        frame->getHeight(),
        frame->getWidth(),
        data_type,
        frame->getData().data());
}
/*sensor_msgs::PointCloud2 MapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint *> map_points)
{
    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = current_frame_time_;
    cloud.header.frame_id = map_frame_id_param_;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points.at(i)->nObs >= min_observations_per_point_)
        {
            data_array[0] = map_points.at(i)->GetWorldPos().at<float>(2);        // x. Do the transformation by just reading at the position of z instead of x
            data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(0); // y. Do the transformation by just reading at the position of x instead of y
            data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(1); // z. Do the transformation by just reading at the position of y instead of z
            // TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }

    return cloud;
}

void PublishMapPoints(std::vector<ORB_SLAM3::MapPoint *> map_points, ros::Publisher map_points_publisher_)
{
    sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud(map_points);
    map_points_publisher_.publish(cloud);
}*/

void PublishTF(Pose &ps, tf2_ros::TransformBroadcaster tfb)
{
    
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "orb_slam3_camera";
    transformStamped.transform.translation.x = ps.position[0];
    transformStamped.transform.translation.y = ps.position[1];
    transformStamped.transform.translation.z = ps.position[2];
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = ps.rotation.x();
    transformStamped.transform.rotation.y = ps.rotation.y();
    transformStamped.transform.rotation.z = ps.rotation.z();
    transformStamped.transform.rotation.w = ps.rotation.w();

    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
}

/* -------------------------------------------------------------------------
 * CONSTANTS
 * ------------------------------------------------------------------------- */

// WLS parameters, taken from the OpenCV WLS filter docs recommended values.
#define WLS_LAMBDA (8000)
#define WLS_SIGMA (1.0)

/* -------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{

    // get depthai left and right camera feed to orb slam then take orb slam 2 point cloud and tf2

    // CREATE THREAD FOR LOOP THAT RUNS ORB SLAM 2 PUBLISH DATA THROUGH USING MUTEX AND ACCESSING IT
    // CREATE LOOP THAT RUNS ORB SLAM 2 THEN PUBLISH THE DATA USING PUBLISH
    ros::init(argc, argv, "STEREO_OAKD");
    ros::start();

    if (argc != 3)
    {
        cout << argc << endl;
        ROS_WARN("Arguments supplied via command line are neglected.");
    }
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    ros::NodeHandle node_handle;
    //ros::Publisher PC = node_handle.advertise<sensor_msgs::PointCloud2>("ORBSLAM2_PC2", 1);
    //ros::Publisher pose_publisher_ = node_handle.advertise<geometry_msgs::PoseStamped>("ORBSLAM2_POSE", 1);
    tf2_ros::TransformBroadcaster tfb;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // image_transport::ImageTransport image_transport (node_handle);

    // RGBDNode node (ORB_SLAM3::System::RGBD, node_handle, image_transport);

    // Create the pipeline that we're going to build. Pipelines are depthai's
    // way of chaining up different series or parallel process, sort of like
    // gstreamer.
    //
    // Our pipeline is going to extract the left and right rectified images
    // from the cameras so we can pass these into the SLAM system, as well as
    // disparity maps for building a point cloud.
    dai::Pipeline pipeline;
    // We need to create all the nodes in our pipeline, which are:
    //  - the left and right monochrome (greyscale) stereo cameras of the OAK-D
    //  - a stereo depth node, which generates disparity maps and rectified
    //    images. The disparity map will be used in constructing the global
    //    point cloud, and the rectified images for SLAM tracking.
    //  - output nodes, which allow us to get the rectified image data and
    //    disparity map to use outside the pipeline.
    auto mono_left = pipeline.create<dai::node::MonoCamera>();
    auto mono_right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xout_rectif_left = pipeline.create<dai::node::XLinkOut>();
    auto xout_rectif_right = pipeline.create<dai::node::XLinkOut>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();

    // And we set the names of each output node, so we can access them later as
    // output queues
    xout_rectif_left->setStreamName("rectified_left");
    xout_rectif_right->setStreamName("rectified_right");
    xout_disp->setStreamName("disparity");

    // Now we set which cameras are actually connected to the left and right
    // nodes, and set their resolution and framerate
    mono_left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    mono_left->setResolution(
        dai::MonoCameraProperties::SensorResolution::THE_720_P);
    mono_left->setFps(20.0);
    mono_right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    mono_right->setResolution(
        dai::MonoCameraProperties::SensorResolution::THE_720_P);
    mono_right->setFps(20.0);

    // Now we set the stereo node to output rectified images and disp maps. We
    // also set the rectify frames to not be mirrored, and to use black to fill
    // the edges of the rectified images. We need non-flipped images as we're
    // going to use them later down the line as input to the SLAM,
    // unfortunately this means our output disparity map will be flipped, so
    // we'll have to correct that later. We don't output depth as this would
    // disable the disparity map output.
    //
    // We also enable extended disparity depth, which increases the maximum
    // disparity and therefore provides a shorter minimum depth.
    stereo->setOutputRectified(true);
    stereo->setOutputDepth(false);
    stereo->setRectifyEdgeFillColor(0);
    stereo->setRectifyMirrorFrame(false);
    stereo->setExtendedDisparity(true);

    // We now link the cameras up to the stereo node
    mono_left->out.link(stereo->left);
    mono_right->out.link(stereo->right);

    // And the stereo rectified and disp outputs to the output nodes
    stereo->rectifiedLeft.link(xout_rectif_left->input);
    stereo->rectifiedRight.link(xout_rectif_right->input);
    stereo->disparity.link(xout_disp->input);

    // Now we can connect to the OAK-D device and start our pipeline
    dai::Device device(pipeline);
    // Finally to actually see the outputs we need to get their output queues
    // We use a max buffer size of 8 frames and set it into non-blocking mode.
    auto rectif_left_queue = device.getOutputQueue("rectified_left", 8, false);
    auto rectif_right_queue = device.getOutputQueue("rectified_right", 8, false);
    auto disp_queue = device.getOutputQueue("disparity", 8, false);
    // Create the WLS (weighted least squares) filter, which we use to improve
    // the quality of our disparity map. Also set the lambda and sigma values
    // auto wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    // wls_filter->setLambda(WLS_LAMBDA);
    // wls_filter->setSigmaColor(WLS_SIGMA);

    // To use OpenCV's reprojectImageTo3D we need a Q matrix, which is obtained
    // from stereoRectify. This means we'll have to extract some data from the
    // device itself, which is why this is done here.
    // TODO: actually calculate these
    cv::Mat R1, R2, P1, P2, Q;

    // Create the SLAM system. First argument is path to the ORB_SLAM3 vocab
    // file. The second is the path to the settings file for this particular
    // camera setup. The values in this file were taken from what's printed out
    // of `depthai_demo.py`.
    //
    // While the OAK-D does have an IMU we can't use it right now, but support
    //  cout << "HIIDDS" << endl;is coming soon! For now just use stereo mode for SLAM, which isn't as
    // accurate as IMU_STEREO.
    //
    // The last input tells the system to display it's UI.

    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    // Formatter, for printing out matrices in a reasonable way.
    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);

    // We also want somewhere to store our pose data
    Pose pose;

    // The time of each frame is required for SLAM, so we take an epoch time
    // (i.e. our start time) now
    auto slam_epoch = std::chrono::steady_clock::now();
    auto rostimenow = ros::Time::now();
    // Now for the main loop
    while (1)
    {
        // Read the output frames from the OAK-D. These are blocking calls, so
        // they will wait until there's data available.
        auto rectif_left_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto rectif_right_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto disp_map_frame = disp_queue->get<dai::ImgFrame>();

        // Convert the frames into opencv images
        auto rectif_left = imgframe_to_mat(rectif_left_frame);
        auto rectif_right = imgframe_to_mat(rectif_right_frame);
        auto disp_map = imgframe_to_mat(disp_map_frame);
        // cout << endl << rectif_left.empty()<<endl;
        // cv::imshow("left", rectif_left_frame->getCvFrame());
        // cv::waitKey(0);
        //  Get the time between the epoch and now, allowing us to get a
        //  timestamp (in seconds) to pass into the slam system.
        auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        double frame_timestamp_s = elapsed_time.count() / 1000000000.0;

        // std::cout << std::setprecision(4) << frame_timestamp_s << ": ";

        // Pass the images into the SLAM system. This produces a matrix with
        // the pose information of the camera.
        // cout << "HIIDD13213S" << endl;
        /*cv::Mat raw_pose = SLAM.TrackStereo(
            rectif_left,
            rectif_right,
            frame_timestamp_s);*/
        Sophus::SE3f pose_sophus = SLAM.TrackStereo(
            rectif_left,
            rectif_right,
            frame_timestamp_s);
        // cout << "HIIDD13213S" << endl;
        // ros::Time tomorrow = ros::Time::now() + ros::Duration(24*60*60);
        current_frame_time_ = rostimenow + ros::Duration(frame_timestamp_s);
        // cout << endl << raw_pose.empty()<<endl;
        // cout << "test";
        // PublishPositionAsPoseStamped(raw_pose,pose_publisher_);
        //  The output pose may be empty if the system was unable to track the
        //  movement, so only get position and rotation if pose isn't empty. We
        //  also put this info an a localisation fix available flag for later
        //  use.
        // bool loc_fix_available = !raw_pose.empty();
        pose.position = pose_sophus.translation();
        pose.rotation = pose_sophus.unit_quaternion();
        PublishTF(pose,tfb);

        //pointcloud??



        // bool loc_fix_available=!pose_sophus.
        /*if (loc_fix_available)
        {*/
        // The pose matrix is a 4x4 extrinsic matrix, with the form:
        // [R_3x3 T_3x1; [0 0 0 1]], we can find the camera position with
        // C = -R'T (R' = R transpose).
        // pose.rotation = raw_pose(cv::Rect(0, 0, 3, 3));
        /*pose.rotation = raw_pose(cv::Rect(0, 0, 3, 3));
        cv::Mat T = raw_pose(cv::Rect(3, 0, 1, 3));
        pose.position = -pose.rotation.t() * T;*/
        // Print the updated position, but transpose it so that instead of
        // a column vector we have a row vector, which is easier to read.
        /*std::cout <<
            "position: " <<
            fmt->format(pose.position.t()) <<
            std::endl;*/
        /*}
        else
        {
            // If we didn't get a pose update log it.
            // std::cout << "no pose update" << std::endl;
        }*/

        // The raw disparity map is flipped, since we flipped the rectified
        // images, so we must flip it as well.
        //cv::flip(disp_map, disp_map, 1);

        // Filter the disparity map
        // cv::Mat filtered_disp_map;
        // wls_filter->filter(disp_map, rectif_right, filtered_disp_map);

        // Apply a colormap to the filtered disparity map, but don't normalise
        // it. Normalising the map will mean that the color doesn't correspond
        // directly with disparity.
        /*cv::Mat colour_disp;
        cv::applyColorMap(disp_map, colour_disp, cv::COLORMAP_JET);
        cv::imshow("disparity", colour_disp);*/

        // See if q pressed, if so quit
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    // Stop all SLAM threads
    SLAM.Shutdown();

    ros::spin();

    ros::shutdown();

    return 0;
}