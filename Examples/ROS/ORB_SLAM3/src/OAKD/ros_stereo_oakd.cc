

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
#include <image_transport/image_transport.h> //used for cv images

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

class Handler
{
private:
    ORB_SLAM3::System SLAM;
    tf2_ros::TransformBroadcaster tfb;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    typedef image_transport::SubscriberFilter ImageSubscriber;
    ImageSubscriber left_img, right_img, disparity_img;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>
        MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cv_bridge::CvImageConstPtr left_img_ptr,right_img_ptr;
    double frame_timestamp_s;
    Pose pose;

public:
    Handler(char * vocabulary, char * config) : it_(nh_), left_img(it_, "image/left", 1), right_img(it_, "image/right", 1), sync(MySyncPolicy(10), left_img, right_img)
    {
        SLAM = ORB_SLAM3::System(vocabulary,config,ORB_SLAM3::System::STEREO, true);
        sync.registerCallback(boost::bind(&MyClass::callback, this, _1, _2));
        tf2_ros::TransformBroadcaster tfb;
    }
    void callback(
        const sensor_msgs::ImageConstPtr &left_img,
        const sensor_msgs::ImageConstPtr &right_im,)
    {
        
        try
        {
            left_img_ptr = cv_bridge::toCvShare(left_img); // use same encoding of left_img also how to store cv mat
            right_img_ptr = cv_bridge::toCvShare(right_im);
            std_msgs::Header h = left_img_ptr->header;
            frame_timestamp_s=h.stamp.sec;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Sophus::SE3f pose_sophus = SLAM.TrackStereo(
            left_img_ptr->image, // CV:MAT USE CV BRIDGE TO GET IT 8UC1 CV MAT
            right_img_ptr->image,
            frame_timestamp_s);
        pose.position = pose_sophus.translation();
        pose.rotation = pose_sophus.unit_quaternion();
        PublishTF(pose);
        
    }

    void PublishTF(Pose &ps)
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

    ~Handler()
    {
        SLAM.Shutdown();
    }
};
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
    typedef image_transport::SubscriberFilter ImageSubscriber;

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
    //ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    Handler hdl(argv[1],argv[2]);
    ros::NodeHandle node_handle;
    // ros::Publisher PC = node_handle.advertise<sensor_msgs::PointCloud2>("ORBSLAM2_PC2", 1);
    // ros::Publisher pose_publisher_ = node_handle.advertise<geometry_msgs::PoseStamped>("ORBSLAM2_POSE", 1);
    /*tf2_ros::TransformBroadcaster tfb;

    ImageSubscriber ImageSubscriber;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image>
        MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync;*/

    // sync.registerCallback(boost::bind(&MyClass::callback, this, _1, _2));

    // Stop all SLAM threads

    ros::spin();

    ros::shutdown();

    return 0;
}