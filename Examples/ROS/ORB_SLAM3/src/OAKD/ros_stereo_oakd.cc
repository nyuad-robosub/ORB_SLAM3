

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
#include <image_transport/subscriber_filter.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"
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
class Params
{
protected:
    ros::NodeHandle nh_;
    std::string vocab, config, left_img_name, right_img_name;
    void ReadParams()
    {
        int badParams = 0;
        badParams += !nh_.getParam("/Stereo_oakd/vocab", vocab);
        badParams += !nh_.getParam("/Stereo_oakd/config", config);
        cout << badParams;
        badParams += !nh_.getParam("/Stereo_oakd/left_img_name", left_img_name);
        cout << badParams;
        badParams += !nh_.getParam("/Stereo_oakd/right_img_name", right_img_name); // what about framerates?
        cout << badParams;

        if (badParams > 0)
        {
            ROS_ERROR("Couldn't find at least one of the parameters");
        }
        /*vocab="/home/rami/nr22-software/include/ORB_SLAM3_OPENCV4/Vocabulary/ORBvoc.txt";
        config="/home/rami/nr22-software/include/ORB_SLAM3_OPENCV4/Examples/ROS/ORB_SLAM3/Asus1.yaml";
        left_img_name="/stereo_publisher/left/image";
        right_img_name="/stereo_publisher/right/image";*/
    }

public:
    Params()
    {
        ReadParams();
    }
};
class Handler : public Params
{
private:
    ORB_SLAM3::System SLAM;
    tf2_ros::TransformBroadcaster tfb;
    image_transport::ImageTransport it_;
    typedef image_transport::SubscriberFilter ImageSubscriber;
    ImageSubscriber left_img, right_img, disparity_img;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image>
        MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cv_bridge::CvImageConstPtr left_img_ptr, right_img_ptr;
    double frame_timestamp_s;

    Pose pose;
    Sophus::SE3f pose_sophus;

    ros::Publisher lost_pub;
    geometry_msgs::TransformStamped transformStamped;

    void PublishTF()
    {
        pose_sophus=pose_sophus.inverse();//Twc to Tcw
        pose.rotation = Eigen::Quaternionf(pose_sophus.rotationMatrix()); 


        transformStamped.transform.rotation.y = pose.rotation.y(); // this is not rotating about its own axis but world causing large circles
        transformStamped.transform.rotation.x = pose.rotation.x();
        transformStamped.transform.rotation.z = pose.rotation.z();
        transformStamped.transform.rotation.w = pose.rotation.w();

        // pose.rotation = Eigen::Quaternionf(pose_sophus.rotationMatrix());

        pose.position = pose_sophus.translation();//vector3f eigen
        //pose.position = pose_sophus.rotationMatrix().inverse() * pose.position;

        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "orb_slam3_camera";
        transformStamped.transform.translation.x = pose.position.x();
        transformStamped.transform.translation.y = pose.position.y();
        transformStamped.transform.translation.z = pose.position.z();
        //pose.rotation = pose_sophus.unit_quaternion();
        // pose.rotation = Eigen::Quaternionf();
     

        transformStamped.header.stamp = ros::Time::now();
        tfb.sendTransform(transformStamped);
    }

    void PublishLost()
    {
        bool islost = SLAM.isLost();
        std_msgs::Bool t;
        t.data = islost;

        lost_pub.publish(t);
    }

public:
    Handler() : Params(), it_(nh_), left_img(it_, left_img_name, 1), right_img(it_, right_img_name, 1), sync(MySyncPolicy(30), left_img, right_img), SLAM(vocab, config, ORB_SLAM3::System::STEREO, true)
    {
        lost_pub = nh_.advertise<std_msgs::Bool>("orb_slam3_lost", 500);
        sync.registerCallback(boost::bind(&Handler::callback, this, _1, _2));
    }
    void callback(
        const sensor_msgs::ImageConstPtr &left_img,
        const sensor_msgs::ImageConstPtr &right_im)
    {

        try
        {
            left_img_ptr = cv_bridge::toCvShare(left_img);
            right_img_ptr = cv_bridge::toCvShare(right_im);
            std_msgs::Header h = left_img_ptr->header;
            frame_timestamp_s = h.stamp.sec;

            pose_sophus = SLAM.TrackStereo(
                left_img_ptr->image,
                right_img_ptr->image,
                frame_timestamp_s);

            // publish TF and whether orb slam is lost or not
            PublishTF();
            PublishLost();
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        catch (...)
        {
            ROS_ERROR("An unknown exception occured.");
            return;
        }
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

int main(int argc, char **argv)
{

    // get depthai left and right camera feed to orb slam then take orb slam 2 point cloud and tf2

    // CREATE THREAD FOR LOOP THAT RUNS ORB SLAM 2 PUBLISH DATA THROUGH USING MUTEX AND ACCESSING IT
    // CREATE LOOP THAT RUNS ORB SLAM 2 THEN PUBLISH THE DATA USING PUBLISH
    ros::init(argc, argv, "Stereo_oakd");

    // ros::start();

    /*if (argc != 3)
    {
        cout << argc << endl;
        ROS_WARN("Arguments supplied via command line are neglected.");
    }*/
    Handler hdl;
    // Stop all SLAM threads

    ros::spin();

    ros::shutdown();

    return 0;
}