#pragma once

#include <ros/ros.h> 

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
// #include <librealsense2/rsutil.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <vision_msgs/Detection2DArray.h>

#include <Convert.h>

#define REAL    1   // Select Real or Gazebo
#define GAZEBO  0   

struct FRAME
{
    int x = 640;
    int y = 480;
    int middle_x = x/2;
    int middle_y = y/2;
};


struct CAMERA_DATA
{
    std::string depth_topic = "camera/depth/image_raw"; //default : For Gazebo, depth cam
    FRAME frame = {};
};

struct THRESHOLD
{
    double box_probability = 0.5;
    float object_distance = 3.0;        // depth 카메라
    float object_distance_margin = 0.2; // depth 카메라 (m 값)
    float yaw_margin_coeff = 0.05; // bounding box (픽셀 값 -> camera frame 의 5%로 설정 (합쳐서 10%))
};

struct TRACKING_SPEED
{
    double speed_x = 0.1;
    double accel_x_coeff = 0.1;

    double speed_yaw = 0.1;
    double accel_yaw_coeff = 0.003;
};

struct BOUNDING_BOX
{
    bool check_box; 
    darknet_ros_msgs::BoundingBox current_darknetBox;
    vision_msgs::Detection2D current_ssdBox;

    std::string class_id;
    double probability;
    int center_x;
    int center_y;
};

// struct INTRINSIC
// {

// };


class ObjectTracking{
    private:
        ros::NodeHandle nh; 

        ros::Publisher pub_tracking;

        ros::Subscriber sub_darknetBox;
        ros::Subscriber sub_ssdBox;
        ros::Subscriber sub_request;

        std::string param_tracking = "drone/mode/tracking";
        bool mode_tracking = false;
        bool real_or_gazebo= REAL;

        image_transport::CameraSubscriber sub_camera;
        image_transport::ImageTransport it;
        tf::TransformBroadcaster pub_tf;
        // rs2_intrinsics intrinsic;
        // INTRINSIC intrinsic;

        ros::Rate rate = ros::Rate(10.0);

        geometry_msgs::TwistStamped CMD_MOVE_DRONE;

        CAMERA_DATA camera_data = {};
        THRESHOLD threshold_data = {};
        BOUNDING_BOX box_data = {};
        TRACKING_SPEED tracking_speed = {};

        int ReadParam();

        void ControlTracking(float distance_x, int center_x, int center_y);

        void cb_Box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg);
        void cb_Box(const vision_msgs::Detection2DArray::ConstPtr& box_msg);

        void cb_Image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
        void cb_Request(const std_msgs::String::ConstPtr& request_msg);

        darknet_ros_msgs::BoundingBox GetBoxHighestProbability(const darknet_ros_msgs::BoundingBoxes::ConstPtr &box_msg);
        vision_msgs::Detection2D GetBoxHighestProbability(const vision_msgs::Detection2DArray::ConstPtr &box_msg);



    public:
        ObjectTracking(const ros::NodeHandle& nodeHandle);
};