/**
 * @file PX4 drone control and tracking person
 * @author Daehyen Kwon (dh0708@kumoh.ac.kr) WENS Lab
 * @date 2022.06.10
 * @version 1.1.3 
 */


#pragma once

#include <signal.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
// #include <rocon_std_msgs/StringArray.h>

#include <Keyboard.h>
#include <Convert.h>

#include <actionlib_msgs/GoalID.h>



#include <darknet_ros_msgs/BoundingBoxes.h>


#define POSITION_MODE 1  // Move by setpoint_position
// #define VELOCITY_MODE 0  // Move by setpoint_velocity

#define TRACKING_ON  1
#define TRACKING_OFF 0
#define TRACKING_DEACTIVATE -1


// Command information of keyboard input
struct CMD_INFO
{
    double time_in;     // Time when the command was entered
    double time_start;  // Time when the command was executed
    char command;
    short try_max;
    short try_num;
    bool success;
};

struct SCOUT_DATA
{
    double timeout_in = 0.5;    // Timeout about time_in
    double timeout_start = 0.5; // Timeout about time_start
    double timeout_connection = 3.0;
    double timeout_detection = 1.5;
    double timeout_communication = 3.0;

    bool mode_control = false;
    bool mode_tracking = false;

    double current_yaw; // radian
    // double default_height = 0.0;

    double move_speed = 0.5;
    double move_angle_speed = 1.0;

    // mavros_msgs::State current_state;
    ros::Time current_time;
    ros::Time prev_time;
};

struct SCOUT_REQUEST
{
    bool arm;
    bool mode_control;
    bool mode_tracking;
    double velocity[3];
    std_msgs::String mode;
};

struct TRACKING_DATA
{
    bool check_data;
    double time_in;
    double time_start;
    geometry_msgs::TwistStamped msg;
};

struct TRACKING_VALUE_TIMEOUT
{
    bool trigger_stop;
    double time_in;
    double time_zero_veloity;
};

struct TRACKING_DETECTION_TIMEOUT
{
    bool trigger_hover;     
    double time_last_in;
};

class Scout{
    private:
        ros::NodeHandle nh;
    
        ros::Subscriber sub_state;
        ros::Subscriber sub_pose;
        // ros::Subscriber sub_tracking;

        ros::Subscriber sub_request;

        ros::Publisher pub_setLocalVelocity;
        ros::Publisher pub_setLocalPose;

        ros::Publisher pub_request;
        ros::Publisher pub_respond;

        ros::Publisher pub_move_base_cancel;

        //========================= box 관련
        ros::Subscriber box_sub;


        //=========================

        ros::Subscriber sub_2d_nav_goal_point;
        ros::Publisher pub_2d_nav_goal_point;


        // ros::ServiceClient client_arming;
        // ros::ServiceClient client_setMode;

        std::string param_tracking = "scout/mode/tracking";
        // std::string param_control = "drone/mode/control";
        std::string param_activate_tracking = "scout/activate/tracking";
        
        int activate_tracking = TRACKING_DEACTIVATE;

        int check_num = 1;

        bool is_tracking = false;

        geometry_msgs::PoseStamped set_localPose;
        nav_msgs::Odometry current_pose;

        geometry_msgs::Twist set_localVelocity;

        actionlib_msgs::GoalID cancel_command;


        geometry_msgs::PoseStamped save_goalpoint;

        darknet_ros_msgs::BoundingBox Box_info;


        CMD_INFO cmd_info = {};
        SCOUT_DATA scout_data = {};
        SCOUT_REQUEST scout_request = {};
        // TRACKING_DATA tracking_data = {};
        // TRACKING_VALUE_TIMEOUT tracking_value_timeout = {};
        TRACKING_DETECTION_TIMEOUT tracking_detection_timeout = {};

        ros::Rate rate = ros::Rate(10.0);

        ros::Time now;
        ros::Time box_request;


       
        bool ReadParam();
        // void SetInit();
        void Shutdown();

        bool CheckTimeout(double time_input, double timeout);
        void CheckCommand(char input);
        bool CheckCommandSuccess();
        // bool CheckConnection(double timeout);
        // bool CheckArm(bool requested_arm);
        // bool CheckFlying();
        // bool CheckMode(std_msgs::String mode);
        // bool CheckControlMode(bool requested_mode);
        // bool CheckTracking(bool requested_mode);

        void PrintCMDSuccess(CMD_INFO &data);
        // void PrintTrackingSucccess();
        void InitCMDData(CMD_INFO &data);
        void DoCommand();
        // void DoTracking();

        void SetCurrentPose();
        void SetCurrentYaw();
        // void SetDefaultHeight();


        // void Arming();
        // void ChangeMode(std_msgs::String mode);

        void MovePosition(double requested_poseX, double requested_poseY, double requested_poseZ);
        void MoveVelocity(double requetsed_velocityX, double requetsed_velocityY, double requetsed_velocityZ);

        void MovePositionAngle(double requested_angle);
        void MoveVelocityAngle(double requested_angleVelocity);

        void Hovering();

        // void Change_ControlMode(bool requested_move);
        // void Change_TrackingMode(bool requested_mode);

        // bool CheckTrackingValue();

        // void Land();

        // void Callback_state(const mavros_msgs::State::ConstPtr& state_msg);
        void Callback_localPose(const nav_msgs::Odometry::ConstPtr& pose_msg);
        void Callback_tracking(const geometry_msgs::TwistStamped::ConstPtr& tracking_msg);
        void Callback_request(const std_msgs::String::ConstPtr& request_msg);
        void Callback_goalpoint(const geometry_msgs::PoseStamped::ConstPtr& navgoalpoint_msg);


        //============= darknet
        void _cb_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg);
        //=============

    public:
        explicit Scout(const ros::NodeHandle& _nodeHandle);
        void Control(); // Main control code
        void Mission();

};