#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

// #include <mavros_msgs/State.h>
// #include <mavros_msgs/CommandBool.h>

// #include <rocon_std_msgs/StringArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <cstdlib>

#include <Convert.h>
#include <Keyboard.h>
#include <nav_msgs/Odometry.h>


#define POSITION_MODE 1  // Move by setpoint_position
// #define VELOCITY_MODE 0  // Move by setpoint_velocity

#define TRACKING_DEACTIVATE -1

struct SCOUT_DATA
{
    double time_now;

    double pose[3];
    double pose_angle;

    // bool control_mode;
    
    double setPose[3];
    // double setPose_angle;
    
    double setVel[3];
    // double setVel_angle;
    
    // bool tracking_mode;
    // int activate_tracking;
    // double setVel_tracking[3];
    // double setVel_tracking_angle;

    // mavros_msgs::State state;
    std::string system_status;

    double time_request;
    std::string request;

    // double time_respond;
    std::string respond;
    // std::string respond_msg;
};

static SCOUT_DATA data = {};
static const std::string interface[] = {
        "+-----------------------------------------------------+",
        "|                   Control Pixhawk                   |",
        "|                                                     |",
        "|     1  2  3                           0             |",
        "|     Q  W            G         U       O  P          |",
        "|     A  S  D         T         J                     |",
        "|        X                                            |",
        "|                                                     |",
        "| throttle  UP / DOWN      : U / J                    |",
        "| rotation CCW / Stop / CW : 1 / 2 / 3                |",
        "| FORWARD / BACKWARD       : W / X                    |",
        "| LEFT / STOP / RIGHT      : A / S / D                |",
        "| LAND                     : G                        |",
        "| Control (Pose/Vel)       : 0                        |",
        "| Tracking Mode            : T                        |",
        "| Quit                     : Q                        |",
        "+-----------------------------------------------------+"
};

static const std::string info[] = {
        "+-----------------------------------------------------+",
        "                Information Pixhawk                    ",
        " Time ",
        " Position / Angle :",
        " Arm  / Control   :",
        " Mode / Status    :",
        "-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -",
        " Setpoint Position :",
        " Setpoint Velocity :",
        "-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -",
        " Time [Request] :", " Request :",
        " Time [Respond] :", " Respond :",
        " Message :",
        "-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -",
        " Tracking :",
        " Tracking Velocity :",
        "+-----------------------------------------------------+"
};
// 보낸 메세지랑 현재 입력된 메세지를 구분해야 할 듯 하다.

void PrintData(){
    int index = 0;
    
    printf("%s\n", info[index++].c_str());
    printf("%s\n", info[index++].c_str());

    // Time / Position, Angle / Arm, Control / Mode, Status
    printf("%s %lf \n", info[index++].c_str(), data.time_now);
    printf("%s [%.2lf, %.2lf, %.2lf] / [%.2lf] \n", info[index++].c_str(), data.pose[0], data.pose[1], data.pose[2], data.pose_angle);
    // printf("%s %s / %s \n", info[index++].c_str(), data.state.armed ? "Armed" : "Disarmed", data.control_mode ? "POSITION":"VELOCITY");
    // printf("%s %s / %s \n", info[index++].c_str(), data.state.mode.c_str(), data.system_status.c_str());
    
    printf("%s\n", info[index++].c_str());

    // Setpoint Position / Setpoint Velocity
    // printf("%s [%.2lf, %.2lf, %.2lf] / [%.2lf] \n", info[index++].c_str(), data.setPose[0], data.setPose[1], data.setPose[2], data.setPose_angle);
    // printf("%s [%.2lf, %.2lf, %.2lf] / [%.2lf] \n", info[index++].c_str(), data.setVel[0], data.setVel[1], data.setVel[2], data.setVel_angle);

    printf("%s\n", info[index++].c_str());

    // Request & time / Respond & time / Respond message
    // printf("%s %lf ", info[index++].c_str(), data.time_request);
    // printf("%s %s\n", info[index++].c_str(), data.request.c_str());
    // printf("%s %lf ", info[index++].c_str(), data.time_respond);
    // printf("%s %s \n", info[index++].c_str(), data.respond.c_str());
    // printf("%s %s \n", info[index++].c_str(), data.respond_msg.c_str());
    // printf("%s\n", info[index++].c_str());

    // Tracking / Tracking Velocity
    // printf("%s %s \n", info[index++].c_str(), data.activate_tracking == TRACKING_DEACTIVATE ? "Deactivate" : (data.tracking_mode ? "True" : "False"));
    // printf("%s [%.2lf, %.2lf, %.2lf] / [%.2lf] \n", info[index++].c_str(), data.setVel_tracking[0], data.setVel_tracking[1], data.setVel_tracking[2], data.setVel_tracking_angle);

    // printf("%s\n", info[index++].c_str());


}

void PrintInterface(){
    for (int i = 0; i < sizeof(interface)/sizeof(std::string); i++)
    {
        printf("%s\n", interface[i].c_str());
    }
}

// void callback_State(const mavros_msgs::State::ConstPtr& msg_state){
//     data.state = *msg_state;
    
//     switch (data.state.system_status){
//         case 0 : data.system_status = "UNINIT"; break;
//         case 1 : data.system_status = "BOOT"; break;
//         case 2 : data.system_status = "CALIBRATING"; break;
//         case 3 : data.system_status = "STANDBY"; break;
//         case 4 : data.system_status = "ACTIVE"; break;
//         case 5 : data.system_status = "CRITICAL"; break;
//         case 6 : data.system_status = "EMERGENCY"; break;
//         case 7 : data.system_status = "POWEROFF"; break;
//         case 8 : data.system_status = "FLIGHT_TERMINATION"; break;

//         default: data.system_status = "ERROR"; break;
//     }
// }

// void callback_Respond(const std_msgs::Header::ConstPtr& msg_respond)
// {
//     data.time_respond = msg_respond->stamp.toSec();
//     data.respond = msg_respond->frame_id[0];
//     data.respond_msg = msg_respond->frame_id.substr(1, msg_respond->frame_id.length());
// }


void callback_Pose(const nav_msgs::Odometry::ConstPtr& msg_pose){
    data.pose[0] = msg_pose->pose.pose.position.x;
    data.pose[1] = msg_pose->pose.pose.position.y;
    data.pose[2] = msg_pose->pose.pose.position.z;

    data.pose_angle = radian_to_degree(quaternion_to_euler(msg_pose->pose.pose.orientation.x,
        msg_pose->pose.pose.orientation.y, msg_pose->pose.pose.orientation.z, msg_pose->pose.pose.orientation.w)[2]);

    if (data.pose_angle < 0)
        data.pose_angle += 360;
}

// void callback_SetLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg_setLocalPose)
// {
//     data.setPose[0] = msg_setLocalPose->pose.position.x;
//     data.setPose[1] = msg_setLocalPose->pose.position.y;
//     // data.setPose[2] = msg_setLocalPose->pose.position.z;

//     data.setPose_angle = radian_to_degree(quaternion_to_euler(msg_setLocalPose->pose.orientation.x, 
//         msg_setLocalPose->pose.orientation.y, msg_setLocalPose->pose.orientation.z, msg_setLocalPose->pose.orientation.w)[2]);

//     if (data.setPose_angle < 0)
//         data.setPose_angle += 360;
// }

// void callback_SetLocalVelocity(const geometry_msgs::Twist::ConstPtr& msg_setLocalVel)
// {
//     data.setVel[0] = msg_setLocalVel->linear.x;
//     data.setVel[1] = msg_setLocalVel->linear.y;
//     data.setVel[2] = msg_setLocalVel->linear.z;

//     data.setVel_angle = msg_setLocalVel->angular.z;
// }

// void callback_Tracking(const geometry_msgs::TwistStamped::ConstPtr& msg_tracking)
// {
//     data.setVel_tracking[0] = msg_tracking->twist.linear.x;
//     data.setVel_tracking[1] = msg_tracking->twist.linear.y;
//     data.setVel_tracking[2] = msg_tracking->twist.linear.z;

//     data.setVel_tracking_angle = msg_tracking->twist.angular.z;
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interface_node");
    ros::NodeHandle nh;

    ros::Publisher pub_request;

    ros::Subscriber sub_pose;
    ros::Subscriber sub_respond;
    ros::Subscriber sub_tracking;
    ros::Subscriber sub_state;
    ros::Subscriber sub_setLocalPose;
    ros::Subscriber sub_setLocalVeclocity;

    // std::string control_param = "drone/mode/control";
    // std::string tracking_param = "drone/mode/tracking";
    // std::string tracking_activate_param = "drone/activate/tracking";

    // pub_request = nh.advertise<std_msgs::String>("/drone/cmd/request", 5);

    sub_pose = nh.subscribe<nav_msgs::Odometry>("/odom", 1, callback_Pose);
    // sub_respond = nh.subscribe<std_msgs::Header>("/drone/cmd/respond", 10, callback_Respond);
    // sub_tracking = nh.subscribe<geometry_msgs::TwistStamped>("drone/cmd/tracking", 10, callback_Tracking);
    // sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, callback_State);
    // sub_setLocalPose = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10, callback_SetLocalPose);
    // sub_setLocalVeclocity = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, callback_SetLocalVelocity);

    // if (!nh.getParam(tracking_activate_param, data.activate_tracking))
    // {
    //     data.activate_tracking = TRACKING_DEACTIVATE;
    //     ROS_INFO("[FAIL] Get [tracking_activate_param : %d]", data.activate_tracking);
    // }

    init_keyboard();

    std_msgs::String msg_cmd = {};

    while (ros::ok())
    {
        int result = std::system("clear");

        data.time_now = ros::Time::now().toSec();
        // nh.getParam(control_param, data.control_mode);
        // nh.getParam(tracking_param, data.tracking_mode);

        if (_kbhit())
        {   
            data.request.replace(0, 1, 1, _getch());
            msg_cmd.data = data.request;

            data.time_request = ros::Time::now().toSec();
            
            if (data.request == "q" || data.request == "Q")
            {
                pub_request.publish(msg_cmd);
                
                printf("\n");
                printf("Shutdown Code\n\n");
                nh.shutdown();
                break;
            }
            else if (data.request != "0")
            {
                pub_request.publish(msg_cmd);
            }
        }


        if (data.request == "0")
        {
            PrintInterface();
            
            if (ros::Time::now().toSec() - data.time_request >= 2.0)
            {
                data.request = "";
                data.time_request = 0;
            }
        }
        else
        {
            PrintData();
        }
        

        ros::spinOnce();
        ros::Rate(10.0).sleep();
    }

    close_keyboard();

    return 0;

}
