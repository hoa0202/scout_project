#include <scout.h>

/* MAV_STATE -> system_status 
https://mavlink.io/en/messages/common.html
0	MAV_STATE_UNINIT	Uninitialized system, state is unknown.
1	MAV_STATE_BOOT	System is booting up.
2	MAV_STATE_CALIBRATING	System is calibrating and not flight-ready.
3	MAV_STATE_STANDBY	System is grounded and on standby. It can be launched any time.
4	MAV_STATE_ACTIVE	System is active and might be already airborne. Motors are engaged.
5	MAV_STATE_CRITICAL	System is in a non-normal flight mode. It can however still navigate.
6	MAV_STATE_EMERGENCY	System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
7	MAV_STATE_POWEROFF	System just initialized its power-down sequence, will shut down now.
8	MAV_STATE_FLIGHT_TERMINATION	System is terminating itself.
*/


// void Scout::Callback_state(const mavros_msgs::State::ConstPtr& state_msg)
// {
//     if (drone_data.current_state.mode != state_msg->mode){
//         cmd_info.success = true;
//         ROS_INFO("[INFO] Mode Changed [%s->%s]", drone_data.current_state.mode.c_str(), state_msg->mode.c_str());
        
//         if (state_msg->mode == "OFFBOARD")
//         {   
//             // Flying & Arming+offboard => set current Position & set default height for takeoff
//             if (drone_data.current_state.armed){
//                 SetCurrentPose();
//                 SetCurrentYaw();
                
//                 // Check height and Set height
//                 if (current_pose.pose.position.z <= 0.4)
//                     SetDefaultHeight();
//             }
//             else{
//                 // 
//                 SetCurrentPose();
//                 SetCurrentYaw();
//                 SetDefaultHeight();
//             }
//         }
//     }    

//     // Check failsafeDrone
//         }
//         else{
//             ROS_ERROR("[EMERGENCY] Dump Value : %d", state_msg->system_status);
//             return;
//         }
//     }

//     drone_data.current_state = *state_msg;

// }



// void Drone::Callback_tracking(const geometry_msgs::TwistStamped::ConstPtr& tracking_msg)
// {
//     // Check tracking mode
//     if (drone_data.mode_tracking == false)
//         return;

//     // Check working command
//     if (tracking_data.check_data == true)
//         return;

//     // Store time when the command come in (Timeout)
//     tracking_data.time_in = ros::Time::now().toSec();

//     tracking_detection_timeout.trigger_hover = false;
//     tracking_detection_timeout.time_last_in = tracking_data.time_in;
    
//     // Communication Timeout (if Subscribed message is old, Don't execute the message)
//     if (tracking_data.time_in - tracking_msg->header.stamp.toSec() >= drone_data.timeout_communication)
//     {
//         ROS_WARN("[TIMEOUT] Old Tracking data [Du:%lf]", tracking_data.time_in - tracking_msg->header.stamp.toSec());
//         tracking_data = {};
//         return;
//     }

//     // Tracking trigger On
//     tracking_data.check_data = true;
//     tracking_data.msg = *tracking_msg;
    
//     // ROS_INFO("[DEBUG] tracking data [%lf]", tracking_data.time_in);

// }

// Store a request to execute the request.
void Scout::Callback_request(const std_msgs::String::ConstPtr& request_msg)
{
    if (cmd_info.command == '\0')
    {
        char ch = *(request_msg->data.c_str());

        if (ch == '`' || ch == 'Q')
        {
            Shutdown();
            return;
        }

        CheckCommand(ch);      

        std_msgs::Header respond_msg;
        respond_msg.stamp = ros::Time::now();
        respond_msg.frame_id += ch;
        respond_msg.frame_id += "received";
        pub_respond.publish(respond_msg);
    }
}


// Store current position
void Scout::Callback_localPose(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
    current_pose = *pose_msg;
}


void Scout::Callback_goalpoint(const geometry_msgs::PoseStamped::ConstPtr& navgoalpoint_msg)
{

    save_goalpoint = *navgoalpoint_msg;

}

void Scout::Callback_goalpoint_result(const move_base_msgs::MoveBaseActionResult::ConstPtr& movebase_goal_result_msg)
{
    movebase_goal_result= *movebase_goal_result_msg;
}

// void Scout::_cb_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &box_msg)
// {
//     darknet_ros_msgs::BoundingBox Box = box_msg->bounding_boxes[0];

//     // box_request = ros::Time::now();
//     now = ros::Time::now();
//     Box_info.Class = Box.Class;
//     Box_info.probability = Box.probability;
    
// }