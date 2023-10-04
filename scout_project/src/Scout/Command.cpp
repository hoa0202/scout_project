#include <scout.h>

// Read launchfile parameter
bool Scout::ReadParam()
{
    int error_count = 0;

    if(!nh.getParam("timeout_in", scout_data.timeout_in)){
        ROS_WARN("[FAIL] Get [timeout_in : %.1lf]", scout_data.timeout_in);
        error_count++;
    }
        
    if(!nh.getParam("timeout_start", scout_data.timeout_start)){
        ROS_WARN("[FAIL] Get [timeout_start : %.1lf]", scout_data.timeout_start);
        error_count++;
    }
    
    // if(!nh.getParam("timeout_connection", scout_data.timeout_connection)){
    //     ROS_WARN("[FAIL] Get [timeout_connection : %.1lf]", scout_data.timeout_connection);
    //     error_count++;
    // }

    // if(!nh.getParam("default_height", scout_data.default_height)){
    //     ROS_WARN("[FAIL] Get [default_height : %.2lf]", scout_data.default_height);
    //     error_count++;
    // }

    if(!nh.getParam("activate_tracking", activate_tracking)){
        ROS_WARN("[FAIL] Get [activate_tracking : %d]", activate_tracking);
        error_count++;
    }

    if (error_count == 0)
    {
        ROS_INFO("[PARAM] Get [timeout_in : %.1lf]", scout_data.timeout_in);
        ROS_INFO("[PARAM] Get [timeout_start : %.1lf]", scout_data.timeout_start);
        // ROS_INFO("[PARAM] Get [timeout_connection : %.1lf]", scout_data.timeout_connection);
        // ROS_INFO("[PARAM] Get [default_height : %.2lf]", scout_data.default_height);
        ROS_INFO("[PARAM] Get [activate_tracking : %d]", activate_tracking);
    }

    return error_count;
}


// Check px4 connection
// bool Drone::CheckConnection(double timeout)
// {
//     double time_start = 0;

//     ROS_INFO("[TRY] Connect....");

//     while(ros::ok())
//     {
//         if (time_start == 0)
//             time_start = ros::Time::now().toSec();

//         scout_data.current_time = ros::Time::now();

//         if (scout_data.current_state.connected)
//         {
//             ROS_INFO("[SUCCESS] Connected");
//             return true;
//         }

//         if (CheckTimeout(time_start, timeout)){
//             ROS_ERROR("[TIMEOUT] Connection");
//             return false;
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     ROS_ERROR("[FAIL] Connect");
//     return false;
// }



// Check Arm
// bool Drone::CheckArm(bool requested_arm)
// {
//     if (scout_data.current_state.armed == requested_arm)
//         return true;
//     return false;
// }

// Check if Drone is flying
// bool Drone::CheckFlying()
// {
//     // system_status 4 = MAV_STATE_ACTIVE
//     if (scout_data.current_state.armed == true && scout_data.current_state.system_status == 4){
//         return true;
//     }

//     return false;
// }

// Check if Mode is same with parameter
// bool Drone::CheckMode(std_msgs::String mode)
// {
//     if (scout_data.current_state.mode == mode.data)
//         return true;

//     return false;
// }

// void Drone::Arming()
// {
//     if (CheckArm(true))
//     {
//         ROS_ERROR("Already Armed");
//         return;
//     }

//     scout_request.arm = true;

//     mavros_msgs::CommandBool cmd_arm;
//     cmd_arm.request.value = scout_request.arm;

//     client_arming.call(cmd_arm);

//     if (cmd_arm.response.success)
//     {ne::Hovering()
// {
//     if(scout_data.mode_control == POSITION_MODE) 
//         pub_setLocalPose.publish(set_localPose);
//     else{
//         SetCurrentYaw();
//         pub_setLocalVelocity.publish(set_localVelocity);
//     }
// }
//         ROS_INFO("[Try] Send Arming Command [R:%d  S:%d]", cmd_arm.response.result, cmd_arm.response.success);
//     }
//     else
//     {
//         ROS_WARN("[FAIL] Send Arming Command [R:%d  S:%d]", cmd_arm.response.result, cmd_arm.response.success);
//     }
// }

// Change PX4 mode as parameter
// void Drone::ChangeMode(std_msgs::String mode)
// {
//     if (CheckMode(mode))
//     {
//         ROS_WARN("[FAIL] Already %s mode", scout_data.current_state.mode.c_str());
//         return;
//     }

//     ROS_INFO("[TRY] Change Mode to [%s]", mode.data.c_str());

//     mavros_msgs::SetMode cmd_mode;
//     cmd_mode.request.base_mode = 0;
//     cmd_mode.request.custom_mode = mode.data;

//     client_setMode.call(cmd_mode);
// }

// Set local position as current position
void Scout::SetCurrentPose()
{
    set_localPose.pose.position.x = current_pose.pose.pose.position.x;
    set_localPose.pose.position.y = current_pose.pose.pose.position.y;
    set_localPose.pose.position.z = current_pose.pose.pose.position.z;
}

// Set Local yaw as current yaw
void Scout::SetCurrentYaw()
{
    set_localPose.pose.orientation.w = current_pose.pose.pose.orientation.w;
    set_localPose.pose.orientation.x = current_pose.pose.pose.orientation.x;
    set_localPose.pose.orientation.y = current_pose.pose.pose.orientation.y;
    set_localPose.pose.orientation.z = current_pose.pose.pose.orientation.z;

    double *current_RPY = quaternion_to_euler(set_localPose.pose.orientation.x, set_localPose.pose.orientation.y,
                                              set_localPose.pose.orientation.z, set_localPose.pose.orientation.w);

    if (current_RPY[2] < 0)
        current_RPY[2] = current_RPY[2] + M_PI * 2;
    else if (current_RPY[2] > M_PI * 2)
        current_RPY[2] = current_RPY[2] - M_PI * 2;

    scout_data.current_yaw = current_RPY[2];

    delete[] current_RPY;
}

// Move Drone by setpoint_position
void Scout::MovePosition(double requested_poseX, double requested_poseY, double requested_poseZ)
{
    // Stop
    if (requested_poseX == 0 && requested_poseY == 0 && requested_poseZ == 0)
    {
        SetCurrentPose();

    }

    else
    {
        double goal_position[3] = {0, 0, requested_poseZ};

        if (requested_poseX > 0)
        {
            goal_position[0] = requested_poseX * cos(scout_data.current_yaw);
            goal_position[1] = requested_poseX * sin(scout_data.current_yaw);
        }
        else if (requested_poseX < 0)
        {
            goal_position[0] = abs(requested_poseX) * -cos(scout_data.current_yaw);
            goal_position[1] = abs(requested_poseX) * -sin(scout_data.current_yaw);
        }

        if (requested_poseY > 0)
        {
            goal_position[0] = requested_poseY * sin(scout_data.current_yaw);  //-cos(current_yaw + M_PI_2);
            goal_position[1] = requested_poseY * -cos(scout_data.current_yaw); //-sin(current_yaw + M_PI_2);
        }
        else if (requested_poseY < 0)
        {
            goal_position[0] = abs(requested_poseY) * -sin(scout_data.current_yaw); // cos(current_yaw + M_PI_2);
            goal_position[1] = abs(requested_poseY) * cos(scout_data.current_yaw);  // sin(current_yaw + M_PI_2);
        }

        set_localPose.pose.position.x += goal_position[0];
        set_localPose.pose.position.y += goal_position[1];
        set_localPose.pose.position.z += goal_position[2];


    }
}

/**
 * @brief Turn Drone in position mode
 * 
 * @param requested_angle radian
 */
void Scout::MovePositionAngle(double requested_angle)
{
    if (requested_angle == 0)
    {
        SetCurrentYaw();
    }
    else
    {
        double goal_radian = scout_data.current_yaw + degree_to_radian(requested_angle);

        if (goal_radian >= M_PI * 2)
            goal_radian -= M_PI * 2;
        else if (goal_radian <= 0)
            goal_radian += M_PI * 2;

        scout_data.current_yaw = goal_radian;

        double *q;
        q = euler_to_quaternion(0, 0, scout_data.current_yaw);

        set_localPose.pose.orientation.x = q[0];
        set_localPose.pose.orientation.y = q[1];
        set_localPose.pose.orientation.z = q[2];
        set_localPose.pose.orientation.w = q[3];

        delete[] q;
    }
}

// Move drone by setpoint_velocity
void Scout::MoveVelocity(double requested_velocityX, double requested_velocityY, double requested_velocityZ)
{
    scout_request.velocity[0] = requested_velocityX;
    scout_request.velocity[1] = requested_velocityY;
    scout_request.velocity[2] = requested_velocityZ;

    // if (requested_velocityX == 0 && requested_velocityY == 0 && requested_velocityZ == 0){
    //     set_localVelocity.linear = {};
    //     SetCurrentYaw();
    //                     ROS_INFO("test1 : %.1lf]", set_localVelocity.linear);

    // }

    // else{
    //     double goal_speed[3] = {0, 0, requested_velocityZ};

    //     if (requested_velocityX > 0)
    //     {
    //         goal_speed[0] = requested_velocityX * cos(scout_data.current_yaw);
    //         goal_speed[1] = requested_velocityX * sin(scout_data.current_yaw);
    //     }
    //     else if (requested_velocityX < 0)
    //     {
    //         goal_speed[0] = abs(requested_velocityX) * -cos(scout_data.current_yaw);
    //         goal_speed[1] = abs(requested_velocityX) * -sin(scout_data.current_yaw);
    //     }

    //     if (requested_velocityY > 0)
    //     {
    //         goal_speed[0] = requested_velocityY * sin(scout_data.current_yaw);  //-cos(current_yaw + M_PI_2);
    //         goal_speed[1] = requested_velocityY * -cos(scout_data.current_yaw); //-sin(current_yaw + M_PI_2);
    //     }
    //     else if (requested_velocityY < 0)
    //     {
    //         goal_speed[0] = abs(requested_velocityY) * -sin(scout_data.current_yaw); // cos(current_yaw + M_PI_2);
    //         goal_speed[1] = abs(requested_velocityY) * cos(scout_data.current_yaw);  // sin(current_yaw + M_PI_2);
    //     }

    //     set_localVelocity.linear.x = goal_speed[0];
    //     set_localVelocity.linear.y = goal_speed[1];
    //     set_localVelocity.linear.z = goal_speed[2];
    //             ROS_INFO("test 2: %.1lf, %.1lf, %.1lf]", set_localVelocity.linear.x, set_localVelocity.linear.y, set_localVelocity.linear.z);

    // }

    set_localVelocity.linear.x = scout_request.velocity[0];
    set_localVelocity.linear.y = scout_request.velocity[1];
    set_localVelocity.linear.z = scout_request.velocity[2];
}

// Trun drone by setpoint_velocity
void Scout::MoveVelocityAngle(double requested_angleVelocity)
{
    set_localVelocity.angular.z = requested_angleVelocity;
    // if (requested_angleVelocity == 0)
    //     SetCurrentYaw();
}

// void Drone::Land()
// {
//     if (!CheckFlying())
//     {
//         ROS_WARN("[FAIL] Not Flying [Arm:%d , Status:%d]", scout_data.current_state.armed, scout_data.current_state.system_status);
//         return;
//     }

//     Change_TrackingMode(false);

//     scout_request.mode.data = "AUTO.LAND";
//     ChangeMode(scout_request.mode); // Change mode as AUTO.LAND
// }

void Scout::Hovering()
{
    // if(scout_data.mode_control == POSITION_MODE)
    //     pub_setLocalPose.publish(set_localPose);
    // else{
    //     SetCurrentYaw();
        pub_setLocalVelocity.publish(set_localVelocity);
    // }
}

// void Drone::SetDefaultHeight()
// {
//     set_localPose.pose.position.z = scout_data.default_height;
// }

void Scout::Shutdown()
{
    std_msgs::String msg_shutdown;
    msg_shutdown.data = "`";
    pub_request.publish(msg_shutdown); // Transfer shutdown cmd to other node 

    ROS_INFO("[EXIT] Scout node");
    close_keyboard();
    ros::shutdown();
}


// bool Drone::CheckControlMode(bool requested_mode)
// {
//     if (scout_data.mode_control == requested_mode)
//     {
//         nh.setParam(param_control, scout_data.mode_control);
//         return true;
//     }
    
//     return false;
// }

// bool Drone::CheckTracking(bool requested_mode)
// {
//     if (scout_data.mode_tracking == requested_mode)
//     {
//         nh.setParam(param_tracking, scout_data.mode_tracking);
//         return true;
//     }
    
//     return false;
// }



// void Drone::Change_ControlMode(bool requested_move)
// {
//     if (requested_move == POSITION_MODE)
//     {
//         SetCurrentPose();
//         SetCurrentYaw();
//     }
//     else{
//         set_localVelocity = {};
//     }

//     scout_request.mode_control = requested_move;
//     scout_data.mode_control = requested_move;
// }


// void Scout::Change_TrackingMode(bool requested_mode)
// {
//     // if (requested_mode == TRACKING_ON)
//     // {
//     //     Change_ControlMode(VELOCITY_MODE);
//     // }
//     // else
//     // {
//     //     Change_ControlMode(POSITION_MODE);
//     // }

//     scout_request.mode_tracking = requested_mode;
//     scout_data.mode_tracking = requested_mode;

//     nh.setParam(param_tracking, requested_mode);
// }


void Scout::Set2dnavyaw()
{

    double *current_RPY = quaternion_to_euler(save_goalpoint.pose.orientation.x, save_goalpoint.pose.orientation.y,
                                              save_goalpoint.pose.orientation.z, save_goalpoint.pose.orientation.w);

    if (current_RPY[2] < 0)
        current_RPY[2] = current_RPY[2] + M_PI * 2;
    else if (current_RPY[2] > M_PI * 2)
        current_RPY[2] = current_RPY[2] - M_PI * 2;

    set2dnav.euler_yaw = current_RPY[2];

    delete[] current_RPY;
}