#include <scout.h>

Scout::Scout(const ros::NodeHandle& _nodeHandle):
    nh(_nodeHandle),

    // sub_state(nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Drone::Callback_state, this)),
    // sub_pose(nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Drone::Callback_localPose, this)),
    sub_pose(nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Scout::Callback_localPose, this)),
    
    // sub_tracking(nh.subscribe<geometry_msgs::TwistStamped>("drone/cmd/tracking", 1, &Drone::Callback_tracking, this)),

    sub_request(nh.subscribe<std_msgs::String>("scout/cmd/request", 1, &Scout::Callback_request, this)),
    sub_2d_nav_goal_point(nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &Scout::Callback_goalpoint, this)),
    pub_2d_nav_goal_point(nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10)),



    pub_setLocalVelocity(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
    pub_setLocalPose(nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10)),

    pub_request(nh.advertise<std_msgs::String>("scout/cmd/request", 10)),
    pub_respond(nh.advertise<std_msgs::Header>("scout/cmd/respond", 10)),
    
    // client_arming(nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming")),
    // client_setMode(nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"))

    box_sub(nh.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 10, &Scout::_cb_box, this)),

    pub_move_base_cancel(nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10))


{
    
    set_localPose = {};
    set_localVelocity = {};
    // SetDefaultHeight();

    ReadParam();

    nh.setParam(param_activate_tracking, activate_tracking);

    // drone_data.mode_control = POSITION_MODE;
    // nh.setParam(param_control, POSITION_MODE);

    // drone_data.mode_tracking = false;
    nh.setParam(param_tracking, false);

 
    // if (!CheckConnection(scout_data.timeout_connection))
    // {
    //     Shutdown();
    //     return; 
    // }

    init_keyboard();
}