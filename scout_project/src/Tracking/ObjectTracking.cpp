#include <ObjectTracking.h>

ObjectTracking::ObjectTracking(const ros::NodeHandle &nodeHandle) : 
    
    nh(nodeHandle),
    it(nh),

    pub_tracking(nh.advertise<geometry_msgs::TwistStamped>("/drone/cmd/tracking", 10)),
    
    sub_darknetBox(nh.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 10, &ObjectTracking::cb_Box, this)),
    sub_ssdBox(nh.subscribe<vision_msgs::Detection2DArray>("detectnet/detections", 10, &ObjectTracking::cb_Box, this)),

    sub_request(nh.subscribe<std_msgs::String>("drone/cmd/request", 1, &ObjectTracking::cb_Request, this))

{
    ReadParam();
        
    std::string topic_image = nh.resolveName(camera_data.depth_topic);
    sub_camera = it.subscribeCamera(camera_data.depth_topic, 1024, &ObjectTracking::cb_Image, this);


    camera_data.frame.middle_x = camera_data.frame.x / 2; // intel realsense d435 : 640 x 480
    camera_data.frame.middle_y = camera_data.frame.y / 2;

};


int ObjectTracking::ReadParam()
{
    int error_count = 0;

    if(!nh.getParam("camera_data_width", camera_data.frame.x)){
        ROS_WARN("[FAIL] Get [camera_data_width : %d]", camera_data.frame.x);
        error_count++;
    }

    if(!nh.getParam("camera_data_height", camera_data.frame.x)){
        ROS_WARN("[FAIL] Get [camera_data_height : %d]", camera_data.frame.y);
        error_count++;
    }

    if(!nh.getParam("depth_camera_topic", camera_data.depth_topic)){
        ROS_WARN("[FAIL] Get [depth_camera_topic : %s]", camera_data.depth_topic.c_str());
        error_count++;
    }

    if(!nh.getParam("box_probability_threshold", threshold_data.box_probability)){
        ROS_WARN("[FAIL] Get [box_probability_threshold : %.2lf]", threshold_data.box_probability);
        error_count++;
    }

    if(!nh.getParam("tracking_speed_x", tracking_speed.speed_x)){
        ROS_WARN("[FAIL] Get [tracking_speed_x : %.2lf]", tracking_speed.speed_x);
        error_count++;
    }

    if(!nh.getParam("tracking_accel_x_coeff", tracking_speed.accel_x_coeff)){
        ROS_WARN("[FAIL] Get [tracking_accel_x_coeff : %.2lf]", tracking_speed.accel_x_coeff);
        error_count++;
    }

    if(!nh.getParam("tracking_speed_yaw", tracking_speed.speed_yaw)){
        ROS_WARN("[FAIL] Get [tracking_speed_yaw : %.2lf]", tracking_speed.speed_yaw);
        error_count++;
    }

    if(!nh.getParam("tracking_accel_yaw_coeff", tracking_speed.accel_yaw_coeff)){
        ROS_WARN("[FAIL] Get [tracking_accel_yaw_coeff : %.2lf]", tracking_speed.accel_yaw_coeff);
        error_count++;
    }

    if(!nh.getParam(param_tracking, mode_tracking)){
        ROS_WARN("[FAIL] Get [param_tracking : %s]", mode_tracking ? "true" : "false");
        error_count++;
    }

    if(!nh.getParam("real_or_gazebo", real_or_gazebo)){
        ROS_WARN("[FAIL] Get [real_or_gazebo : %s]", real_or_gazebo ? "true" : "false");
        error_count++;
    }

    if (error_count == 0)
    {
        ROS_INFO("[PARAM] Get [camera_data_width : %d", camera_data.frame.x);
        ROS_INFO("[PARAM] Get [camera_data_height : %d", camera_data.frame.y);
        ROS_INFO("[PARAM] Get [depth_camera_topic : %s", camera_data.depth_topic.c_str());
        ROS_INFO("[PARAM] Get [box_probability_threshold : %.2lf", threshold_data.box_probability);
        ROS_INFO("[PARAM] Get [tracking_speed_x : %.2lf", tracking_speed.speed_x);
        ROS_INFO("[PARAM] Get [tracking_accel_x_coeff : %.2lf", tracking_speed.accel_x_coeff);
        ROS_INFO("[PARAM] Get [tracking_speed_yaw : %.2lf", tracking_speed.speed_yaw);
        ROS_INFO("[PARAM] Get [tracking_accel_yaw_coeff : %.2lf", tracking_speed.accel_yaw_coeff);
        ROS_INFO("[PARAM] Get [param_tracking : %s]", mode_tracking ? "true" : "false");
        ROS_INFO("[PARAM] Get [real_or_gazebo : %s]", real_or_gazebo ? "true" : "false");
    }

    return error_count;
}


// Shutdown when subscribe command from drone_node
void ObjectTracking::cb_Request(const std_msgs::String::ConstPtr &request_msg)
{
    std::string data = request_msg->data;
    
    if (data == "q" || data == "Q")
    {
        ROS_INFO("Exit Object_Tracking node...");
        ros::shutdown();
    }
}


// Store Yolo bounding box with the highest probability
void ObjectTracking::cb_Box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &box_msg)
{
    // Check working box_msg
    if (!box_data.check_box)
    {
        if(!nh.getParam(param_tracking, mode_tracking)){
            ROS_WARN("[FAIL] Get [param_tracking]");
            return;
        }

        if (!mode_tracking)
            return;

        box_data.check_box = true;
        box_data.current_darknetBox = GetBoxHighestProbability(box_msg);

        box_data.class_id = box_data.current_darknetBox.Class;
        box_data.probability = box_data.current_darknetBox.probability;
        box_data.center_x = (box_data.current_darknetBox.xmax + box_data.current_darknetBox.xmin)/2;
        box_data.center_y = (box_data.current_darknetBox.ymax + box_data.current_darknetBox.ymin)/2;

    }
}

// Store Mobilenet bounding box with the highest probability
void ObjectTracking::cb_Box(const vision_msgs::Detection2DArray::ConstPtr &box_msg)
{
    if (!box_data.check_box)
    {
        if(!nh.getParam(param_tracking, mode_tracking)){
            ROS_WARN("[FAIL] Get [param_tracking]");
            return;
        }

        if (!mode_tracking)
            return;
            

        box_data.check_box = true;
        box_data.current_ssdBox = GetBoxHighestProbability(box_msg);

        box_data.class_id = std::to_string(box_data.current_ssdBox.results[0].id);
        box_data.probability = box_data.current_ssdBox.results[0].score;
        box_data.center_x = (int)box_data.current_ssdBox.bbox.center.x;
        box_data.center_y = (int)box_data.current_ssdBox.bbox.center.y;

    }
}

darknet_ros_msgs::BoundingBox ObjectTracking::GetBoxHighestProbability(const darknet_ros_msgs::BoundingBoxes::ConstPtr &box_msg)
{
    int index_highestProbability = 0;

    for (int i = 0; i < box_msg->bounding_boxes.size(); i++)
    {
        if (box_msg->bounding_boxes[i].probability >= box_msg->bounding_boxes[index_highestProbability].probability)
            index_highestProbability = i;
    }

    return box_msg->bounding_boxes[index_highestProbability];
}


vision_msgs::Detection2D ObjectTracking::GetBoxHighestProbability(const vision_msgs::Detection2DArray::ConstPtr &box_msg)
{
    int index_highestProbability = 0;


    for (int i = 0; i < box_msg->detections.size(); i++)
    {
        if (box_msg->detections[i].results[0].score >= box_msg->detections[index_highestProbability].results[0].score)
            index_highestProbability = i;
    }

    return box_msg->detections[index_highestProbability];
}


// Publish velocity for tracking (with depth camera)
void ObjectTracking::cb_Image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if (!box_data.check_box)
        return;

    if (box_data.probability < threshold_data.box_probability)
    {
        ROS_INFO("[DEBUG] Low box_probability [P:%.2lf]", box_data.probability);
        
        box_data.check_box = false;
        return;
    }


    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try
    {
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        image = input_bridge->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("[FAIL] Convert image");
        return;
    }

    // intrinsic.width = info_msg->width;
    // intrinsic.height = info_msg->height;
    // intrinsic.ppx = info_msg->K[2];
    // intrinsic.ppy = info_msg->K[5];
    // intrinsic.fx = info_msg->K[0];
    // intrinsic.fy = info_msg->K[4];
    // intrinsic.model = RS2_DISTORTION_NONE;
    // for (int i = 0; i < info_msg->D.size(); i++)
    // {
    //     intrinsic.coeffs[i] = info_msg->D[i];
    // }


    float pixel[2] = {(float)box_data.center_x, (float)box_data.center_y};
    float point[3] = {0,};
    float *distance = &point[2];

    float depth_in_m = image.at<short int>(cv::Point(box_data.center_x, box_data.center_y));
    
    // if (real_or_gazebo == REAL)
    //     rs2_deproject_pixel_to_point(&point[0], &intrinsic, &pixel[0], depth_in_m / 1000); //In real world
    // else
    //     rs2_deproject_pixel_to_point(&point[0], &intrinsic, &pixel[0], depth_in_m); // In gazebo


    ROS_INFO("[DEBUG] Distance : %f", *distance);

    if (*distance > 9){
        ROS_WARN("[FAIL] Too far [d:%f]", *distance);
    }
    else if (*distance > 0)
        ControlTracking(*distance, box_data.center_x, box_data.center_y);
    else 
        ROS_WARN("[FAIL] Measure distance [d:%f]", *distance);


    CMD_MOVE_DRONE.header.seq++;
    CMD_MOVE_DRONE.header.stamp = ros::Time::now();
    pub_tracking.publish(CMD_MOVE_DRONE);
    

    box_data.check_box = false;
    box_data.current_darknetBox = {};
    CMD_MOVE_DRONE = {};

}

// Publish velocity for tracking
void ObjectTracking::ControlTracking(float distance_x, int box_center_x, int box_center_y)
{
    // Control x-position based on distance
    // keep the distance of threshold.object_distance (m)

    float accel_x = 0;

    if (distance_x < threshold_data.object_distance - threshold_data.object_distance_margin)
    {
        // Distance < threshold + margin -> Go forward
        accel_x = abs(distance_x - (threshold_data.object_distance - threshold_data.object_distance_margin)) * tracking_speed.accel_x_coeff;
        CMD_MOVE_DRONE.twist.linear.x = -1 * (tracking_speed.speed_x + accel_x);
    }
    else if (distance_x > threshold_data.object_distance + threshold_data.object_distance_margin)
    {
        // Distance > threhosld + margin -> Go back
        accel_x = abs(distance_x - (threshold_data.object_distance + threshold_data.object_distance_margin)) * tracking_speed.accel_x_coeff;
        CMD_MOVE_DRONE.twist.linear.x = tracking_speed.speed_x + accel_x;
    }
    else{
        // Distance is in the margin
        CMD_MOVE_DRONE.twist.linear.x = 0;
    }

    // Contrl yaw value based on bounding box's x-axis

    int distance_y_withMargin = camera_data.frame.x * threshold_data.yaw_margin_coeff;
    int distance_y = abs(camera_data.frame.middle_x - box_center_x);
    
    float accel_yaw = (distance_y - distance_y_withMargin) * tracking_speed.accel_yaw_coeff;

    if (box_center_x < (camera_data.frame.middle_x - distance_y_withMargin))
    {
        // left side => Turn left (+ value) 
        CMD_MOVE_DRONE.twist.angular.z = (tracking_speed.speed_yaw + accel_yaw);
    }
    else if (box_center_x > (camera_data.frame.middle_x + distance_y_withMargin))
    {
        // right side => Turn right (- value)
        CMD_MOVE_DRONE.twist.angular.z = -1 * (tracking_speed.speed_yaw + accel_yaw);
    }

    ROS_INFO("[DEBUG] Box [center: %d][distance]:%d][margin:%d]", box_data.center_x, distance_y, distance_y_withMargin);
    ROS_INFO("[DEBUG] Yaw control [V:%lf][A:%lf][BoxDistance:%d]", CMD_MOVE_DRONE.twist.angular.z, accel_yaw, distance_y - distance_y_withMargin);


}