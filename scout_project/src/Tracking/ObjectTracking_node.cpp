#include <ObjectTracking.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "object_tracking_node");
    ros::NodeHandle nh;

    ROS_INFO("Object_Tracking node start");
    ObjectTracking ob(nh);
    
    ros::spin();

    return 0;
}