#include <scout.h>

void Scout::Mission()
{
    now = ros::Time::now();
    if (is_tracking == true)
    {
        // ROS_INFO("check 1");

        // if (now - box_request > ros::Duration(0.1))
        // {
            ROS_INFO("countinue");

        //     box_request = now;


            if(Box_info.Class == "person")
            {
                Box_info.Class="";
                if(Box_info.probability >= 0.7)
                {
                    // cancel_command.stamp.sec = 0;
                    // cancel_command.stamp.nsec = 0;
                    // cancel_command.id = "";

                    pub_move_base_cancel.publish(cancel_command);
                    // MoveVelocity(0, 0, 0);
                    // MoveVelocityAngle(0);
                    ROS_INFO("person detect check_num : 0");
                    check_num = 0;
                }
            }else if (check_num == 0){
                pub_2d_nav_goal_point.publish(save_goalpoint);
                check_num = 1;
                ROS_INFO("mission continue checkpoint : 1 ");
            }
        // }   
    }
}





