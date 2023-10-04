#include <scout.h>

void Scout::Mission()
{
    now = ros::Time::now();
    if (is_tracking == true)
    {
        // ROS_INFO("check 1");

        // if (now - box_request > ros::Duration(0.1))
        // {
            // ROS_INFO("countinue");

            if (yaw_check==true){
                Set2dnavyaw();
                yaw_check=false;
                ROS_INFO("%.2lf",set2dnav.euler_yaw);

                first_goalpoint.header=save_goalpoint.header;

                first_goalpoint.pose.position.x=save_goalpoint.pose.position.x + (-3*cos(set2dnav.euler_yaw));
                first_goalpoint.pose.position.y=save_goalpoint.pose.position.y + (-3*sin(set2dnav.euler_yaw));
                
                first_goalpoint.pose.orientation=save_goalpoint.pose.orientation;

                

                second_goalpoint.header=save_goalpoint.header;

                second_goalpoint.pose.position.x=save_goalpoint.pose.position.x + (-1.5*cos(set2dnav.euler_yaw));
                second_goalpoint.pose.position.y=save_goalpoint.pose.position.y + (-1.5*sin(set2dnav.euler_yaw));
                
                second_goalpoint.pose.orientation=save_goalpoint.pose.orientation;

            }

            


            ROS_INFO("%f",first_goalpoint.pose.position.x);
            ROS_INFO("%f",second_goalpoint.pose.position.x);
            if (check_num == 0 && yaw_check==false)
            {
                pub_2d_nav_goal_point.publish(first_goalpoint);

                
                check_num=1;
                ROS_INFO("check num 1");

            }else if (check_num == 1 && movebase_goal_result.status.status == 3)
            {
                pub_2d_nav_goal_point.publish(second_goalpoint);
                check_num=2;
                ROS_INFO("check num 2");

            }else if (check_num ==2 && movebase_goal_result.status.status == 3)
            {
                ROS_INFO("goal Reach");
                std_msgs::String goal_reached_hello_world;
                goal_reached_hello_world.data = "hello world";
                pub_goal_reached.publish(goal_reached_hello_world);
                // check_num=3;
            }


        //     box_request = now;


        //     if(Box_info.Class == "person")
        //     {
        //         Box_info.Class="";
        //         if(Box_info.probability >= 0.7)
        //         {
        //             // cancel_command.stamp.sec = 0;
        //             // cancel_command.stamp.nsec = 0;
        //             // cancel_command.id = "";

        //             pub_move_base_cancel.publish(cancel_command);
        //             // MoveVelocity(0, 0, 0);
        //             // MoveVelocityAngle(0);
        //             ROS_INFO("person detect check_num : 0");
        //             check_num = 0;
        //         }
        //     }else if (check_num == 0){
        //         pub_2d_nav_goal_point.publish(save_goalpoint);
        //         check_num = 1;
        //         ROS_INFO("mission continue checkpoint : 1 ");
        //     }
        // // }   
    }
}







