#include <mainNode.h>
#include <virtualConstraintsNode.h>

int main(int argc, char **argv)
{
    
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
//     robot_interface_ROS robot;
    robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
//     tf::Vector3 distance_foots;
// //    robot.

//--------initialize robot so that q1 is exactly 0---------
    if (ros::ok())
    {
    robot.sense(); /*inside there is ros::spinOnce*/
    VC.straighten_up_action();

    }
//---------------------------------------------------------
 
    while (ros::ok()) 
    {
// //         ROS_INFO("entered");
        VC.get_robot().sense();
        
//         VC.check_q1();
//         VC.calc_q1();
//         VC.send_point();
        VC.calc_trajectory();
        

        

    }




}







// Eigen::Vector3d ret;
// Eigen::Vector3d vel;
// Eigen::Vector3d acc;
// 
// Eigen::Vector3d starting_position;
// Eigen::Vector3d ending_position;
// bool flag = true;
// starting_position << 0.0, -0.1, -0.96;
// ending_position << 0.0, -0.1, -0.96;
// double init_time = ros::Time::now().toSec();
// double starting_time = ros::Time::now().toSec();
// double ending_time = starting_time + 5;
// 
// while (ros::ok())
// {
//         if (ros::Time::now().toSec() >= init_time+6)
//         {
//             if (flag == true)
//             {
// //                 ROS_INFO("entered");
//                 starting_position << 0.0, -0.1, -0.96;
//                 ending_position << 0.2, -0.1, -0.96;
//                 starting_time = ros::Time::now().toSec();
//                 ending_time = starting_time + 5;
//                 flag = false;
//             }
//         }
// //         ROS_INFO("flag %d", flag);
//     //     Eigen::Vector3d& start, const Eigen::Vector3d& end, double clearance,double t_start, double t_end, double time, Eigen::Vector3d* vel, Eigen::Vector3d* acc  
//         ret = VC.compute_swing_trajectory(starting_position, ending_position, 0, starting_time, ending_time, ros::Time::now().toSec()); 
//         std::string sep = "\n----------------------------------------\n";
// //         std::cout << "starting_position: " << std::endl << starting_position << sep;
// //         std::cout << "ending_position: " << std::endl << ending_position << sep;
//         std::cout << ret << sep;
// }
