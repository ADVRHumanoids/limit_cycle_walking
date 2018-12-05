#include <mainNode.h>
#include <virtualConstraintsNode.h>

int main(int argc, char **argv)
{
    
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
//     robot_interface_ROS robot;
    robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
// --------initialize robot so that q1 is exactly 0---------
    if (ros::ok())
    {
    robot.sense(); /*inside there is ros::spinOnce*/
    VC.straighten_up_action();  
//     VC.initial_shift_action();
    VC.sense_q1();
    }
//---------------------------------------------------------
//     while (ros::ok())
    while (ros::ok() && VC.get_n_step() < VC.get_max_steps())
    {
        VC.get_robot().sense();
        VC.run();
    } 
    
    
// Eigen::Vector3d starting_position, ending_position, ret;
// std::string sep = "\n----------------------------------------\n";
// starting_position << 0, 0, 0;
// ending_position << 1, 1, 1;
// double clearance = 0.2;
// double starting_time, ending_time;
// starting_time = ros::Time::now().toSec();
// ending_time = ros::Time::now().toSec() + 4;
// bool flag = true;
// 
//         while (ros::ok() && ros::Time::now().toSec() <= ending_time +1)
//  {
//             ret = VC.compute_swing_trajectory(starting_position, ending_position, clearance, starting_time, ending_time, ros::Time::now().toSec(), "yz"); 
//             std::cout << ret.transpose() << sep;
// 
// }

}