#include <mainNode.h>
#include <virtualConstraintsNode.h>

int main(int argc, char **argv)
{
    
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
//     robot_interface_ROS robot;
    robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
//     VC.get_param_ros();
//--------initialize robot so that q1 is exactly 0---------
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
}