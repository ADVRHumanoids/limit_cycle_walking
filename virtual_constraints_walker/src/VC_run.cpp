// #include <virtualConstraintsNode.h>
// 
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "virtual_constraints");
//     
//     
//     virtualConstraintsNode VC;
//     
//     /*synchronize this node to the cartesian interface*/
//     ros::service::waitForService("cartesian/get_task_list"); 
//     
//     robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
// //     std::vector<Eigen::MatrixXd> supportPolygon(VC.get_max_steps());
//     
// // --------initialize robot so that q1 is exactly 0---------
//     if (ros::ok())
//     {
//     robot.sense(); /*inside there is ros::spinOnce*/
//     VC.straighten_up_action();  
// //     VC.initial_shift_action();
//     VC.sense_q1();
//     }
// // // // ---------------------------------------------------------
// // //     while (ros::ok())
//     while (ros::ok() && VC.get_n_step() < VC.get_max_steps())
//     {
//         VC.get_robot().sense();
//         VC.run();
//     }
// }    
// 
// 
// 




#include <virtualConstraintsNode.h>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "virtual_constraints");
//     
//     
//     virtualConstraintsNode VC;
//     
//     /*synchronize this node to the cartesian interface*/
//     ros::service::waitForService("cartesian/get_task_list"); 
//      ros::Rate loop_rate(100);
//     robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
// //     std::vector<Eigen::MatrixXd> supportPolygon(VC.get_max_steps());
//     
// // --------initialize robot so that q1 is exactly 0---------
//     if (ros::ok())
//     {
//     robot.sense(); /*inside there is ros::spinOnce*/
//     VC.straighten_up_action();  
// //     VC.initial_shift_action();
//     VC.sense_q1();
//     }
// // // // ---------------------------------------------------------
// // //     while (ros::ok())
//     while (ros::ok() && VC.get_n_step() < VC.get_max_steps())
//     {
//         VC.get_robot().sense();
//         VC.run();
//         
//         loop_rate.sleep();
// //         VC.lat_oscillator_com();
//     }
// }    





int main(int argc, char **argv)
{

    ros::init(argc, argv, "virtual_constraints");
    virtualConstraintsNode VC;
    Eigen::Vector3d starting_position, ending_position, ret;
    
    

    std::string sep = "\n----------------------------------------\n";
    starting_position << 0, 0, 0;
    ending_position << 1, 1, 1;
    double clearance = 0;
    double starting_time, ending_time;
    
    double t_max = ros::Time::now().toSec()+ 10;
    double t_min = ros::Time::now().toSec();
    double tau;
    
//     while (ros::ok() && ros::Time::now().toSec() <=  t_max)
    for( float i = 0 ; i < 1 ; i += 0.01 )
    {
//         tau = (ros::Time::now().toSec() - t_max) / (t_min - t_max);
        VC.getBezierCurve(i);
    }
    
    starting_time = ros::Time::now().toSec();
    ending_time = ros::Time::now().toSec() + 4;
    bool flag = true;

        while (ros::ok() && ros::Time::now().toSec() <= ending_time +1)
    {  
        ret = VC.compute_swing_trajectory(starting_position, ending_position, clearance, starting_time, ending_time, ros::Time::now().toSec(), "yz");
    }
}