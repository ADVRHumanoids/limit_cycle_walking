#include <virtualConstraintsNode.h>


////// current one ////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_constraints");
    
    
    virtualConstraintsNode VC;
    
    /*synchronize this node to the cartesian interface*/
    ros::service::waitForService("cartesian/get_task_list"); 
    ros::Rate loop_rate(100); //TODO set it from the robot
    robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
    
    
//     std::vector<Eigen::MatrixXd> supportPolygon(VC.get_max_steps());
    
// --------initialize robot so that q1 is exactly 0---------
//     if (ros::ok())
//     {
//     robot.sense(); /*inside there is ros::spinOnce*/
//     VC.straighten_up_action();
//     VC.sense_q1();
//     }
    
    while (ros::ok())
    {
        VC.get_robot().sense();
        VC.exe(ros::Time::now().toSec());
        loop_rate.sleep();
    }
}   






// /////////////////////trial for continuous zmp////////////////////////////////////
// int main(int argc, char **argv)
// {
// 
//     ros::init(argc, argv, "virtual_constraints");
//  virtualConstraintsNode VC;
//     
//     /*synchronize this node to the cartesian interface*/
//     ros::service::waitForService("cartesian/get_task_list"); 
//     ros::Rate loop_rate(100); //TODO set it from the robot
//     robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
// 
//     Eigen::VectorXd spatial_zmp_y;
//     robot.sense(); /*inside there is ros::spinOnce*/
//     spatial_zmp_y = VC.initialize_spatial_zmp();
//     
//     VC.zmp_x_offline(100);
//     
//         
// //         loop_rate.sleep();
//     
// }


























/////////////////////trial for traj////////////////////////////////////
// int main(int argc, char **argv)
// {
// 
//     ros::init(argc, argv, "virtual_constraints");
//     virtualConstraintsNode VC;
//     ros::Rate loop_rate(100);
// 
// 
//     double starting_time, ending_time;
// 
//     starting_time = ros::Time::now().toSec();
//     ending_time = ros::Time::now().toSec() + 5;
//     
//     VC.initializeMpc();
// //     VC.zmp_traj(0, 4, times, y);
//     
//     while (ros::ok() && ros::Time::now().toSec() <= ending_time)
// //         for (int i =1; i<2000; i++)
//     {  
//         Eigen::VectorXd com_y = VC.lateral_com();
//         loop_rate.sleep();
//     }
//     
// }

//////////////// trial for bezier ///////////////////////////
// int main(int argc, char **argv)
// {
// 
//     ros::init(argc, argv, "virtual_constraints");
//     virtualConstraintsNode VC;
//     ros::Rate loop_rate(100);
//     double x1 = 0;
//     double x2 = 0.8; 
//     double x3 = 0;
// //     double x4 = 0.1;
// //     double x5 = -0.3;
// //     double x6 = 0.8;
//     
//     double t1 = 0; 
//     double t2 = 0.2; 
//     double t3 = 1;
// //     double t4 = 0.7;
// //     double t5 = 0.8;
// //     double t6 = 1;
//     
//     Eigen::VectorXd vect(3);
//     vect << x1, x2, x3;
// //     vect << x1, x2, x3, x4;
// //     vect << x1, x2, x3, x4, x5;
// //     vect << x1, x2, x3, x4, x5, x6;
//        
//     Eigen::VectorXd vect_t(3);
//     vect_t << t1, t2, t3;
// //     vect_t << t1, t2, t3, t4;
// //     vect_t << t1, t2, t3, t4, t5;
// //     vect_t << t1, t2, t3, z4, t5, t6;
//     
//     double tau = 0;
//     double t_min = ros::Time::now().toSec(); 
//     double t_max = ros::Time::now().toSec()+ 1;
// 
//     
//     while (ros::Time::now().toSec() <=  t_max)
//     {
//         tau = (ros::Time::now().toSec() - t_min) / (t_max - t_min);
//         VC.getBezierCurve(vect, vect_t, tau);
//         
//         loop_rate.sleep();
//     }
// //     Eigen::Vector3d starting_position, ending_position, ret;  
// //     double starting_time, ending_time;
// //     starting_position << 0, 0, 0;
// //     ending_position << 1, 1, 1;
// //     double clearance = 0.5;
// //     starting_time = ros::Time::now().toSec();
// //     ending_time = ros::Time::now().toSec() + 4;
// //     bool flag = true;
// // 
// //         while (ros::ok() && ros::Time::now().toSec() <= ending_time +1)
// //     {  
// //         ret = VC.compute_swing_trajectory(starting_position, ending_position, clearance, starting_time, ending_time, ros::Time::now().toSec(), "xy");
// //         loop_rate.sleep();
// //     }
// }












////////////// trial for trajectory with q ///////////////////////////
// int main(int argc, char **argv)
// {
//     
//     
//     ros::init(argc, argv, "virtual_constraints");
//     virtualConstraintsNode VC;
//     
//     XBot::MatLogger::Ptr _logger;
//     _logger = XBot::MatLogger::getLogger("/tmp/test");
//         
//     ros::Rate loop_rate(100);
//     double x1 = 0;
//     double x2 = 0.8; 
//     double x3 = 0;
//     
//     double tau = 0;
//     double t_min = ros::Time::now().toSec(); 
//     double t_max = ros::Time::now().toSec()+ 1;
// 
// 
//     Eigen::Vector3d starting_position, ending_position, ret;  
//     double starting_time, ending_time;
//     starting_position << 0, 0, 0;
//     ending_position << 1, 1, 1;
// //     ending_position << 1, 1, 1;
//     double clearance = 0;
//     starting_time = ros::Time::now().toSec();
//     ending_time = ros::Time::now().toSec() + 10;
//     bool flag = true;
//     double t = starting_time;
//     double dt = 0.01;
//         while (ros::ok() && ros::Time::now().toSec() <= ending_time +20)
//     {  
//         if (ros::Time::now().toSec() >= starting_time && ros::Time::now().toSec() <= ending_time-5)
//         {
//             t = t + dt;
//         }
//         if (ros::Time::now().toSec() >= starting_time+7 && ros::Time::now().toSec() <= ending_time+2)
//         {
//             t = t - dt;
//         }
//         
//         if (ros::Time::now().toSec() >= ending_time + 3)
//         {
//             t = t + dt;
//         }
//         ret = VC.compute_swing_trajectory(starting_position, ending_position, clearance, starting_time, ending_time, t);
//         _logger->add("ret", ret);
//                                                     
//         std::cout << ret.transpose() << std::endl;
//         loop_rate.sleep();
//     }
//     
//     _logger->flush();
// }











////////////// trial for trajectory with q ///////////////////////////
// int main(int argc, char **argv)
// {
//     
//     
//     ros::init(argc, argv, "virtual_constraints");
//     virtualConstraintsNode VC;
//     
//     XBot::MatLogger::Ptr _logger;
//     _logger = XBot::MatLogger::getLogger("/tmp/test");
//         
//     ros::Rate loop_rate(100);
//     double dt = 0.01;
//     double ending_time = ros::Time::now().toSec()  + 1;
//     double traj = 0;
//     double alpha = 0.1;
//     double h = 1;
//     double q = 0;
//     double x = M_PI/4;     
//     Eigen::Matrix2d R;
//     R << cos(x), sin(x), -sin(x), cos(x);
//     
//     while (ros::ok() && ros::Time::now().toSec() <= ending_time)
//     {  
//         q = q + alpha * dt;
//         traj = h * tan(q);
//         
//         _logger->add("q", q);
//         _logger->add("traj", traj);
//         loop_rate.sleep();
//     }
//     
//     _logger->flush();
// }