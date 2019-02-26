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
//     ros::Rate loop_rate(100);
//     robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
//     
//     double start_idle_time;
//     
//     ros::NodeHandle n_cmd;
//     ros::Publisher _q1_pub;
//     
// 
//     _q1_pub = n_cmd.advertise<std_msgs::Float64>("/q1", 10);
//     
// //     std::vector<Eigen::MatrixXd> supportPolygon(VC.get_max_steps());
//     
// // --------initialize robot so that q1 is exactly 0---------
//     if (ros::ok())
//     {
//     robot.sense(); /*inside there is ros::spinOnce*/
//     VC.straighten_up_action();
//     VC.sense_q1();
//     }
// // ---------------add idle time ----------------------------
//     start_idle_time = ros::Time::now().toSec();
//     while (ros::ok() && ros::Time::now().toSec() <= start_idle_time +4) {}
//     
// //----------------------------------------------------------  
//     double q1_fake = VC.sense_q1();
//     double q1_temp = q1_fake;
// //     std::cout << VC._q1_fake << std::endl;
//     
//     double starting_time = ros::Time::now().toSec();
// //     double ending_time = ros::Time::now().toSec();
//     double last_q1_fake = 0;
//     while (ros::ok() && VC.get_n_step() < VC.get_max_steps())
//     {
// 
// //         VC.get_robot().sense();
//         
//         
//         q1_temp = 0.1*(ros::Time::now().toSec() - starting_time);
//         q1_fake = q1_temp - last_q1_fake;
//         
//         
//         std_msgs::Float64 cmd_q1;
//         cmd_q1.data = q1_fake;
//         _q1_pub.publish(cmd_q1);
//         
//         
//         VC.get_robot().sense();  //boh shouldn't be here
//         
//         
//         VC.fakeCOM();
//         
//         VC.get_robot().sense();
//         
//         VC.run();
//         
//         
//         if (VC.impact())
//         {
//             last_q1_fake = q1_temp;
//         }
//         
//         
//         
//         loop_rate.sleep();
// //         }
//     }
//     
// // ---------------add idle time ----------------------------
//     start_idle_time = ros::Time::now().toSec();
//     while (ros::ok() && ros::Time::now().toSec() <= start_idle_time +4) {}
//     
// //----------------------------------------------------------  
// 
// 
// }    


//// current one ////////////

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
    if (ros::ok())
    {
    robot.sense(); /*inside there is ros::spinOnce*/
    VC.straighten_up_action();
    VC.sense_q1();
    } 
    while (ros::ok())
    {
        VC.get_robot().sense();
        VC.exe(ros::Time::now().toSec());
        loop_rate.sleep();
    }
}   

//////////////////////////////////////////////////////

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "virtual_constraints");
//     XBot::MatLogger::Ptr _logger;
//     _logger = XBot::MatLogger::getLogger("/tmp/porcodio");
//     virtualConstraintsNode VC;
//     
//     /*synchronize this node to the cartesian interface*/
//     ros::service::waitForService("cartesian/get_task_list"); 
//     ros::Rate loop_rate(100); //TODO set it from the robot
//     robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/
//     
//     double first_stance_step = 0.1;
//     double _t_before_first_step = 0.5;
//     double double_stance = 0;
//     int max_steps = 10;
//     double dt = 0.01;
//     
//     Eigen::VectorXd _zmp_t;
//     Eigen::VectorXd _zmp_y;
//     
//     VC.generate_zmp_new(first_stance_step, _t_before_first_step, double_stance, max_steps, dt, _zmp_t, _zmp_y); //TODO once filled, I shouldn't be able to modify them);     
//     
//     _logger->add("zmp_y", _zmp_y);
//     
//     _logger->flush();
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


//////////////// trial for  polynomial ////////////////////////

// int main(int argc, char **argv)
// {
// 
//     ros::init(argc, argv, "virtual_constraints");
//     virtualConstraintsNode VC;
//     ros::Rate loop_rate(100);
//     
//     XBot::MatLogger::Ptr _logger;
//         _logger = XBot::MatLogger::getLogger("/tmp/porcodio");
//     Eigen::Vector3d starting_position, ending_position, ret;
//     Eigen::Vector3d new_starting_position, new_ending_position;
//     double starting_time, ending_time;
//     starting_position <<  -0.0776479, 0.000462967,       -0.12;
//     ending_position <<  -0.0391377, 0.000462967,       -0.12;
//     
// 
//  
//  
//  
//     new_ending_position = ending_position;
//     double clearance = 0.5;
//     starting_time = ros::Time::now().toSec();
//     ending_time = ros::Time::now().toSec() + 10;
//     bool flag = true;
// 
//         while (ros::ok() && ros::Time::now().toSec() <= ending_time +1)
//     {  
// //         if (ros::Time::now().toSec() > ending_time - 5)
// //         {
// //             std::cout << "commanded new end position" << std::endl;
// //             new_ending_position << 8, 8, 8;
// //         }
// //         
// //         if (new_ending_position != ending_position)
// //         {
// //             starting_position = ret;
// //              std::cout << "changed new ending position from : " << ending_position.transpose() << " to: " << new_ending_position.transpose() <<std::endl;
// //             ending_position = new_ending_position;
// //             starting_time = ros::Time::now().toSec();
// //             std::cout << "starting_time: " << starting_time <<std::endl;
// //             std::cout << "ending_time: " << ending_time <<std::endl;
// //            
// //             std::cout << "changed new starting position: " << starting_position.transpose() << std::endl;
// //             
// //         }
//         ret = VC.compute_swing_trajectory(starting_position, ending_position, 0, starting_time, ending_time, ros::Time::now().toSec(), "xy");
//         _logger->add("traj", ret);
//         loop_rate.sleep();
//     }
//     _logger->flush();
// }