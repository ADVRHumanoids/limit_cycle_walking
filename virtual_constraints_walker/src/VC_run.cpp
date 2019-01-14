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
//     VC._q1_fake = VC.sense_q1();
//     
//     std::cout << VC._q1_fake << std::endl;
//     
//     double starting_time = ros::Time::now().toSec();
// //     double ending_time = ros::Time::now().toSec();
//     
//     while (ros::ok() && VC.get_n_step() < VC.get_max_steps())
//     {
//         
//         VC._q1_fake = VC._reset_condition + 0.1*(ros::Time::now().toSec() - starting_time);
//         _q1_pub.publish(VC._q1_fake);
//         
//         
//         VC.get_robot().sense();
//         VC.run();
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




int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_constraints");
    virtualConstraintsNode VC;
    ros::Rate loop_rate(100);
    
    Eigen::Vector3d x; 
    Eigen::Vector3d y;
    
    double dt = 1/100;
    
    x << 0, 0.5, 2;
    y << -1, 0, 3;
    
    std::cout << x.transpose() << std::endl;
    std::cout << y.transpose() << std::endl;
    
    int n = 3;
    int N = 0;
    
    for(int i=0; i<n-1; i++)
        {
            std::cout << "saccapala: " << x.coeff(i+1)-x.coeff(i) << std::endl;
            std::cout << "dt: " << dt << std::endl;
            N= N + (x.coeff(i+1)-x.coeff(i))/dt;
            std::cout << "N: " << N << std::endl;
        }
        
    std::cout << "end N: " << N << std::endl;
    
    Eigen::VectorXd X(N);
    Eigen::VectorXd Y(N);
//     
//     VC.lSpline(x, y, dt, N, X, Y);

   

    
}


//////////////// trial for bezier ///////////////////////////
// int main(int argc, char **argv)
// {
// 
//     ros::init(argc, argv, "virtual_constraints");
//     virtualConstraintsNode VC;
//     ros::Rate loop_rate(100);
//     double x1 = 0.4; 
//     double x2 = 0.2; 
//     double x3 = -0.8;
//     double x4 = 0.1;
//     double x5 = -0.3;
//     double x6 = 0.8;
//     

    
//     Eigen::VectorXd vect(6);
//     vect << x1, x2, x3;
//     vect << x1, x2, x3, x4;
//     vect << x1, x2, x3, x4, x5;
//     vect << x1, x2, x3, x4, x5, x6;
       
//     double tau = 0;
//     double t_min = ros::Time::now().toSec(); 
//     double t_max = ros::Time::now().toSec()+ 1;

    
//     while (ros::Time::now().toSec() <=  t_max)
//     {
//         tau = (ros::Time::now().toSec() - t_min) / (t_max - t_min);
//         VC.getBezierCurve(vect, tau);
//         
//         loop_rate.sleep();
//     }
//     Eigen::Vector3d starting_position, ending_position, ret;  
//     double starting_time, ending_time;
//     starting_position << 0, 0, 0;
//     ending_position << 1, 1, 1;
//     double clearance = 0.5;
//     starting_time = ros::Time::now().toSec();
//     ending_time = ros::Time::now().toSec() + 4;
//     bool flag = true;
// 
//         while (ros::ok() && ros::Time::now().toSec() <= ending_time +1)
//     {  
//         ret = VC.compute_swing_trajectory(starting_position, ending_position, clearance, starting_time, ending_time, ros::Time::now().toSec(), "xy");
//         loop_rate.sleep();
//     }
// }