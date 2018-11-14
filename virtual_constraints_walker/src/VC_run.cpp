#include <mainNode.h>
#include <virtualConstraintsNode.h>

int main(int argc, char **argv)
{
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
//     tf::Vector3 distance_foots;
    
//--------initialize robot so that q1 is exactly 0---------
    if (ros::ok())
    {
    ros::spinOnce();
    VC.straighten_up_action();

    }
//---------------------------------------------------------


  
//  
    while (ros::ok()) {
// //         ROS_INFO("entered");
        ros::spinOnce();
//         VC.get_current_pose();
//         VC.calc_q1();
        VC.get_current_pose();
//         
//         VC.right_step_move();


    
// //     VC.incline();
//     loop_rate.sleep();
    }


}




// Eigen::Vector3d ret;
// Eigen::Vector3d vel;
// Eigen::Vector3d acc;
// 
// Eigen::Vector3d start_point(0,0,0);
// Eigen::Vector3d end_point(1,0,0);
// 
// for (int t = 0; t<=10; t++)
// {
// //     Eigen::Vector3d& start, const Eigen::Vector3d& end, double clearance,double t_start, double t_end, double time, Eigen::Vector3d* vel, Eigen::Vector3d* acc  
//     ret = VC.compute_swing_trajectory(start_point, end_point, 1, 0, 5, t); 
//     std::string sep = "\n----------------------------------------\n";
// 
// std::cout << ret << sep;
// };