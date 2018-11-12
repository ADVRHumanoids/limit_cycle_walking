#include <mainNode.h>
#include <virtualConstraintsNode.h>

int main(int argc, char **argv)
{
 
    
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
    tf::Vector3 distance_foots;
    while (ros::ok()) {
        
    ros::spinOnce();
//     VC.get_q1();
//     VC.listen_z_distance_ankle_com();
    
//         VC.publish_x_position_com();
//         VC.publish_x_position_r_sole();
//             VC.left_move();
//         VC.listen_distance_ankle_to_com();
//     VC.listen_distance_l_to_r_foot();
       distance_foots = VC.listen_distance_l_to_r_foot();
            VC.right_move();
//             if (distance_foots.x() >= 0.2)
//             {
//                 VC.left_move();
//             }
// //         VC.get_q1();
//         VC.joints_state_callback();

    
//     VC.incline();
//     loop_rate.sleep();
    }


}