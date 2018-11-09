#include <mainNode.h>
#include <virtualConstraintsNode.h>

int main(int argc, char **argv)
{
 
    
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
    
    while (ros::ok()) {
        
    ros::spinOnce();
//     VC.get_q1();
//     VC.listen_z_distance_ankle_com();
    
//         VC.publish_x_position_com();
//         VC.publish_x_position_r_sole();
//             VC.left_move();
//             if 
            VC.right_move();
//         VC.get_q1();

    
//     VC.incline();
//     loop_rate.sleep();
    }


}