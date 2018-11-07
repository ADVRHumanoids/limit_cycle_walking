#include <mainNode.h>
#include <virtualConstraintsNode.h>

int main(int argc, char **argv)
{
 
    
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
    
    while (ros::ok()) {
        
    ros::spinOnce();
//     VC.get_q1();
//     VC.listen_z_distance_ankle_com();
    
        VC.publish_x_position_com();
        VC.publish_x_position_r_sole();
//         VC.step();
    
//     VC.incline();
//     loop_rate.sleep();
    }


}





    
    
    
//     ros::spinOnce();
//     loop_rate.sleep();
    
  
//     ros::NodeHandle n;
//       
//     tf::TransformListener listener;
//     
//     listener.waitForTransform("ci/com", "ci/l_ankle", ros::Time::now(), ros::Duration(3.0));
//     
//     
//     
//     
//     
//     ros::Publisher CoM_talker = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 1000);
//     
//     
//     ros::Rate loop_rate(10);    
//       
//       
// int count = 0;
// 
//   while (ros::ok())
//   { 
//     double leg_length;
//     geometry_msgs::PoseStamped msg_cmd;
//     tf::StampedTransform transform;
// 
//     listener.lookupTransform("ci/com", "ci/l_ankle", ros::Time(0), transform);
// 
// 
//     leg_length = transform.getOrigin().z(); /*get distance from com to l_ankle*/
//    
//    
//     
//     
//     int pos = 2 * leg_length * sin(q1);
// 
//     //     pos.push_back(angle);
// 
//     msg_cmd.pose.position.x = pos;
// 
// 
//     //     ROS_INFO("%d", msg.pose.position.x);
// 
//     CoM_talker.publish(msg_cmd);
// 
//     
//     ros::spinOnce();
//     loop_rate.sleep();
//     ++count;