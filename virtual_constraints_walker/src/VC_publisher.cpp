#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "cartesian_interface/CartesianInterface.h"
#include <geometry_msgs/PoseStamped.h>
#include <XBotCore/CommandAdvr.h>
#include <sstream>

int main(int argc, char **argv)
{
      
    ros::init(argc, argv, "VC_publisher");   
    ros::NodeHandle n;
      
    ros::Publisher VC_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 1000);

    ros::Rate loop_rate(10);    
      
    double pos = -0.2;
    int count = 0;

  while (ros::ok())
  {

    geometry_msgs::PoseStamped msg;
    
    
//     std::vector<double> pos;
   
    
//     pos.push_back(angle);
    
    msg.pose.position.z = pos;

//     ROS_INFO("%f", msg.pose.position.z);
    
    VC_pub.publish(msg);

    ros::spinOnce();
 
    loop_rate.sleep();
    ++count;
  }


  return 0;
}