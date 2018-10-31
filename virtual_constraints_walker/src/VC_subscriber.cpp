#include "ros/ros.h"
#include "cartesian_interface/CartesianInterface.h"
#include <sensor_msgs/JointState.h>


void chatterCallback(const sensor_msgs::JointState msg)
{
    Eigen::VectorXd joint_states(19);
    joint_states.setZero();
    std::vector<double> position_vector(20);
    double q1, q2 = 0;
    position_vector = msg.position;
    q2 = position_vector[10];
    
    q1 = msg.position[10];
    ROS_INFO("var is %f", q2);
//       int i = 0;
//     for(std::vector<double>::const_iterator it = msg.position.begin(); it != msg.position.end(); ++it)
//     {
//         joint_states[i] = *it;
//         i++;
//          ROS_INFO_STREAM("Joints state:\n" << joint_states); 
//         
//     }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "VC_subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/cartesian/solution", 1000, chatterCallback);

  ros::spin();

  return 0;
}