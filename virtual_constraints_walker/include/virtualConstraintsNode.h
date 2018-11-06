#ifndef VirtualConstraintsNode_H
#define VirtualConstraintsNode_H


#include <geometry_msgs/PoseStamped.h>
#include <XBotCore/CommandAdvr.h>
#include <tf/transform_listener.h>
#include "cartesian_interface/CartesianInterface.h"
#include <sensor_msgs/JointState.h>

#include <mainNode.h>

#define PI 3.141592653589793238463




class virtualConstraintsNode : mainNode {
public:
    
    virtualConstraintsNode(int argc, char **argv, const char *node_name);
    

    void joints_state_callback(const sensor_msgs::JointState msg_rcv); //this is called by ros


    void com_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros

    
    double get_q1();

    
//     double calc_VC_legs ();

    
    void publish_x_position_com(double pos); 

    
    double listen_z_distance_ankle_com();

    
    void run();

protected:
    
    ros::Subscriber _cartesian_solution_sub;
    ros::Subscriber _com_sub;
    ros::Publisher _com_pub;
    
    std::vector<double> _joints_state;
    
    geometry_msgs::Point _com_state;
    
    tf::StampedTransform _ankle_to_com_transform;
    
    int _joint_number = 10; /*ankle_pitch_angle*/
    
};

#endif
