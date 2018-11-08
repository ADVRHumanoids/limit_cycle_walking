#ifndef VirtualConstraintsNode_H
#define VirtualConstraintsNode_H


#include <geometry_msgs/PoseStamped.h>
#include <XBotCore/CommandAdvr.h>
#include <tf/transform_listener.h>
#include "cartesian_interface/CartesianInterface.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"

#include <mainNode.h>

#define PI 3.141592653589793238463




class virtualConstraintsNode : mainNode {
public:
    
//     struct intial_state_robot /*TODO why not?*/
//     {
//        static geometry_msgs::Point com;
//        static geometry_msgs::Point l_sole;
//        static geometry_msgs::Point r_sole;
//     };
    
    struct robot_state   /*TODO is this bad coding practice?*/
    {
       geometry_msgs::Point com;
       geometry_msgs::Point l_sole;
       geometry_msgs::Point r_sole;
    }; 
    
    virtualConstraintsNode(int argc, char **argv, const char *node_name);
    
    void q1_callback(const std_msgs::Float64 msg_rcv); //this is called by ros
    
    void joints_state_callback(const sensor_msgs::JointState msg_rcv); //this is called by ros


    void com_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
    
    void l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
    
    void r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros

    void get_initial_pose();
    
    double get_q1();

    
//     double calc_VC_legs ();

    
    void publish_x_position_com();
    void publish_x_position_r_sole();

    
    double listen_z_distance_ankle_com();
    
    double incline();
    double step();
    
  
    
protected:
    
 
    
    ros::Subscriber _cartesian_solution_sub; 
    ros::Subscriber _com_sub;
    ros::Subscriber _l_sole_sub, _r_sole_sub;
    ros::Subscriber _q1_sub;
    
    ros::Publisher _com_pub;     
    ros::Publisher _r_sole_pub;
    
    std::vector<double> _joints_state;
    
    robot_state _initial_pose;
    
    geometry_msgs::Point _com_state;
    geometry_msgs::Point _l_sole_state, _r_sole_state;
    geometry_msgs::PoseStamped _initial_com_pose;
    double _q1_state;
    
    tf::StampedTransform _ankle_to_com_transform;
    
    int _joint_number = 10; /*ankle_pitch_angle*/
    
    

    
};

#endif
