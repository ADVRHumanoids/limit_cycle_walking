#ifndef RobotInterfaceROS_H
#define RobotInterfaceROS_H

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


#include <geometry_msgs/PoseStamped.h>
// #include <XBotCore/CommandAdvr.h>
#include <tf/transform_listener.h>
// #include "cartesian_interface/CartesianInterface.h"
#include <sensor_msgs/JointState.h>
// #include "std_msgs/Float64.h"

// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// #include <cartesian_interface/ReachPoseAction.h>

#include<robot_interface.h>

class robot_interface_ROS: public robot_interface
    {
    public:
        
        robot_interface_ROS();
        void sense();
        
    private:
        
        std::vector<double> _joints_state;
        std::vector<ros::Subscriber> _subs;
        
        tf::TransformListener l_ankle_to_com_listener, r_ankle_to_com_listener, _l_to_r_foot_listener;
        tf::StampedTransform l_ankle_to_com_transform;
        tf::StampedTransform r_ankle_to_com_transform;
        tf::StampedTransform _l_to_r_foot_transform;
   
        
        Eigen::Affine3d listen_l_ankle_to_com();
        Eigen::Affine3d listen_r_ankle_to_com();
        
        Eigen::Affine3d listen_l_to_r_foot();
        
        void joints_state_callback(const sensor_msgs::JointState msg_rcv); //this is called by ros
        void com_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        void l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        void r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        
        bool _check_1, _check_2, _check_3, _check_4;
        
//         ros::NodeHandle n;
    };
    
#endif