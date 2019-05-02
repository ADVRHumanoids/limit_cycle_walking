#ifndef RobotInterfaceROS_H
#define RobotInterfaceROS_H

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf2/convert.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/JointState.h>

#include<robot_interface.h>

class robot_interface_ROS: public robot_interface
    {
    public:
        
        robot_interface_ROS();
        void sense();
        
    private:
        
        /*TODO lump together StampedTransform and functions listen and transform*/
        std::vector<double> _joints_state;
        std::vector<ros::Subscriber> _subs;
        
        tf::TransformListener l_com_to_ankle_listener, r_com_to_ankle_listener, _l_to_r_foot_listener;
        tf::StampedTransform l_com_to_ankle_transform;
        tf::StampedTransform r_com_to_ankle_transform;
        tf::StampedTransform _l_to_r_foot_transform;
   
        
        Eigen::Affine3d listen_l_ankle_to_com();
        Eigen::Affine3d listen_r_ankle_to_com();
        
        Eigen::Affine3d listen_l_to_r_foot();
        
        void joints_state_callback(const sensor_msgs::JointState msg_rcv); //this is called by ros
        void com_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        void l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        void r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        void l_sole_ft_callback(const geometry_msgs::WrenchStamped msg_rcv);
        void r_sole_ft_callback(const geometry_msgs::WrenchStamped msg_rcv);
//         void zmp_callback(const geometry_msgs::PoseStamped msg_rcv);
        
        bool _check_1, _check_2, _check_3, _check_4, _check_5, _check_6, _check_7;
        
//         ros::NodeHandle n;
    };
    
#endif