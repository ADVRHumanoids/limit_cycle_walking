
#ifndef VirtualConstraintsNode_H
#define VirtualConstraintsNode_H

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <XBotCore/CommandAdvr.h>
#include <tf/transform_listener.h>
#include "cartesian_interface/CartesianInterface.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_interface/ReachPoseAction.h>

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
    
    struct robot_position
    {
       Eigen::Vector3d com;
       Eigen::Vector3d l_sole;
       Eigen::Vector3d r_sole;
       Eigen::Vector3d ankle_to_com;
       
    };
    
//     enum class State { Busy, Online };  /*TODO put robot state in class*/
//     virtual State getState() const = 0;
    
    virtualConstraintsNode(int argc, char **argv, const char *node_name);
    
    int straighten_up_action();
    
    Eigen::Vector3d straighten_up_goal();
    
    void q1_callback(const std_msgs::Float64 msg_rcv); //this is called by ros
    
    void joints_state_callback(const sensor_msgs::JointState msg_rcv); //this is called by ros
    void com_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
    void l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
    void r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros

    void get_initial_pose();
    void get_current_pose();

    double get_q1();
    double calc_q1();
    
//     double calc_VC_legs ();

    void update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update);
    
    Eigen::Vector3d listen_distance_ankle_to_com();
    Eigen::Vector3d listen_distance_l_to_r_foot();
    
//     double incline();
//     double step();
    void calc_step(double q1,  Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step);
    
    void left_move();
    void right_move();
    
    void left_step_move();
    void right_step_move();
    
    void impact_detected(); 
    
//~~~~~~~~~~~~~~~~~~~~~~~~ compute trajectory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static Eigen::Vector3d compute_swing_trajectory(const Eigen::Vector3d& start, 
                                                const Eigen::Vector3d& end, 
                                                double clearance,
                                                double t_start, 
                                                double t_end,
                                                double time,
                                                Eigen::Vector3d * vel = nullptr,
                                                Eigen::Vector3d * acc = nullptr
                                                );

static double compute_swing_trajectory_normalized_xy(double tau, double* dx = 0, double* ddx = 0);
static double compute_swing_trajectory_normalized_z(double final_height, 
                                                    double tau, 
                                                    double* dx = 0, 
                                                    double* ddx = 0);
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

static double time_warp(double tau, double beta);
    
protected:
    
 
    
    ros::Subscriber _cartesian_solution_sub; 
    ros::Subscriber _com_sub;
    ros::Subscriber _r_sole_sub, _l_sole_sub;
    ros::Subscriber _q1_sub;
    
    ros::Publisher _com_pub;     
    ros::Publisher _r_sole_pub, _l_sole_pub;
    
    
    std::vector<double> _joints_state;
    
    robot_position _initial_pose, _current_pose;
    
    Eigen::Vector3d _com_state;
    Eigen::Vector3d _l_sole_state, _r_sole_state;
    geometry_msgs::PoseStamped _initial_com_pose;
    double _q1_state;
    
    tf::StampedTransform _ankle_to_com_transform;
    tf::StampedTransform _l_to_r_foot_transform;
    int _joint_number = 10; /*ankle_pitch_angle*/

    tf::TransformListener _ankle_to_com_listener, _l_to_r_foot_listener;
    
    
    
    
};

#endif















        
