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

#include <memory>


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
    
    class robot_position
    {
    public:
        
        typedef std::shared_ptr<robot_position> Ptr;
        
        robot_position();
        robot_position(ros::NodeHandle n);
        
        Eigen::Vector3d get_com() {return _com_state;};
        Eigen::Vector3d get_l_sole() {return _l_sole_state;};
        Eigen::Vector3d get_r_sole() {return _r_sole_state;};
        Eigen::Vector3d get_distance_ankle_to_com() {return listen_distance_ankle_to_com();};
        Eigen::Vector3d get_distance_l_to_r_foot() {return listen_distance_l_to_r_foot();};
        
       
        
    private:
        
        std::vector<double> _joints_state;
        ros::Subscriber _cartesian_solution_sub; 
        ros::Subscriber _com_sub;
        ros::Subscriber _r_sole_sub, _l_sole_sub;


        tf::TransformListener _ankle_to_com_listener, _l_to_r_foot_listener;
        tf::StampedTransform _ankle_to_com_transform;
        tf::StampedTransform _l_to_r_foot_transform;
        
        Eigen::Vector3d _com_state;
        Eigen::Vector3d _l_sole_state, _r_sole_state;
    

        
        Eigen::Vector3d listen_distance_ankle_to_com();
        Eigen::Vector3d listen_distance_l_to_r_foot();
        void joints_state_callback(const sensor_msgs::JointState msg_rcv); //this is called by ros
        void com_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        void l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
        void r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv); //this is called by ros
    };
    
    
//     enum class State { Busy, Online };  /*TODO put robot state in class*/
//     virtual State getState() const = 0;
    
    
    virtualConstraintsNode(int argc, char **argv, const char *node_name);
    
    
    Eigen::Vector3d get_com() 
    {
        std::cout << _current_pose->get_com(); 
        return _current_pose->get_com();
    };
    Eigen::Vector3d get_l_sole() {return _current_pose->get_l_sole();};
    Eigen::Vector3d get_r_sole() {return _current_pose->get_r_sole();};
    Eigen::Vector3d get_distance_ankle_to_com() {return _current_pose->get_distance_ankle_to_com();};
    Eigen::Vector3d get_distance_l_to_r_foot() {return _current_pose->get_distance_l_to_r_foot();};
    
        
    double getTime();
    int straighten_up_action();
    
    Eigen::Vector3d straighten_up_goal();
    
    void q1_callback(const std_msgs::Float64 msg_rcv); //this is called by ros
    


    void update_initial_pose();
    void update_current_pose();

    double get_q1();
    double calc_q1();
    
//     double calc_VC_legs ();

    void update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update);
    
//     double incline();
//     double step();
    void calc_step(double q1,  Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step);
    
    void impact_detected(); 
    
    void calc_trajectory();
    void foot_position();
    
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
    void get_current_pose();
    
protected:
    

    ros::Publisher _com_pub;     
    ros::Publisher _r_sole_pub, _l_sole_pub;
    
    ros::Subscriber _q1_sub;
    
    
    robot_position::Ptr _current_pose; /*_initial_pose, */
    
    
    geometry_msgs::PoseStamped _initial_com_pose;
    double _q1_state;
    

    int _joint_number = 10; /*ankle_pitch_angle*/

    
    
    bool _flag = true;
    
    double _q1_step = 0;

//     class step_state 
//     {
//     public:
//         step_state(robot_position _current_pose);
//         Eigen::Vector3d get_state();
//         Eigen::Vector3d update_state();
//         
//     private:
//         Eigen::Vector3d _starting_position;
//         Eigen::Vector3d _ending_position;
//         Eigen::Vector3d _previous_ending_position;
//         double _startTime, _endTime;
//         double _clearing;
//     };
};

#endif















        
