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

#include <XBotInterface/MatLogger.hpp>

#include <memory>


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_interface/ReachPoseAction.h>

#include <mainNode.h>

#include <robot_interface.h>
#include <robot_interface_ROS.h>

#define PI 3.141592653589793238463



class virtualConstraintsNode : mainNode {
public:
    
//     struct intial_state_robot /*TODO why not?*/
//     {
//        static geometry_msgs::Point com;
//        static geometry_msgs::Point l_sole;
//        static geometry_msgs::Point r_sole;
//     };

//         enum class State { Busy, Online };
    enum class Side : char { Left = 'L', Right = 'R'};
    
    friend std::ostream& operator<<(std::ostream& os, virtualConstraintsNode::Side s)
{
    return os << static_cast<char>(s);
};

    class data_step
    {
    public:
        
        void set_data_step(Eigen::Vector3d step_initial_pose, 
                           Eigen::Vector3d step_final_pose,
                           Eigen::Vector3d com_initial_pose,
                           Eigen::Vector3d com_final_pose,
                           double clearing,
                           double starTime, 
                           double endTime)
        {
            _step_initial_pose =  step_initial_pose;
            _step_final_pose = step_final_pose;
            _com_initial_pose = com_initial_pose;
            _com_final_pose = com_final_pose;
            _clearing = clearing;
            _starTime = starTime;
            _endTime = endTime;        
        }
        
        void log(XBot::MatLogger::Ptr logger) { logger->add("step_initial_pose", _step_initial_pose);
                                                logger->add("step_final_pose", _step_final_pose);
                                                logger->add("com_initial_pose", _com_initial_pose);
                                                logger->add("com_final_pose", _com_final_pose);
                                                logger->add("clearing", _clearing);
                                                logger->add("starTime", _starTime);
                                                logger->add("endTime", _endTime);}
        
        Eigen::Vector3d get_foot_initial_pose() {return _step_initial_pose;};
        Eigen::Vector3d get_foot_final_pose() {return _step_final_pose;};
        Eigen::Vector3d get_com_initial_pose() {return _com_initial_pose;};
        Eigen::Vector3d get_com_final_pose() {return _com_final_pose;};
        double get_clearing() {return _clearing;};
        double get_starTime() {return _starTime;};
        double get_endTime() {return _endTime;};
                              
    private:
        
        Eigen::Vector3d _step_initial_pose, _step_final_pose;
        Eigen::Vector3d _com_initial_pose, _com_final_pose;
        double _clearing;
        double _starTime, _endTime;
    };  
    
//     enum class State { Busy, Online };  /*TODO put robot state in class*/
//     virtual State getState() const = 0;
    
    
    virtualConstraintsNode(int argc, char **argv, const char *node_name);
    ~virtualConstraintsNode() {_logger->flush();};
    
    robot_interface_ROS&  get_robot() {return _current_pose_ROS;}; /*this is a reference (I can also use a pointer) because the class _current_pose_ROS is not copiable*/
    robot_interface get_initial_robot() {return _initial_pose;};
    
    double getTime();
    int straighten_up_action();
    int initial_shift_action();
    
    Eigen::Vector3d straighten_up_goal();
    
    void q1_callback(const std_msgs::Float64 msg_rcv); //this is called by ros
    


    void update_initial_pose();
    void update_current_pose_ROS();

    double get_q1();
    bool new_q1();
    
    double sense_q1();
    
    
//     double calc_VC_legs ();

    void update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update);
    
//     double incline();
//     double step();
    void calc_step(double q1,  Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step);
    
    bool impact_detected(); 
    
    void run();
    void foot_position();
    
    void update_command_step(Eigen::Vector3d initial_pose, Eigen::Vector3d final_pose, double clearing, double startTime, double endTime);
    
    void send_step(Eigen::Vector3d foot_command, Eigen::Vector3d com_command);
    void send_step_left(Eigen::Vector3d foot_command, Eigen::Vector3d com_command);
    void send_step_right(Eigen::Vector3d foot_command, Eigen::Vector3d com_command);
    
    void update_step();
    void update_step_left();
    void update_step_right();
    
    void send_point();
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
    void get_current_pose_ROS();
    
    void first_q1();
    
    
protected:
    
//     ros::NodeHandle n;
    ros::Publisher _com_pub;     
    ros::Publisher _r_sole_pub, _l_sole_pub;
    
    ros::Subscriber _q1_sub;
    
    
    robot_interface _initial_pose, _previous_initial_pose;
    robot_interface_ROS _current_pose_ROS;
    
    double _q1_cmd;
    double _q1_state;
    

    int _joint_number = 10; /*ankle_pitch_angle*/

    Eigen::Vector3d _foot_trajectory;
    
    bool  _flag = true;
    
    double _q1_step = 0;

    bool _check_received = false;
    
    data_step _step;
    
    Side _current_side = Side::Left; /*means that starting swinging leg is LEFT. Pinned leg is RIGHT.*/
    
    XBot::MatLogger::Ptr _logger;
    
    double _terrain_heigth;
    
    int _step_counter;
};





#endif















        
