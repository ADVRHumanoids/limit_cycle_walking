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

#include <robot_interface.h>
#include <robot_interface_ROS.h>

#define PI 3.141592653589793238463



class virtualConstraintsNode {
public:

    class data_step
    {
    public:
        
        void set_data_step(Eigen::Vector3d step_initial_pose, 
                           Eigen::Vector3d step_final_pose,
                           Eigen::Vector3d com_initial_pose,
                           Eigen::Vector3d com_final_pose,
                           double step_clearing,
                           double com_clearing,
                           double starTime, 
                           double endTime)
        {
            _step_initial_pose =  step_initial_pose;
            _step_final_pose = step_final_pose;
            _com_initial_pose = com_initial_pose;
            _com_final_pose = com_final_pose;
            _step_clearing = step_clearing;
            _com_clearing = com_clearing;
            _starTime = starTime;
            _endTime = endTime;        
        }
        
        void log(XBot::MatLogger::Ptr logger) { logger->add("step_initial_pose", _step_initial_pose);
                                                logger->add("step_final_pose", _step_final_pose);
                                                logger->add("com_initial_pose", _com_initial_pose);
                                                logger->add("com_final_pose", _com_final_pose);
                                                logger->add("step_clearing", _step_clearing);
                                                logger->add("com_clearing", _com_clearing);
                                                logger->add("starTime", _starTime);
                                                logger->add("endTime", _endTime);}
        
        Eigen::Vector3d get_foot_initial_pose() {return _step_initial_pose;};
        Eigen::Vector3d get_foot_final_pose() {return _step_final_pose;};
        Eigen::Vector3d get_com_initial_pose() {return _com_initial_pose;};
        Eigen::Vector3d get_com_final_pose() {return _com_final_pose;};
        double get_step_clearing() {return _step_clearing;};
        double get_com_clearing() {return _com_clearing;};
        double get_starTime() {return _starTime;};
        double get_endTime() {return _endTime;};
        
        void set_foot_initial_pose(Eigen::Vector3d step_initial_pose) {_step_initial_pose =  step_initial_pose;};
        void set_foot_final_pose(Eigen::Vector3d step_final_pose) {_step_final_pose = step_final_pose;};
        void set_com_initial_pose(Eigen::Vector3d com_initial_pose) {_com_initial_pose = com_initial_pose;};
        void set_com_final_pose(Eigen::Vector3d com_final_pose) {_com_final_pose = com_final_pose;};
        void set_step_clearing(double step_clearing) {_com_clearing = step_clearing;};
        void set_com_clearing(double com_clearing) {_com_clearing = com_clearing;};
        void set_starTime(double starTime) {_starTime = starTime;};
        void set_endTime(double endTime) {_endTime = endTime;};
    private:
        
        Eigen::Vector3d _step_initial_pose, _step_final_pose;
        Eigen::Vector3d _com_initial_pose, _com_final_pose;
        double _step_clearing, _com_clearing;
        double _starTime, _endTime;
    };  
    
    
    virtualConstraintsNode();
    ~virtualConstraintsNode() {_logger->flush();};
    
    robot_interface_ROS&  get_robot() {return _current_pose_ROS;}; /*this is a reference (I can also use a pointer) because the class _current_pose_ROS is not copiable*/
    robot_interface get_initial_robot() {return _initial_pose;};
    int get_max_steps() {return _initial_param.get_max_steps();};
    bool get_param_ros();
    double getTime();
    int straighten_up_action();
    int initial_shift_action();
    
    Eigen::Vector3d straighten_up_goal();
    
    void q1_callback(const std_msgs::Float64 msg_rcv); //this is called by ros
    
    double get_q1();
    bool new_q1();
    
    double sense_q1();
    double sense_qlat();
    
    Eigen::MatrixXd get_supportPolygon();
    
    void update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update);
    
    double lateral_com();
    
    void calc_step(double q1,  Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step);
    
    bool impact_detected(); 
    
    void run();
    
    double lat_oscillator_com(double starting_time, double phase);
    
    void send(std::string type, Eigen::Vector3d command);
    void send_step(Eigen::Vector3d foot_command, Eigen::Vector3d com_command);
    
    void update_step();
//~~~~~~~~~~~~~~~~~~~~~~~~ compute trajectory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Eigen::Vector3d compute_swing_trajectory(const Eigen::Vector3d& start, 
                                                    const Eigen::Vector3d& end, 
                                                    double clearance,
                                                    double t_start, 
                                                    double t_end,
                                                    double time,
                                                    std::string side,
                                                    Eigen::Vector3d * vel = nullptr,
                                                    Eigen::Vector3d * acc = nullptr
                                                    );

double compute_swing_trajectory_normalized_plane(double dx0, double ddx0, 
                                                                         double dxf, double ddxf, 
                                                                         double tau, 
                                                                         double* __dx, double* __ddx);

    static double compute_swing_trajectory_normalized_clearing(double final_height, 
                                                               double tau,
                                                               double* dx = 0, 
                                                               double* ddx = 0);

    static double time_warp(double tau, double beta);
    
    void FifthOrderPlanning(double x0, double dx0, double ddx0,
                                                double xf, double dxf, double ddxf,
                                                double start_time, double end_time, 
                                                double time, double& x, double& dx, double& ddx
                                                );
    
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    double getPt( double n1, double n2, float perc);
    void getBezierCurve(float tau);


    void first_q1();
    int get_n_step() {return _step_counter;};
    
protected:
    
//     ros::NodeHandle n;
    bool _start_walk = 0;
    bool _end_walk = 0;
    
    double _starting_time;
    double _reducer;
    int _numerator = 0;
    ros::Publisher _com_pub;     

    std::map<robot_interface::Side, ros::Publisher> _sole_pubs;
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
    
    robot_interface_ROS::Side _current_side = robot_interface::Side::Double; 
    
    XBot::MatLogger::Ptr _logger;
    
    double _terrain_heigth;
    
    int _step_counter;
//     ros::NodeHandle n;
    
    class param
    {
    public:
        
        double get_crouch() {return _crouch;};
        double get_clearance_step() {return _clearance_step;};
        double get_duration_step() {return _duration_step;};
        robot_interface::Side get_first_step_side() {return _first_step_side;};
        int get_max_steps() {return _max_steps;};
        
        void set_crouch(double crouch) {_crouch = crouch;};
        void set_clearance_step(double clearance_step) {_clearance_step = clearance_step;};
        void set_duration_step(double duration_step) {_duration_step = duration_step;};
        void set_first_step_side(robot_interface::Side first_step_side) {_first_step_side = first_step_side;};
        void set_max_steps(int max_steps) {_max_steps = max_steps;};
        
    private:
        
        double _crouch, _clearance_step, _duration_step;
        robot_interface::Side _first_step_side;
        int _max_steps;

    } _initial_param;
    
};



#endif















        
