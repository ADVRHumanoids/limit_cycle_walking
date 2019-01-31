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

#include <OpenMpC/solver/UnconstrainedMpc.h>

#include <XBotInterface/MatLogger.hpp>

#include <memory>


#include <mapSteps.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_interface/ReachPoseAction.h>

#include <robot_interface.h>
#include <robot_interface_ROS.h>

#define PI 3.141592653589793238463
# define grav 9.80665 


class virtualConstraintsNode {
//     friend class mapSteps;
public:
     
    enum class Event { IMPACT, START, STOP, EMPTY };
    enum class State { INIT, IDLE, FIRSTSTEP, WALK, LASTSTEP, EXIT };
    
    
    class data_step_poly
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
    
        void log(XBot::MatLogger::Ptr logger) { logger->add("step_initial_pose_Poly", _step_initial_pose);
                                                logger->add("step_final_pose_Poly", _step_final_pose);
                                                logger->add("com_initial_pose_Poly", _com_initial_pose);
                                                logger->add("com_final_pose_Poly", _com_final_pose);
                                                logger->add("step_clearing_Poly", _step_clearing);
                                                logger->add("com_clearing_Poly", _com_clearing);
                                                logger->add("starTime_Poly", _starTime);
                                                logger->add("endTime_Poly", _endTime);}
        
        const Eigen::Vector3d get_foot_initial_pose() const {return _step_initial_pose;};
        const Eigen::Vector3d get_foot_final_pose() const {return _step_final_pose;};
        const Eigen::Vector3d get_com_initial_pose() const {return _com_initial_pose;};
        const Eigen::Vector3d get_com_final_pose() const {return _com_final_pose;};
        const double get_step_clearing() const {return _step_clearing;};
        const double get_com_clearing() const {return _com_clearing;};
        const double get_starTime() const {return _starTime;};
        const double get_endTime() const {return _endTime;};
        
        void set_foot_initial_pose(Eigen::Vector3d step_initial_pose) {_step_initial_pose =  step_initial_pose;};
        void set_foot_final_pose(Eigen::Vector3d step_final_pose) {_step_final_pose = step_final_pose;};
        void set_com_initial_pose(Eigen::Vector3d com_initial_pose) {_com_initial_pose = com_initial_pose;};
        void set_com_final_pose(Eigen::Vector3d com_final_pose) {_com_final_pose = com_final_pose;};
        void set_step_clearing(double step_clearing) {_step_clearing = step_clearing;};
        void set_com_clearing(double com_clearing) {_com_clearing = com_clearing;};
        void set_starTime(double starTime) {_starTime = starTime;};
        void set_endTime(double endTime) {_endTime = endTime;};
    private:
        
        Eigen::Vector3d _step_initial_pose, _step_final_pose;
        Eigen::Vector3d _com_initial_pose, _com_final_pose;
        double _step_clearing, _com_clearing;
        double _starTime, _endTime;
    };  
    
    class data_step_bezier
    {
    public:
        
        void set_data_step(Eigen::Vector3d step_initial_pose, 
                           Eigen::Vector3d step_final_pose,
                           double step_clearing)
        {
            _step_initial_pose =  step_initial_pose;
            _step_final_pose = step_final_pose;
            _step_clearing = step_clearing;   
        }
    
        void log(XBot::MatLogger::Ptr logger) { logger->add("step_initial_pose_Bezier", _step_initial_pose);
                                                logger->add("step_final_pose_Bezier", _step_final_pose);  
                                                logger->add("step_clearing_Bezier", _step_clearing);
                                                logger->add("points_bezier", _points_bezier_x);
                                                logger->add("points_bezier", _points_bezier_z);
        }
        
        const Eigen::Vector3d get_foot_initial_pose() const {return _step_initial_pose;};
        const Eigen::Vector3d get_foot_final_pose() const {return _step_final_pose;};
        const double get_step_clearing() const {return _step_clearing;};
        const Eigen::Vector2d get_points_bezier_x() const {return _points_bezier_x;};
        const Eigen::Vector3d get_points_bezier_z() const {return _points_bezier_z;};
        
        void set_foot_initial_pose(Eigen::Vector3d step_initial_pose) {_step_initial_pose =  step_initial_pose;};
        void set_foot_final_pose(Eigen::Vector3d step_final_pose) {_step_final_pose = step_final_pose;};
        void set_step_clearing(double step_clearing) {_step_clearing = step_clearing;};
        Eigen::Vector2d set_points_bezier_x(Eigen::Vector2d points_bezier_x) {_points_bezier_x = points_bezier_x;}; // TODO It should be possible to add a desired number of points
        Eigen::Vector3d set_points_bezier_z(Eigen::Vector3d points_bezier_z) {_points_bezier_z = points_bezier_z;};
        
    private:
        
        Eigen::Vector3d _step_initial_pose, _step_final_pose;
        
        Eigen::Vector2d _points_bezier_x;
        Eigen::Vector3d _points_bezier_z;

        double _step_clearing;
    };  
    
    
    
    class data_com
    {
    public:
        
        void log(XBot::MatLogger::Ptr logger) { logger->add("com_initial_pose", _com_initial_pose);}
        const Eigen::Vector3d get_com_initial_pose() const {return _com_initial_pose;};
        void set_com_initial_pose(Eigen::Vector3d com_initial_pose) {_com_initial_pose = com_initial_pose;};

    private:
        
        Eigen::Vector3d _com_initial_pose;
    };  
    
    class item_MpC
    {
        public:
            
        item_MpC(double initial_height, double Ts, double T)
        {
            
            double h = initial_height; //TODO current or initial?
            double w = sqrt(9.8/h);
            
            // inverted pendulum
            _integrator = OpenMpC::dynamics::LtiDynamics::Integrator(3,1);
            _window_length = T;
            
            _Ts = Ts;
            _C_zmp << 1 ,0, -1.0/std::pow(w, 2.0);
            
            _integrator->addOutput("zmp", _C_zmp);
            
//             Ts = 0.01; // dt window
//             
//             T = 5; //length window in sec
            
            int N = _window_length/0.01; //same as the integration time 
            
            OpenMpC::UnconstrainedMpc lqr(_integrator, _Ts, N+1);
            
            Eigen::MatrixXd Q(1,1);
            Eigen::MatrixXd R(1,1);
            
            Q << 1000000; //1000000
            R << 1;
            
            lqr.addInputTask(R);
            lqr.addOutputTask("zmp", Q);
            
            lqr.compute();
            
//             _K_fb.resize(3,1);
//             _K_prev.resize(N+1,1);
            
            _K_fb = lqr.getStateFeedbackGain();
            _K_prev = lqr.getOutputFeedforwardGainPreview("zmp");
                 
//             std::cout << "_K_fb: " << _K_fb << std::endl;
//             std::cout << "_K_prev: " << _K_prev << std::endl;
//             
//             std::cout << "K_fb_size: " << _K_fb.size() << std::endl;
//             std::cout << "K_prev_size: " << _K_prev.size() << std::endl;
            
            
            
            
        };
        

        

        Eigen::MatrixXd _K_fb;
        Eigen::MatrixXd _K_prev;
        double _window_length;
        Eigen::Matrix<double, 1,3> _C_zmp;
        OpenMpC::dynamics::LtiDynamics::Ptr _integrator;
        double _Ts;
        

        
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
    Eigen::Affine3d l_sole_orientation_goal();
    Eigen::Affine3d r_sole_orientation_goal();
    
    void q1_callback(const std_msgs::Float64 msg_rcv); //this is called by ros
    
    void set_q1(double cmd_q1);
    double get_q1();
    bool new_q1();
    
    double sense_q1();
    double sense_qlat();
    
    Eigen::MatrixXd get_supportPolygon();
    
    void update_pose(Eigen::Vector3d *current_pose, Eigen::Vector3d update);
    
    
    double initialize_MpC();
    Eigen::Vector3d lateral_com(double t);
    
    Eigen::Vector3d calc_com(double q1);
    Eigen::Vector3d calc_step(double q1);
    
    int impact_detected(); 
    bool impact();
    void fakeCOM();
    void exe(double time);  // TODO put everything but this on the protected side
    
    double lat_oscillator_com(double starting_time, double phase);
    
    void send(std::string type, Eigen::Vector3d command);
    void send_com(Eigen::Vector3d com_command);
    void send_step(Eigen::Vector3d foot_command);
    
    void update_com();
    void update_step();
    
    void run_walk();
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

    
    
    double getPt( double n1, double n2, double perc);
    double getBezierCurve(Eigen::VectorXd coeff_vec, double tau);
    double getBezierCurve(Eigen::VectorXd coeff_vec, Eigen::VectorXd coeff_vec_t, double tau);

    void first_q1();
    int get_n_step() {return _step_counter;};
    
    
    void initialize_cmd_fake_q1();
    void cmd_fake_q1();
    
    void generate_zmp(double y_start, double t_start, double double_stance, double dt, Eigen::VectorXd& zmp_t, Eigen::VectorXd& zmp_y);
    void lSpline(Eigen::VectorXd x, Eigen::VectorXd y, double dt, Eigen::VectorXd& X, Eigen::VectorXd& Y);
    void zmp_traj(double window_start, double window_end, Eigen::VectorXd& zmp_window_t, Eigen::VectorXd& zmp_window_y);
    
    
    
    void initializeMpc();
    double _q1_fake;
    double _reset_condition = 0;
    double _impact_cond = 0;
    double _reset_time = 0;
    
    double _starting_time = 0;
    double _internal_time = 0;
protected:
    
//     ros::NodeHandle n;
    double _start_walk;
    double _steep_coeff;
    
    double _reducer;
    int _numerator = 0;
    ros::Publisher _com_pub;     

    std::map<robot_interface::Side, ros::Publisher> _sole_pubs;
    ros::Subscriber _q1_sub;
    
    ros::Publisher _q1_pub;
    
    robot_interface _initial_pose, _previous_initial_pose;
    robot_interface_ROS _current_pose_ROS;
    
    double _q1_cmd;
    double _q1_state;
    
    double _initial_height;
    int _joint_number = 10; /*ankle_pitch_angle*/

    Eigen::Vector3d _foot_trajectory;
    
    bool  _flag = true;
    
    double _q1_step = 0;
    double _q1_offset;
    bool _check_received = false;
    
    Eigen::VectorXd _zmp_t;
    Eigen::VectorXd _zmp_y;
    
    data_step_poly _poly_step;
    data_step_bezier _bezi_step;
    data_com _com_info;
    std::shared_ptr<item_MpC> _MpC_lat;
    
    robot_interface_ROS::Side _current_side = robot_interface::Side::Double; 
    robot_interface::Side _other_side;
    
    XBot::MatLogger::Ptr _logger;
    
    double _terrain_heigth;
    
    int _step_counter;
//     ros::NodeHandle n;
    
    double _q_initial;
    double _q_min;
    double _q_max;
    
    bool _init_completed = 0;
    
    Eigen::VectorXd _zmp_window_t;
    Eigen::VectorXd _zmp_window_y;

//     Eigen::VectorXd u(1);
    Eigen::Matrix<double,1,1> _u;
    
    double _step_duration;
    Eigen::Vector3d _com_y; 
    
    class param
    {
    public:
        
        double get_crouch() {return _crouch;};
        double get_clearance_step() {return _clearance_step;};
        double get_duration_step() {return _duration_step;};
        robot_interface::Side get_first_step_side() {return _first_step_side;};
        int get_max_steps() {return _max_steps;};
        double get_double_stance() {return _double_stance;};
        double get_indent_zmp() {return _indentation_zmp;};
        double get_initial_time() {return _start_time;};
        void set_crouch(double crouch) {_crouch = crouch;};
        void set_clearance_step(double clearance_step) {_clearance_step = clearance_step;};
        void set_duration_step(double duration_step) {_duration_step = duration_step;};
        void set_first_step_side(robot_interface::Side first_step_side) {_first_step_side = first_step_side;};
        void set_max_steps(int max_steps) {_max_steps = max_steps;};
        void set_double_stance(double double_stance) {_double_stance = double_stance;};
        void set_indent_zmp(double indentation_zmp) {_indentation_zmp = indentation_zmp;};
        void set_start_time(double start_time) {_start_time = start_time;
            
        };
    private:
        
        double _crouch, _clearance_step, _duration_step, _indentation_zmp, _double_stance, _start_time;
        robot_interface::Side _first_step_side;
        int _max_steps;
        
    } _initial_param;
    
    double _initial_q1;
    
    State _current_state = State::INIT;
    Event _event = Event::EMPTY;
    
    Eigen::Vector3d _pointsBezier_z;
    Eigen::Vector2d _pointsBezier_x;
    
    Eigen::Vector3d _initial_com_position;
    Eigen::Vector3d _initial_sole_position;
    Eigen::Vector3d _final_sole_position;
    
    Eigen::Vector3d lateral_com();
    
    void v_core(double time);
    void commander(double time, double tau);
    
    bool ST_init(double time);
    bool ST_idle(double time);
    bool ST_firstStep();
    bool ST_walk();
    bool ST_lastStep();
    int _cycleCounter = 1;
    
    bool _initCycle = 1; //just needed to stop the code after the first cycle of walking
    
    
    
        friend std::ostream& operator<<(std::ostream& os, Event s)
    {
        switch (s)
        {
            case Event::EMPTY : return os << "empty";
            case Event::IMPACT :  return os << "impact";
            case Event::START :  return os << "start";
            case Event::STOP :  return os << "stop";
            default : return os << "wrong event";
        }
    };
    
    
    friend std::ostream& operator<<(std::ostream& os, State s)
    {
        switch (s)
        {
            case State::IDLE :  return os << "idle";
            case State::INIT :  return os << "init";
            case State::FIRSTSTEP :  return os << "first step";
            case State::LASTSTEP :  return os << "last step";
            case State::WALK : return os << "walking";
            case State::EXIT : return os << "exit";
            default : return os << "wrong state";
        }
    };

};



#endif















        
