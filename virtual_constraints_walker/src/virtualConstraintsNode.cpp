#include <virtualConstraintsNode.h>
#include <atomic>


#include <boost/geometry.hpp>
#include <boost/polygon/polygon.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
// #include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>

#include <XmlRpcValue.h>

#define runOnlyOnce ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    
 
    
virtualConstraintsNode::virtualConstraintsNode()
    {
//         initialze_cmd_fake_q1(); //TODO
        
        std::string this_node_name = ros::this_node::getName();
        _logger = XBot::MatLogger::getLogger("/tmp/" + this_node_name);
        ros::NodeHandle n;
//         ros::NodeHandle nh("xbotcore");
        
        _real_com_pos.setZero();
        _real_com_vel.setZero();
        
        

    
        f_callback = boost::bind(&item_gains::callback, &controller_gains, _1, _2);
        _dyn_rec_server.setCallback(f_callback);
    
    
        /* model and robot */
        auto cfg = XBot::ConfigOptionsFromParamServer(n);
        _robot = XBot::RobotInterface::getRobot(cfg);
        _model = XBot::ModelInterface::getModel(cfg);
        
        /* cartesian interface */
        _ci = std::make_shared<XBot::Cartesian::RosImpl>();
//         
        
        _dt = 0.01;
        _step_counter = 0;
        get_param_ros(); //initial parameters from ros
        
        _initial_pose = _current_pose_ROS;
        _initial_step_y = _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1);
        
        /* build foot stabilizer */
        _stab = std::make_shared<footStabilizer>(_dt);
        

        /* get position com */
        _poly_com.set_com_initial_position(_current_pose_ROS.get_com());

        Eigen::Affine3d cartesioWorld_T_com;
        _ci->getPoseReference("com", cartesioWorld_T_com);
        
//         _poly_com.set_com_initial_position();
        
//         std::cout << _current_pose_ROS.get_com().transpose() << std::endl;
//         std::cout << base_T_com.translation().transpose() << std::endl;
        
        _q1_state = sense_q1();
        
        _terrain_heigth =  _current_pose_ROS.get_sole(_current_side).coeff(2);

        /* prepare subscriber for position and velocity of the FLOATING BASE from gazebo */
        _fb_pos_sub = n.subscribe("/xbotcore/floating_base_position", 10, &item_fb::pos_callback, &_fb);
        _fb_vel_sub = n.subscribe("/xbotcore/floating_base_velocity", 10, &item_fb::vel_callback, &_fb);
   
    
//      prepare subscriber node for commands
        _switch_srv = n.advertiseService("/virtual_constraints/walk_switch", &virtualConstraintsNode::cmd_switch, this);
             
//      prepare subscriber node
        _q1_sub = n.subscribe("/q1", 10, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 10); /*publish to /cartesian/com/reference*/
        
        _sole_pubs[robot_interface::Side::Left] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/l_sole/reference", 10); /*publish to /l_sole/reference*/
        _sole_pubs[robot_interface::Side::Right] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 10); /*publish to /r_sole/reference*/
        
        _waist_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian//Waist/reference", 10);
        
//        // required for com STABILIZER
        _zmp_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com_stabilizer/zmp/reference", 10);
    
        /* publish cp reference */
        _cp_ref_pub = n.advertise<geometry_msgs::PoseStamped>("/virtual_constraints/cp_ref", 10);
        
        /* publish cp real */
        _cp_real_pub = n.advertise<geometry_msgs::PoseStamped>("/virtual_constraints/cp_real", 10);
        
        /* publish foot stabilizer angle */
        _footstab_pub = n.advertise<geometry_msgs::PoseStamped>("/virtual_constraints/foot_stab", 10);
        
        /* publish real com from gazebo */
        _real_com_pub = n.advertise<nav_msgs::Odometry>("/virtual_constraints/real_com", 10);
        
        /* publish bool walking switch */
        _switch_walk_pub = n.advertise<std_msgs::Bool>("/virtual_constraints/bool_walking", 10);
    }

bool virtualConstraintsNode::get_param_ros()
    {
        ros::NodeHandle nh_priv("~");
        int max_steps;
        double clearance_heigth, duration_step, drop, indentation_zmp, duration_double_stance, start_time, lean_forward, max_inclination, mpc_Q, mpc_R, duration_preview_window;
        std::vector<double> thresholds_impact_right(2), thresholds_impact_left(2);
        
        bool real_impacts, walking_forward, use_poly_com, manage_delay; 
        
        std::string first_side;

        /*default parameters*/
        double default_drop = -0.12;
        double default_clearance_heigth = 0.1;
        double default_duration_step = 2;
        std::string default_first_side = "Left";
        int default_max_steps = 10;
        double default_indentation_zmp = 0;
        double default_duration_double_stance = 0;
        double default_start_time = 1;
        double default_lean_forward = 0;
        std::vector<double> default_thresholds_impact_right = {50, 100};
        std::vector<double> default_thresholds_impact_left = {50, 100};
        bool default_real_impacts = 0;
        bool default_walking_forward = 0;
        double default_max_inclination = 0.1;
        double default_mpc_Q = 1000000;
        double default_mpc_R = 1;
        bool default_use_poly_com = 0;
        bool default_manage_delay = 1;
        double default_duration_preview_window = 5; //sec
        
        drop = nh_priv.param("initial_crouch", default_drop);
        max_steps = nh_priv.param("max_steps", default_max_steps);
        duration_step = nh_priv.param("duration_step", default_duration_step);
        clearance_heigth = nh_priv.param("clearance_step", default_clearance_heigth);
        first_side = nh_priv.param("first_step_side", default_first_side);
        indentation_zmp = nh_priv.param("indent_zmp", default_indentation_zmp);
        duration_double_stance = nh_priv.param("double_stance_duration", default_duration_double_stance);
        start_time = nh_priv.param("start_time", default_start_time);
        lean_forward = nh_priv.param("lean_forward", default_lean_forward);
        thresholds_impact_right = nh_priv.param("force_sensor_threshold_right", default_thresholds_impact_right);
        thresholds_impact_left = nh_priv.param("force_sensor_threshold_left", default_thresholds_impact_left);
        real_impacts = nh_priv.param("real_impacts", default_real_impacts);
        walking_forward = nh_priv.param("walking_forward", default_walking_forward); 
        max_inclination = nh_priv.param("max_inclination", default_max_inclination); 
        mpc_Q = nh_priv.param("mpc_Q", default_mpc_Q); 
        mpc_R = nh_priv.param("mpc_R", default_mpc_R);
        use_poly_com = nh_priv.param("use_poly_com", default_use_poly_com);
        duration_preview_window = nh_priv.param("duration_preview_window", default_duration_preview_window);
        
        _initial_param.set_crouch(drop);
        _initial_param.set_max_steps(max_steps);
        _initial_param.set_duration_step(duration_step);
        _initial_param.set_clearance_step(clearance_heigth);
        _initial_param.set_indent_zmp(indentation_zmp);
        _initial_param.set_double_stance(duration_double_stance);
        _initial_param.set_start_time(start_time);
        _initial_param.set_lean_forward(lean_forward);
        _initial_param.set_threshold_impact_right(thresholds_impact_right);
        _initial_param.set_threshold_impact_left(thresholds_impact_left);
        _initial_param.set_switch_real_impact(real_impacts);
        _initial_param.set_walking_forward(walking_forward);
        _initial_param.set_max_inclination(max_inclination);
        _initial_param.set_MPC_Q(mpc_Q);
        _initial_param.set_MPC_R(mpc_R);
        _initial_param.set_use_poly_com(use_poly_com);
        _initial_param.set_duration_preview_window(duration_preview_window);
        if (first_side == "Left")
                _initial_param.set_first_step_side(robot_interface::Side::Left);
        else if (first_side == "Right")
                _initial_param.set_first_step_side(robot_interface::Side::Right);
        else std::cout << "unknown side starting command" << std::endl;
    }   

Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
        /* center the CoM w.r.t. the feet */
        straight_com(0) = _current_pose_ROS.get_sole(_current_side).coeff(0) + _initial_param.get_lean_forward();
        straight_com(1) = (_current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1) + _current_pose_ROS.get_sole(robot_interface::Side::Right).coeff(1))/2; //TODO put in
        straight_com(2) = _initial_param.get_crouch();
        
        /*TODO PUT DEFAULT POSITION*/
        _poly_com.set_com_initial_position(straight_com);
        std::cout << straight_com.transpose() << std::endl;
        return straight_com;
    }
    
Eigen::Affine3d virtualConstraintsNode::l_sole_orientation_goal()
    {      
        Eigen::Affine3d sole_o = _current_pose_ROS.get_sole_tot(robot_interface::Side::Left);
        
        Eigen::Quaterniond goal_orientation;
        
        goal_orientation.x() = 0;
        goal_orientation.y() = 0;
        goal_orientation.z() = 0;
        goal_orientation.w() = 1;
        
        sole_o.linear() = goal_orientation.normalized().toRotationMatrix();
        return sole_o;
    }

Eigen::Affine3d virtualConstraintsNode::r_sole_orientation_goal()
    {      
        Eigen::Affine3d sole_o = _current_pose_ROS.get_sole_tot(robot_interface::Side::Right);
        
        Eigen::Quaterniond goal_orientation;
        
        goal_orientation.x() = 0;
        goal_orientation.y() = 0;
        goal_orientation.z() = 0;
        goal_orientation.w() = 1;
        
        sole_o.linear() = goal_orientation.normalized().toRotationMatrix();
        return sole_o;
    }
    
int virtualConstraintsNode::straighten_up_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
    {
        
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_com("cartesian/com/reach", true); /*without /goal!!*/
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_r_sole("cartesian/r_sole/reach", true); /*without /goal!!*/
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_l_sole("cartesian/l_sole/reach", true); /*without /goal!!*/
      
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_com.waitForServer(); //will wait for infinite time
        ac_r_sole.waitForServer();
        ac_l_sole.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        
        float cmd_duration_time;
        cmd_duration_time = 1; //15;
        
        cartesian_interface::ReachPoseGoal goal_com;
        cartesian_interface::ReachPoseGoal goal_l_step, goal_r_step;
        
        // send a goal to the action
        geometry_msgs::Pose cmd_initial_l_sole, cmd_initial_r_sole;
        
        Eigen::Vector4d orientation_sole;
        
        geometry_msgs::Pose q_l;
        geometry_msgs::Pose q_r;
        
        tf::poseEigenToMsg(l_sole_orientation_goal(), q_l);
        tf::poseEigenToMsg(r_sole_orientation_goal(), q_r);
        
        goal_r_step.frames.push_back(q_r); /*wants geometry_msgs::Pose*/
        goal_r_step.time.push_back(cmd_duration_time); 
        
        goal_l_step.frames.push_back(q_l); /*wants geometry_msgs::Pose*/
        goal_l_step.time.push_back(cmd_duration_time);      
        
        ac_r_sole.sendGoal(goal_r_step);
        ac_l_sole.sendGoal(goal_l_step);
        
        geometry_msgs::Pose cmd_initial_com;
        
        
        tf::pointEigenToMsg(straighten_up_goal(), cmd_initial_com.position);
   
        goal_com.frames.push_back(cmd_initial_com); /*wants geometry_msgs::Pose*/
        goal_com.time.push_back(cmd_duration_time);

        ac_com.sendGoal(goal_com);
    
        //wait for the action to return
        bool finished_before_timeout = ac_com.waitForResult(ros::Duration(5.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_com.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        /* fill initial pose with pose after straighten_up_action */
        _current_pose_ROS.sense();
        _initial_pose = _current_pose_ROS; 
        
        _foot_pos_y_right = _initial_pose.get_sole_tot(robot_interface::Side::Right).translation()[1];
        _foot_pos_y_left = _initial_pose.get_sole_tot(robot_interface::Side::Left).translation()[1];
        
        _distance_feet << 0, fabs(_foot_pos_y_right - _foot_pos_y_left), 0; /*HACK*/
        
        _initial_height = fabs(_current_pose_ROS.get_com().coeff(2) - _current_pose_ROS.get_sole(_current_side).coeff(2));
        _initial_com_to_ankle = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
        
        _starting_com_pos = _initial_pose.get_com();
        _starting_foot_pos = _initial_pose.get_sole_tot(_current_side); // get position of starting point
        
        _starting_foot_pos_left = _initial_pose.get_sole_tot(robot_interface::Side::Left);
        _starting_foot_pos_right = _initial_pose.get_sole_tot(robot_interface::Side::Right);
       
        _first_com_pos = _starting_com_pos;
        
        
        _poly_com.set_com_initial_position(_initial_pose.get_com());
        
        // HACK _current com is not actually here at the beginning, but it is needed
        _current_world_to_com = _current_pose_ROS.get_world_to_com();
        _current_world_to_com(0) = _current_pose_ROS.get_world_to_com().coeff(0) + _current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2)*tan(_initial_param.get_max_inclination()) - _initial_param.get_lean_forward();
        _initial_q1 = sense_q1();
        
//         std::cout << sense_q1() << std::endl;
        //exit
        return 0;
}


bool virtualConstraintsNode::cmd_switch(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
    {
        if (req.data)
        {
            _last_event = _event;
            _event = Event::START;
            std::cout << "START received. Starting ..." << std::endl;
            res.message = "Walking started";
            res.success = true;
        }
        else
        {
            _last_event = _event;
            _event = Event::STOP;
            std::cout << "STOP received. Stopping ..." << std::endl;
            res.message = "Walking stopped";
            res.success = false;
        }
            return true;
    };
    
void virtualConstraintsNode::cmd_switch_callback(const std_msgs::Bool msg_rcv)
{
        if (msg_rcv.data == true)
        {
            _last_event = _event;
            _event = Event::START;
        } 
        else
        {
            _last_event = _event;
            _event = Event::STOP;
            _stopped_received = 1; // for logging
        }
}

void virtualConstraintsNode::q1_callback(const std_msgs::Float64 msg_rcv) //this is called by ros
{
    _q1_cmd = msg_rcv.data;
    _check_received = true;
}

double virtualConstraintsNode::get_q1()
{       
    return _q1_cmd;
    ROS_INFO("%f", _q1_state);
        
}

double virtualConstraintsNode::sense_q1()
{   
//         _current_pose_ROS.sense(); // TODO put back?
    double q1, q2;
    Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
//     std::cout << "_current_world_to_com: " << _current_world_to_com.transpose() << std::endl;
//     std::cout << "get_world_to_com: " << _current_pose_ROS.get_world_to_com().transpose() << std::endl;
    Eigen::Matrix2d _R_steer_local;
    double theta;
    
       /* TODO refactor: this is needed for the steering */
    if (_step_counter >= 4  && _step_counter < 15)
    {
        theta = - _theta_steer;
        _R_steer_local << cos(theta), -sin(theta),
                          sin(theta), cos(theta);
    }
    else
    {
        theta = 0;
        _R_steer_local << cos(theta), -sin(theta),
                          sin(theta), cos(theta);
    }
        
    
    /* TODO refactor: this is needed for the lenght of the step */
    /* take care of the lenght of the step*/
    double offset_q1;
    if (_step_counter >= 4+1  && _step_counter < 5+1) /*Left*/
    {
        offset_q1 = 0.05; /* 0.001 */
    }
    else if (_step_counter >= 5+1 && _step_counter < 6+1) /*Right*/
    {
        offset_q1 = 0.05; /* 0.02 */ 
    }
    else if (_step_counter >= 6+1 && _step_counter < 7+1) /*Left*/
    {
        offset_q1 = 0.05;
    }
    else if (_step_counter >= 7+1 && _step_counter < 8+1) /*Right*/
    {
        offset_q1 = 0.05;
    }
    else if (_step_counter >= 8+1 && _step_counter < 9+1) /*Left*/
    {
        offset_q1 = 0.05;
    }
    else if (_step_counter >= 9+1 && _step_counter < 10+1) /*Right*/
    {
        offset_q1 = 0.05;
    }
    else if (_step_counter >= 10+1 && _step_counter < 15+1) /*Left*/
    {
        offset_q1 = 0.05;
    }    
    else       
    {
        offset_q1 = 0.05;
    }
    
    
    if (_current_side == robot_interface::Side::Double)
    {
        Eigen::Vector3d left_ankle_to_com, right_ankle_to_com, world_to_com;
        Eigen::Vector3d dist_com;
        
        left_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left);
        right_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right);
        
        world_to_com = _current_pose_ROS.get_world_to_com(); //current world to CoM
        
//         std::cout << "world_to_com: " << world_to_com.transpose() << std::endl;
        
        dist_com = world_to_com - _current_world_to_com; // current - initial (resetted at each step) CoM
               
        dist_com.head(2) = _R_steer_local * dist_com.head(2); // rotate back CoM

        
//         std::cout << "theta: " << theta << std::endl;
//         double theta_calc = atan2(world_to_com(1) - _current_world_to_com(1), world_to_com(0) - _current_world_to_com(0));
//         std::cout << "theta_calc: " << theta_calc << std::endl;
    
//         q1 = atan(dist_com(0)/ - left_ankle_to_com(2)) - _initial_param.get_max_inclination(); // HACK

        q1 = atan(dist_com(0)/ fabs(left_ankle_to_com(2))) - offset_q1; // HACK
        

    }
    else
    {
    Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com, world_to_com;
    Eigen::Vector3d dist_com;
    
    robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
    swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
    stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
    
    
        
        world_to_com = _current_pose_ROS.get_world_to_com(); //current world to CoM
        
        
        dist_com =  world_to_com - _current_world_to_com; // current CoM - initial CoM(resetted at each step) 
               
        dist_com.head(2) = _R_steer_local * dist_com.head(2); // rotate back CoM
    
        q1 = atan(dist_com(0)/ fabs(stance_ankle_to_com(2))) - offset_q1; // HACK
//         q1 = atan(dist_com(0)/- stance_ankle_to_com(2)) - _q1_max; // HACK
    }

    return q1;
}  

bool virtualConstraintsNode::new_q1()
{
    bool flag_q1 = false;
    double last_q1_step = 0;
    last_q1_step = _q1_step;
    _q1_step = get_q1();
    
    if (fabs(last_q1_step - _q1_step) >= 1e-10) /* TODO this is very sad*/ /*it's a problem of initialization*/
    {
        flag_q1 = true;
    }

    return flag_q1;
}
    

void virtualConstraintsNode::left_sole_phase()
{
    Eigen::Matrix<double, 6 ,1> ft_left = _current_pose_ROS.get_ft_sole(robot_interface::Side::Left);
  
    switch (_current_phase_left) {
        case Phase::LAND :
            if (fabs(ft_left.coeff(2)) <= _initial_param.get_threshold_impact_left().at(0)) //threshold_min
            {
                _previous_phase_left = _current_phase_left;
                _current_phase_left = Phase::FLIGHT;
            }
            break;
        case Phase::FLIGHT : 
            if (fabs(ft_left.coeff(2)) >= _initial_param.get_threshold_impact_left().at(1)) //threshold_max
            {
                _previous_phase_left = _current_phase_left;
                _current_phase_left = Phase::LAND;
            }
            break;
        default : 
            throw std::runtime_error(std::string("Phase not recognized (left sole)!"));
            break;
    }
}

void virtualConstraintsNode::right_sole_phase()
{
    Eigen::Matrix<double, 6 ,1> ft_right = _current_pose_ROS.get_ft_sole(robot_interface::Side::Right);

    switch (_current_phase_right) {
        case Phase::LAND :
            if (fabs(ft_right.coeff(2)) <= _initial_param.get_threshold_impact_right().at(0)) //threshold_min
            {
                _previous_phase_right = _current_phase_right;
                _current_phase_right = Phase::FLIGHT;
                
            }
            break;
        case Phase::FLIGHT : 
            if (fabs(ft_right.coeff(2)) >= _initial_param.get_threshold_impact_right().at(1)) //threshold_max
            {
                _previous_phase_right = _current_phase_right;
                _current_phase_right = Phase::LAND;
            }
            break;
        default : 
            throw std::runtime_error(std::string("Phase not recognized (right sole)!"));
            break;
    }
}

bool virtualConstraintsNode::real_impacts()
{       
        right_sole_phase();
        left_sole_phase();

        
        // left foot
        if (_previous_phase_left == Phase::FLIGHT &&  _current_phase_left == Phase::LAND)
        {
            std::cout << "LEFT impact detected" << std::endl;
            _previous_phase_left = _current_phase_left;
            _time_real_impact = _internal_time;
            return true;
        }
        
        // right foot
        if (_previous_phase_right == Phase::FLIGHT &&  _current_phase_right == Phase::LAND)
        {
            std::cout << "RIGHT impact detected" << std::endl;
            _previous_phase_right = _current_phase_right;
            _time_real_impact = _internal_time;
            return true;
        }
        
        
        return false;
}

bool virtualConstraintsNode::fake_impacts()
{ 
//     std::cout << "_q1_max: " << _q1_max << std::endl;
//     std::cout << "sense_q1: " << sense_q1() << std::endl;
    
    if (_q1_min <= _q1_max)
    {
        _cond_q = (sense_q1() >= _q1_max);
    }
    else if (_q1_min > _q1_max)
    {
        _cond_q = (sense_q1() <= _q1_max);
    }
    
    _cond_step = fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-3;
    
    if (_cond_step && _cond_q)
    {
        _time_fake_impact = _internal_time;
        return true;
    }
    
    return false;
}

bool virtualConstraintsNode::impact_detector()
{
    if (_initial_param.get_switch_real_impact())
    {
        return real_impacts();
    }
    else
    {
//         _flag_impact =  real_impacts();
        return fake_impacts();
    }
}

int virtualConstraintsNode::impact_routine()                
    {
        if (_current_state != State::IDLE)
        {     
            if (impact_detector())
            {
                _last_event = _event;
                _event = Event::IMPACT;
//                 _current_pose_ROS.sense();
               
                robot_interface::Side last_side = _current_side;
//                 std::cout << "Last side: " << last_side << std::endl;
                _initial_pose = _current_pose_ROS;
            // ----------------------------------
                _q1_min = -sense_q1(); /*HACK*/
            // -------------------------------
                _current_side = robot_interface::Side::Double;

                std::cout << "Impact! Current side: " << _current_side << std::endl;
                
                /* get position of BOTH foot */
                _right_foot_position = _current_pose_ROS.get_sole_tot(robot_interface::Side::Right);
                _left_foot_position = _current_pose_ROS.get_sole_tot(robot_interface::Side::Left);
                // ---------------------------------------------------- 
//                 _poly_com.set_com_initial_position(_initial_pose.get_com());
                
                _current_world_to_com = _current_pose_ROS.get_world_to_com();
                // ----------------------------------------------------
// 
                
                if (last_side == robot_interface::Side::Left)
                {
                    _current_side = robot_interface::Side::Right;
                    _other_side = robot_interface::Side::Left;
                }
                else if (last_side == robot_interface::Side::Right)
                {
                    _current_side = robot_interface::Side::Left;
                    _other_side = robot_interface::Side::Right;
                }
                else ROS_INFO("wrong side");
                
//                 _q1_min = sense_q1(); /*HACK*/
                std::cout << "State changed. Current side: " << _current_side << std::endl;

                return 1;
            }
            else
            {
                return 0; // if impact detector does not detect an impact
            }
        }
        else
        {
            return 0; // if we are in IDLE, the impact_routine is disabled
        }
    }


    
// void virtualConstraintsNode::first_q1()
//     {
//         ROS_INFO("waiting for command...");
//         std::cout << "Initial state: " << _current_side << std::endl;
//         while (!_check_received)
//         {
//             _current_pose_ROS.sense();
//             
//             if (_check_received)
//             {
//                 _q1_state = _q1_cmd;
//             }
//                 
//         }
//         ROS_INFO("command received! q1 = %f", _q1_state);
//     }
    
void virtualConstraintsNode::exe(double time)
{       

    double dt = _dt;
    if (_init_completed == 0)
    {
        initialize(time);
    }
    else
    {
        if (_first_time == 0)
        {
            _starting_time = time;
            _first_time = 1;
            
        }
        
    _internal_time = time - _starting_time;
    
    _impact_cond = _internal_time - _reset_time;
    
//     val_getter();
//  // -------------for q1--------------------------------------
    q_handler();
//     q_max_handler(); /* TODO _q1_max it's already setted inside core */
//  // ---------------------------------------------------------  
    
//     if (_internal_time >= _initial_param.get_start_time() && _cycle_counter == 0)
//     {
//         _last_event = _event;
//         _event = Event::START;
//     };
//     
//     if (_cycle_counter == 2)
//     {
//         _last_event = _event;
//         _event = Event::STOP;
//         _stopped_received = 1;
//         
//     };

        if (impact_routine())
        {    
            std::cout<<"impact routine"<<std::endl;

            _reset_condition = _q1_temp;
//             _q1_temp = 0; //ver2
            
            _reset_time = _internal_time;
        }

//  // -------------for q1, after impact------------------------
        _q1_old = _q1;
        _q1 = _q1_temp - _reset_condition;
//  // --------------------------------------------------------- 

        core(_internal_time);
        commander(_internal_time);
    }
}

void virtualConstraintsNode::send(std::string type, Eigen::Vector3d command)
    {
        geometry_msgs::PoseStamped cmd;
        tf::pointEigenToMsg(command, cmd.pose.position);
        
        if (type == "right")
        {
            _sole_pubs[robot_interface::Side::Right].publish(cmd);
        }
        else if (type == "left")
        {
            _sole_pubs[robot_interface::Side::Left].publish(cmd);
        }
        else if (type == "com")
        {
            _com_pub.publish(cmd);  /*TODO make a map here, so there is only one publisher and you decide to whom*/
        }
        else std::cout << "you're trying to send the command to a non existent task" << std::endl;
    }

// void virtualConstraintsNode::receiver()
//     {
//             
//     }

void virtualConstraintsNode::send_com(Eigen::Vector3d com_command)
    {
        geometry_msgs::PoseStamped cmd_com;
        tf::pointEigenToMsg(com_command, cmd_com.pose.position);
        
        _com_pub.publish(cmd_com);
    }
void virtualConstraintsNode::send_step(Eigen::Vector3d foot_command)
    {
        geometry_msgs::PoseStamped cmd_sole;
        tf::pointEigenToMsg(foot_command, cmd_sole.pose.position);
        _sole_pubs[_current_side].publish(cmd_sole);
    }

void virtualConstraintsNode::send_step(Eigen::Affine3d foot_command)
    {
        geometry_msgs::PoseStamped cmd_sole;
        tf::poseEigenToMsg(foot_command, cmd_sole.pose);

        _sole_pubs[_current_side].publish(cmd_sole);
    }
    
void virtualConstraintsNode::send_step_right(Eigen::Affine3d foot_command)
    {
        geometry_msgs::PoseStamped cmd_sole_r;
        tf::poseEigenToMsg(foot_command, cmd_sole_r.pose);

        _sole_pubs[robot_interface::Side::Right].publish(cmd_sole_r);
    }

void virtualConstraintsNode::send_step_left(Eigen::Affine3d foot_command)
    {
        geometry_msgs::PoseStamped cmd_sole_l;
        tf::poseEigenToMsg(foot_command, cmd_sole_l.pose);

        _sole_pubs[robot_interface::Side::Left].publish(cmd_sole_l);
    }
    
void virtualConstraintsNode::send_zmp(Eigen::Vector3d zmp_command)
    {
        geometry_msgs::PoseStamped cmd_zmp;
        tf::pointEigenToMsg(zmp_command, cmd_zmp.pose.position);
        
        _zmp_pub.publish(cmd_zmp);
    }
    
void virtualConstraintsNode::send_waist(Eigen::Affine3d waist_command)
    {
        geometry_msgs::PoseStamped cmd_waist;
        tf::poseEigenToMsg(waist_command, cmd_waist.pose);
        
        _waist_pub.publish(cmd_waist);
    }

void virtualConstraintsNode::send_cp_ref(Eigen::Vector3d cp_ref)
{
        geometry_msgs::PoseStamped state_cp_ref;
        tf::pointEigenToMsg(cp_ref, state_cp_ref.pose.position);
        
        _cp_ref_pub.publish(state_cp_ref);
}

void virtualConstraintsNode::send_cp_real(Eigen::Vector3d cp_real)
{
        geometry_msgs::PoseStamped state_cp_real;
        tf::pointEigenToMsg(cp_real, state_cp_real.pose.position);
        
        _cp_real_pub.publish(state_cp_real);
}
void virtualConstraintsNode::send_footstab(Eigen::Vector3d footstab_command)
{
        geometry_msgs::PoseStamped state_footstab;
        tf::pointEigenToMsg(footstab_command, state_footstab.pose.position);
        
        _footstab_pub.publish(state_footstab);
}

void virtualConstraintsNode::send_real_com(Eigen::Vector3d real_com_state, Eigen::Vector3d real_com_velocity)
{
        nav_msgs::Odometry state_com_real;
        
        /* there is no angular velocity, only linear */
        Eigen::Matrix<double, 6, 1> twist_velocity;
        twist_velocity.setZero();
        twist_velocity[0] = real_com_velocity[0];
        twist_velocity[1] = real_com_velocity[1];
        twist_velocity[2] = real_com_velocity[2];

        tf::pointEigenToMsg(real_com_state, state_com_real.pose.pose.position);
        tf::twistEigenToMsg(twist_velocity, state_com_real.twist.twist);
        
        _real_com_pub.publish(state_com_real);
}


Eigen::Affine3d virtualConstraintsNode::compute_trajectory(Eigen::Affine3d T_i, Eigen::Affine3d T_f,
                                                double clearance,
                                                double start_time, double end_time, 
                                                double time
                                                )
    {

    
    //position
    
    Eigen::Vector3d ret;

    double dx0 = 0;
    double ddx0 = 0;
    
    double dxf = 0;
    double ddxf = 0;
    
    double dx; 
    double ddx;
    
    double beta = 1; //2
    double tau = std::min(std::max((time - start_time)/(end_time - start_time), 0.0), 1.0);
    double alpha = compute_swing_trajectory_normalized_plane(dx0, ddx0, dxf, ddxf, time_warp(tau, beta), &dx, &ddx);
    
    ret.head<2>() = (1-alpha)*T_i.translation().head<2>() + alpha*T_f.translation().head<2>();
    ret.z() = T_i.translation().z() + compute_swing_trajectory_normalized_clearing(T_f.translation().z()/clearance, time_warp(tau, beta))*clearance;
    
    //rotation
    
    Eigen::Quaterniond q_start(T_i.linear());
    Eigen::Quaterniond q_end(T_f.linear());
    
    double tau2, dtau2, ddtau2;
    virtualConstraintsNode::FifthOrderPlanning(0,0,0,1,0,0,start_time, end_time, time, tau2, dtau2, ddtau2);
    Eigen::Affine3d interpolated;
    interpolated.setIdentity();
    interpolated.linear() = q_start.slerp(tau, q_end).toRotationMatrix();
    interpolated.translation() = ret;
//     interpolated.translation() = (1 - tau)*start.translation() + tau*end.translation();
    
    return interpolated;
    
    }
            
Eigen::Vector3d virtualConstraintsNode::compute_swing_trajectory(const Eigen::Vector3d& start, 
                                                                 const Eigen::Vector3d& end,
                                                                 double clearance,
                                                                 double t_start, 
                                                                 double t_end, 
                                                                 double time,
                                                                 Eigen::Vector3d* vel,
                                                                 Eigen::Vector3d* acc
                                                                )
{
    Eigen::Vector3d ret;

    double dx0 = 0;
    double ddx0 = 0;
    
    double dxf = 0;
    double ddxf = 0;
    
    double dx; 
    double ddx;
    
    double beta = 1; //2
    double tau = std::min(std::max((time - t_start)/(t_end - t_start), 0.0), 1.0);
    double alpha = compute_swing_trajectory_normalized_plane(dx0, ddx0, dxf, ddxf, time_warp(tau, beta), &dx, &ddx);
    
    ret.head<2>() = (1-alpha)*start.head<2>() + alpha*end.head<2>();
    ret.z() = start.z() + compute_swing_trajectory_normalized_clearing(end.z()/clearance, time_warp(tau, beta))*clearance;
     
    return ret;
}

double virtualConstraintsNode::compute_swing_trajectory_normalized_plane(double dx0, double ddx0, 
                                                                         double dxf, double ddxf, 
                                                                         double tau, 
                                                                         double* __dx, double* __ddx)
{
    
    double x, dx, ddx;
    FifthOrderPlanning(0, dx0, ddx0, 1, dxf, ddxf, 0, 1, tau, x, dx, ddx);

    if(__dx) *__dx = dx;
    if(__ddx) *__ddx = ddx; 
    
    return x;
}

double virtualConstraintsNode::compute_swing_trajectory_normalized_clearing(double final_height, double tau, double* dx, double* ddx)
{
    double x = std::pow(tau, 3)*std::pow(1-tau, 3);
    double x_max = 1./64.;
    x = x/x_max;
    
//     if(dx) *dx = powdx.dot(avec);
//     if(ddx) *ddx = powddx.dot(avec);
    return x;
    
}

double virtualConstraintsNode::time_warp(double tau, double beta)
{
    return 1.0 - std::pow(1.0 - tau, beta);
}

void virtualConstraintsNode::FifthOrderPlanning(double x0, double dx0, double ddx0,  //initial position
                                                double xf, double dxf, double ddxf,  //final position
                                                double start_time, double end_time, 
                                                double time, double& x, double& dx, double& ddx 
                                                )
{
    Eigen::Matrix6d A;
    A << 1.0000,         0,         0,         0,         0,         0,
              0,    1.0000,         0,         0,         0,         0,
              0,         0,    0.5000,         0,         0,         0,
       -10.0000,   -6.0000,   -1.5000,   10.0000,   -4.0000,    0.5000,
        15.0000,    8.0000,    1.5000,  -15.0000,    7.0000,   -1.0000,
        -6.0000,   -3.0000,   -0.5000,    6.0000,   -3.0000,    0.5000;
    
    double alpha = (end_time-start_time);
    alpha = std::max(1e-6, alpha);
    double tau = (time - start_time)/alpha; // d/dt = d/d(alpha*tau)
    tau = std::max(0.0, tau);
    tau = std::min(tau, 1.0);
        
    Eigen::Vector6d b;
    b << x0, dx0*alpha, ddx0*std::pow(alpha,2.0), xf, dxf*alpha, ddxf*std::pow(alpha,2.0);
    
    Eigen::Vector6d coeffs = A*b;
    
    Eigen::Vector6d t_v, dt_v, ddt_v;
    for(int i = 0; i < 6; i++)
    {
        t_v(i) = std::pow(tau, i);
        dt_v(i) = i > 0 ? std::pow(tau, i-1)*i : 0;
        ddt_v(i) = i > 1 ? std::pow(tau, i-2)*i*(i-1) : 0;
        
    }
    
    x = coeffs.dot(t_v);
    dx = coeffs.dot(dt_v)/alpha;
    ddx = coeffs.dot(ddt_v)/(alpha*alpha);
}


void virtualConstraintsNode::lSpline(Eigen::VectorXd times, Eigen::VectorXd y, double dt, Eigen::VectorXd &times_vec, Eigen::VectorXd &Y)
{

    int n = times.size();
    double N = 0;
    
    
    Eigen::VectorXi N_chunks(n-1);
    
    for(int i=0; i<n-1; i++)
    { 
        N = N + (times.coeff(i+1)-times.coeff(i))/dt;
        N_chunks(i) = round((times.coeff(i+1)-times.coeff(i))/dt);   ; //round((times.coeff(i+1)-times.coeff(i))/dt);      
    }
    
//     std::cout << "N: " << N << std::endl;
//     std::cout << "N_progress: " << N_chunks.transpose() << std::endl;

    Y.resize(N+1,1);
    Y.setZero();
    
    times_vec.setLinSpaced(N, dt, dt*N);

//     std::cout << "times_vec_init" << times_vec(0) << std::endl;
//     std::cout << "times_vec: " << times_vec.transpose() << std::endl;
     
    int idx = 0;
    for(int i=0; i<n-1; i++)
    {
        Eigen::VectorXd temp_vec(N_chunks(i)+1);
        temp_vec.setLinSpaced(N_chunks(i)+1, y.coeff(i), y.coeff(i+1));

//         std::cout << "temp_vec.size(): " << temp_vec.size()-1 << std::endl;
        
        Y.segment(idx, temp_vec.size()-1) = temp_vec.segment(1,temp_vec.size()-1);
        idx += (temp_vec.size()-1);
        
//         std::cout << "idx: " << idx << std::endl;
    }

//     std::cout << "Y: " << Y.transpose() << std::endl;
    for (int i = 0; i < times_vec.size(); i++)
    {
        _logger->add("comy_traj_planned", times_vec(i));
        _logger->add("times_traj_planned", Y(i));
    }
}
    
Eigen::Vector3d virtualConstraintsNode::lateral_com(double time)
{
        
        double dt = _dt; //TODO take it out from here
     
        // algorithm to choose from T (duration of window prev for _item_MpC) the spatial pattern
        
        double velocity_step;
// 
        if (_step_type == Step::HALF)
        {
            velocity_step = _nominal_half_step / _initial_param.get_duration_step();
        }
        else if (_step_type == Step::FULL)
        {
            velocity_step = _nominal_full_step / _initial_param.get_duration_step();
        }
        
        double length_preview_window = _initial_param.get_duration_preview_window() * velocity_step; // spatial length
        double dx =  velocity_step * dt; // dx for the spatial window
        
        spatial_zmp(_current_spatial_zmp_y, _spatial_window_preview, length_preview_window, dx, _step_type); // spatial zmp
        _zmp_window_y.resize(_spatial_window_preview.size());
        _zmp_window_y = _spatial_window_preview;

        if (_current_state == State::STARTING)
        {
            /* this is for the beginning, to move the com before the step */
            if (_internal_time < _start_walk)
            {
                _zmp_starting.head(_zmp_starting.size()-1) = _zmp_starting.tail(_zmp_starting.size()-1);
                _logger->add("zmp_start", _zmp_starting);
                _zmp_window_y = _zmp_starting;
            }
        }
        
        _u = _MpC_lat->_K_fb * _com_y + _MpC_lat->_K_prev * _zmp_window_y;
        _MpC_lat->_integrator->integrate(_com_y, _u, dt, _com_y);
         
        return _com_y;
}

void virtualConstraintsNode::commander(double time)
{
  
        /* send com sagittal and lateral */ 
        Eigen::Vector3d delta_com, delta_com_rot;
        _previous_com_trajectory = _com_trajectory;
        robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
        Eigen::Vector3d com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(other_side);
        
        Eigen::Affine3d right_foot_trajectory, left_foot_trajectory;
        
        /* set right and left position of foot */
        right_foot_trajectory = _right_foot_position;
        left_foot_trajectory = _left_foot_position;
        
        /* com displacement */
        delta_com(0) = fabs(com_to_ankle_distance.z()) * tan(_q1);
        delta_com(1) = lateral_com(time).coeff(0);
        delta_com(2) = 0;
        
       /* ------------------------------------- steer -------------------------------------------------- */
       /* TODO refactor: this is needed for the steering */
       Eigen::Matrix2d R_steer_local;
       
        if (_step_counter >= 4 && _step_counter < 15)
        {
            double theta = _theta_steer;
                R_steer_local << cos(theta), -sin(theta),
                                sin(theta), cos(theta);
        }
        else
        {
                double theta = 0;
                R_steer_local << cos(theta), -sin(theta),
                                sin(theta), cos(theta);
        }
        
        delta_com_rot.setZero();
        delta_com_rot.head(2) = R_steer_local * delta_com.head(2);
        /* ---------------------------------------------------------------------------------------------- */
        
        _com_trajectory = _poly_com.get_com_initial_position() + delta_com_rot;
        

        /* -------------------------------------- fake com ---------------------------------------------- */
        
        _com_trajectory_fake = _initial_com_position_fake + delta_com;
        /* ---------------------------------------------------------------------------------------------- */
  
        /* UPDATE ROBOT AND MODEL FROM XBOTCORE */
        _robot->sense();
        _model->syncFrom(*_robot);
        _model->setFloatingBaseState(_fb.getPose(), _fb.getVelocity());
        _model->update();
        
        _model->getCOM(_real_com_pos);
        _model->getCOMVelocity(_real_com_vel);
        
        /* send real com from gazebo */
        send_real_com(_real_com_pos, _real_com_vel);
        
        /* UPDATE FOOT CONTROLLER */
        Eigen::Vector3d cp_inst_ref;
        cp_inst_ref[0] = get_com_velocity()[0]*sqrt(_initial_height/grav);
        cp_inst_ref[1] = get_com_velocity()[1]*sqrt(_initial_height/grav);
        cp_inst_ref[2] = 0.0;
       
        /* set CP ref from robot commands (com pos and vel commanded) */
        _stab->setInstantaneousCPRef(cp_inst_ref);
        send_cp_ref(cp_inst_ref);
        
        /* set reconfigurable gains for foot controller */
        if (controller_gains.areChanged())
        {
            _stab->setKp(controller_gains.getKp());
            _stab->setKd(controller_gains.getKd());
        }
        
        /* update stabilizer with com position and velocity REAL from gazebo */
        _stab->update(_real_com_pos[2], _real_com_vel);
        
        /* get CP from real com position and velocity */
        Eigen::Vector3d cp_real = _stab->getInstantaneousCP();
        send_cp_real(cp_real);
        
        /* get stabilizer action */
        Eigen::Vector3d foot_stab = _stab->getTheta();
        send_footstab(foot_stab);
        
        /* from stabilizer ankle angle to affine matrix */
        /* pitch */
        _base_R_l_sole *= (Eigen::AngleAxisd(foot_stab[0], Eigen::Vector3d::UnitY())).toRotationMatrix();
        _base_R_r_sole *= (Eigen::AngleAxisd(foot_stab[0], Eigen::Vector3d::UnitY())).toRotationMatrix();
        /* roll */
        _base_R_l_sole *= (Eigen::AngleAxisd(foot_stab[1], Eigen::Vector3d::UnitX())).toRotationMatrix();
        _base_R_r_sole *= (Eigen::AngleAxisd(foot_stab[1], Eigen::Vector3d::UnitX())).toRotationMatrix();

        
            

        /* send foot */
        
        _previous_foot_trajectory = _foot_trajectory;
        _foot_trajectory = compute_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time);

        /* ------------------- send always both feet ------------------ */
        if (_current_side == robot_interface::Side::Right)
        {
            right_foot_trajectory = _foot_trajectory;
            right_foot_trajectory.linear() *= _base_R_r_sole;
        }
        else if (_current_side == robot_interface::Side::Left)
        {
            left_foot_trajectory = _foot_trajectory;
            left_foot_trajectory.linear() *= _base_R_l_sole;
        } 
        else if (_current_side == robot_interface::Side::Double)
        {
            right_foot_trajectory = _right_foot_position;
            right_foot_trajectory.linear() *= _base_R_r_sole;
            
            left_foot_trajectory = _left_foot_position;
            left_foot_trajectory.linear() *= _base_R_l_sole;
        }
        /* ------------------------------------------------------------ */
        /* for stabilizer */
        double zmp_sag_ref, zmp_lat_ref;
        zmp_sag_ref = _com_trajectory.coeff(0);
        
        /* send ZMP ref before kajita */
//         int sign_first_stance_step = (_current_pose_ROS.get_sole(_other_side).coeff(1) > 0) - (_current_pose_ROS.get_sole(_other_side).coeff(1) < 0); //
//         zmp_lat_ref = _current_pose_ROS.get_sole_tot(_other_side).translation().coeff(1) - sign_first_stance_step * _initial_param.get_indent_zmp();
        /* send ZMP ref after kajita */
        zmp_lat_ref = _MpC_lat->_C_zmp*_com_y;
        _zmp_ref << zmp_sag_ref, zmp_lat_ref, 0;

        if (_started == 0 || (_started == 1 && _internal_time < _start_walk))
        {
            _zmp_ref << _first_com_pos.coeff(0), 0, 0;
        }
        
        /* send waist*/
        
        _waist_trajectory = _final_waist_pose;
        
        
        _logger->add("com_vel", sense_com_velocity());
        
//         _logger->add("previous_com_trajectory", _previous_com_trajectory);
        _logger->add("com_vel_cmd", get_com_velocity());
        
        _logger->add("foot_vel_cmd", get_foot_velocity());
        
        _logger->add("foot_vel_current", sense_foot_velocity());
         
        _logger->add("initial_pose_foot", _poly_step.get_foot_initial_pose().translation());
        _logger->add("final_pose_foot", _poly_step.get_foot_final_pose().translation());
        
//         std::cout << "time_begin: " << _poly_step.get_starTime() << std::endl;
//         std::cout << "time_end: " << _poly_step.get_endTime() << std::endl;
        
        _logger->add("com_pos", _current_pose_ROS.get_com());
        _logger->add("foot_pos_right", _current_pose_ROS.get_sole(robot_interface::Side::Right));
        _logger->add("foot_pos_left", _current_pose_ROS.get_sole(robot_interface::Side::Left));
        
        _logger->add("com_trajectory_fake", _com_trajectory_fake);
        _logger->add("foot_trajectory_fake", _foot_trajectory_fake.translation());
        
        _logger->add("com_trajectory", _com_trajectory);
        _logger->add("foot_trajectory", _foot_trajectory.translation());
        
        
        _logger->add("time", _internal_time);
        
        _logger->add("ft_left", _current_pose_ROS.get_ft_sole(robot_interface::Side::Left));
        _logger->add("ft_right", _current_pose_ROS.get_ft_sole(robot_interface::Side::Right));

        _logger->add("landed_left", static_cast<int>(_current_phase_left));
        _logger->add("landed_right",  static_cast<int>(_current_phase_right));
    
        _logger->add("zmp_ref", _zmp_window_y.coeff(0));
        _logger->add("window_tot", _zmp_window_y);
       
        _logger->add("com_y", _com_y);
        _logger->add("u", _u);
        _logger->add("zmp", _MpC_lat->_C_zmp*_com_y);
        
//         _logger->add("flag_impact", _flag_impact);
        
        _logger->add("steep_coeff", _steep_coeff);
//         _logger->add("alpha", alpha);
        _logger->add("q1_cmd", _q1);
        _logger->add("q1_temp", _q1_temp);
        
        
//         _logger->add("q2_sensed", q2);
        _logger->add("q1_sensed", sense_q1());
        _logger->add("q1_max", _q1_max);
        _logger->add("q1_min", _q1_min);
        _logger->add("q1_start", _q1_start);

        _logger->add("vel_q1", _vel_q1);
        _logger->add("steepness", _steep_coeff);
        
       
        
        _logger->add("delta_com", delta_com);
//         _logger->add("zmp_x", calc_zmp_x(delta_com.coeff(0)));
        
        _logger->add("switch_cmd", _cmd_switch);
        _logger->add("current_spatial_zmp_y", _current_spatial_zmp_y);
        _logger->add("current_spatial_zmp_y_cmd", _current_spatial_zmp_y_cmd);
        
        _logger->add("left_foot_to_com", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left));
        _logger->add("right_foot_to_com", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right));
        
        _logger->add("initial_com_to_ankle", _initial_com_to_ankle);
    
        if (_event == Event::STOP)
        {
            _logger->add("stopped_received", 1);
        }
        else
        {
            _logger->add("stopped_received", 0);
        }
        
        _logger->add("started", _started);
        
        if (_event == Event::IMPACT)
        {
            _logger->add("impact_detected", 1);
        }
        else
        {
            _logger->add("impact_detected", 0);
        }
        
        _logger->add("zmp_stab", _zmp_ref);
        
        _logger->add("starting_foot_pos", _starting_foot_pos.translation());
        _logger->add("starting_com_pos", _starting_com_pos);
        
        _poly_step.log(_logger);
       
        _logger->add("step_counter", _step_counter);
        _logger->add("distance_feet", _distance_feet);
        _logger->add("current_world_to_com", _current_world_to_com);
        _logger->add("com_final_position",  _poly_com.get_com_final_position());
        _logger->add("com_initial_position",  _poly_com.get_com_initial_position());
        
//         tilt_x_meas();
        _logger->add("initial_com_position", _initial_com_position);
        _logger->add("final_com_position", _final_com_position);
        
        _logger->add("initial_sole_position", _initial_sole_pose.translation());
        _logger->add("final_sole_position", _final_sole_pose.translation());
                
        _logger->add("cond_q", _cond_q);
        _logger->add("cond_step", _cond_step);
        
//         _logger->add("cp", cp);
        
        send_com(_com_trajectory);
        
//         send_step(_foot_trajectory);
//         
        send_step_right(right_foot_trajectory);
        send_step_left(left_foot_trajectory);
        
        send_zmp(_zmp_ref);
//         send_waist(_waist_trajectory);
        
//         std::cout << "current_side: " << _current_side << std::endl;
        
        /* publish starting message */
        std_msgs::Bool _started_cmd;
        _started_cmd.data = _started;
        _switch_walk_pub.publish(_started_cmd);
        
//     }
    
    //burn impact event
    if (_event == Event::IMPACT)
    {
        _event = Event::EMPTY;
    }
}
    
bool virtualConstraintsNode::ST_idle(double time)
{
    _initial_com_position = _final_com_position; // the reason is, I get as initial position when in IDLE the final position computed (in computestep)
//     _initial_com_position = _poly_com.get_com_initial_position();
//     _initial_com_position(0) =  _current_pose_ROS.get_com().coeff(0);
    _initial_com_position_fake = _final_com_position_fake;
    
    _initial_sole_pose = _current_pose_ROS.get_sole_tot(_current_side);
//     _initial_sole_pose = _final_sole_pose;
//     _final_sole_pose = _initial_sole_pose;
//     _final_com_position = _initial_com_position;
    _initial_waist_pose = _current_pose_ROS.get_waist();
    /*-------------------------------------------------------------*/
    
    _poly_step.set_foot_initial_pose(_initial_sole_pose);
    _poly_step.set_foot_final_pose(_final_sole_pose);
    
    // ---------------------------------------------
    _poly_step.set_starTime(time);
    _poly_step.set_endTime(time+10);
    
//     _poly_step.set_starTime(_q1);
//     _poly_step.set_endTime(_q1+10);// TODO more or less work so that everything is parametrized as q !!!!!!!!!!!!!
    
    
    _poly_step.set_step_clearing(0);
    
    // ---------------------------------------------
    _poly_com.set_com_initial_position(_initial_com_position);
//     _poly_com.set_com_final_position(_final_com_position);


    // ---------------------------------------------
    
    _poly_com.set_starTime(time);
    _poly_com.set_endTime(time+10);
    
    return 1;
};

        
bool virtualConstraintsNode::ST_walk(double time, Step step_type)
{

//     _com_trajectory(1) = 0; /* HACK! to keep the CoM in the middle of the walk*/
    
//     _initial_com_position = _current_pose_ROS.get_com();
    _initial_com_position = _final_com_position;
//     _initial_com_position = _com_trajectory;
    _initial_com_position_fake = _final_com_position_fake;
//     std::cout << "com_traj: " << _com_trajectory.transpose() << std::endl; // trajectory planned by virtual_constraints_node
//     std::cout << "com_computed: " << _final_com_position.transpose() << std::endl; // trajectory computed ahead by virtual_constraints_node
//     std::cout << "com_cartesio: " << _current_pose_ROS.get_com().transpose() << std::endl; // trajectory computed by cartesi/o
//     std::cout << "com_realized: " << _current_pose_ROS.get_world_to_com().transpose() << std::endl; // trajectory realized by openSoT
    
    _initial_sole_pose = _current_pose_ROS.get_sole_tot(_current_side);
    _initial_waist_pose = _current_pose_ROS.get_waist();
    
    _final_sole_pose = _initial_sole_pose;
    _final_com_position = _initial_com_position;
    _initial_waist_pose = _final_waist_pose;
    
    _final_com_position_fake = _initial_com_position_fake;
    
    compute_step(step_type);
    
    _poly_step.set_foot_initial_pose(_initial_sole_pose);
    _poly_step.set_foot_final_pose(_final_sole_pose);
    
    // ---------------------------------------------
    _poly_step.set_starTime(time);
    _poly_step.set_endTime(time + _initial_param.get_duration_step());
    
//     _poly_step.set_starTime(_q1);
//     _poly_step.set_endTime(_q1 + _q1_max); // TODO more or less work so that everything is parametrized as q !!!!!!!!!!!!!
    _poly_step.set_step_clearing(_initial_param.get_clearance_step());
    
    // ---------------------------------------------
//     _initial_com_position(1) = 0;
    _poly_com.set_com_initial_position(_initial_com_position);
//     _poly_com.set_com_final_position(_final_com_position);
    
    _poly_com.set_starTime(time);
    _poly_com.set_endTime(time + _initial_param.get_duration_step());
    

    

    return 1;
};

bool virtualConstraintsNode::compute_step(Step step_type)
{
                
                double q1_max_new;
                    
                q1_max_new = _q1_max;
                Eigen::Matrix2d R_steer;
                
                
               
                
                /* TODO refactor: this is needed for the steering and the length step */
                
                double theta = _theta_steer; // change heading
                R_steer << cos(theta), -sin(theta),
                                sin(theta), cos(theta);
                if (_step_counter >= 4 && _step_counter < 5)       /*Left*/
                {
                    _q1_max = 0.05; /*0.001*/ 
                    q1_max_new = _q1_max; // change step length
                    double theta = _theta_steer; // change heading
                    R_steer << cos(theta), -sin(theta),
                                    sin(theta), cos(theta);
                }
                else if (_step_counter >= 5 && _step_counter < 6)   /*Right*/
                {
                    _q1_max = 0.05; /*0.02*/ 
                    q1_max_new = _q1_max; // change step length
                    double theta = _theta_steer; // change heading
                    R_steer << cos(theta), -sin(theta),
                                    sin(theta), cos(theta);
                }
                else if (_step_counter >= 6 && _step_counter < 7)   /*Left*/
                {
                    _q1_max = 0.05;
                    q1_max_new = _q1_max; // change step length
                    double theta = _theta_steer; // change heading
                    R_steer << cos(theta), -sin(theta),
                                    sin(theta), cos(theta);
                }
                else if (_step_counter >= 7 && _step_counter < 8)  /*Right*/
                {
                    _q1_max = 0.05;
                    q1_max_new = _q1_max; // change step length
                    double theta = _theta_steer; // change heading
                    R_steer << cos(theta), -sin(theta),
                                    sin(theta), cos(theta);
                }
                else if (_step_counter >= 8 && _step_counter < 9)   /*Left*/
                {
                    _q1_max = 0.05;
                    q1_max_new = _q1_max; // change step length
                    double theta = _theta_steer; // change heading
                    R_steer << cos(theta), -sin(theta),
                                    sin(theta), cos(theta);
                }
                else if (_step_counter >= 9 && _step_counter < 10) /*Right*/
                {
                    _q1_max = 0.05;
                    q1_max_new = _q1_max; // change step length
                    double theta = _theta_steer; // change heading
                    R_steer << cos(theta), -sin(theta),
                                    sin(theta), cos(theta);
                }
                else if (_step_counter >= 10 && _step_counter < 15)
                {
                    _q1_max = 0.05;
                    q1_max_new = _q1_max; // change step length
                    double theta = _theta_steer; // change heading
                    R_steer << cos(theta), -sin(theta),
                                    sin(theta), cos(theta);
                }
                else
                {
                    _q1_max = 0.05;
                    q1_max_new = _q1_max;
                    double theta = 0;
                    R_steer << cos(theta), -sin(theta),
                                sin(theta), cos(theta); 
                }
                /*----------------generate q1-------------------------*/
                double q1 = (q1_max_new - _q1_min);
                

                _steep_coeff = (q1_max_new - _q1_min)/_step_duration;
                /*----------------------------------------------------*/
                
                std::cout << "q1_max: " << q1_max_new << std::endl;
                std::cout << "q1_min: " << _q1_min << std::endl;
                std::cout << "angle: " << q1 << std::endl;
               
                Eigen::Vector2d disp_com; // displacement in the xy plane
                Eigen::Vector2d disp_com_rot; // displacement in the xy plane
                disp_com << fabs(_current_pose_ROS.get_distance_ankle_to_com(_current_side).z()) * tan(q1), 0; // displacement of com in x
                
                disp_com_rot = R_steer * disp_com; // angle steering
                std::cout << "disp_com_rot: " << disp_com_rot.transpose() << std::endl;
                
                _final_com_position.head(2) = _initial_com_position.head(2) + disp_com_rot;
                _final_com_position_fake.head(2) = _initial_com_position_fake.head(2) + disp_com;

                
                
                _final_sole_pose.translation() = _current_pose_ROS.get_sole_tot(_other_side).translation() + 2 * (_final_com_position - _current_pose_ROS.get_sole_tot(_other_side).translation());  // get final step given the displacement vector

                /* TODO refactor: this is needed for the steering (orientation) */
                double theta_heading;
                /* orientation */
                
                if (_step_counter >= 4 && _step_counter < 15)
                {
                    theta_heading = _theta_steer;
                }
                else
                {
                    theta_heading = 0;
                }
                
                /* Sole */
                _final_sole_pose.linear() = (Eigen::AngleAxisd(theta_heading, Eigen::Vector3d::UnitZ())).toRotationMatrix();
                
                /* Waist */
                _final_waist_pose.linear() = (Eigen::AngleAxisd(theta_heading, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

void virtualConstraintsNode::planner(double time) 
{  
    switch (_current_state)
    {
            case State::STARTING :
                _step_type = Step::HALF;
                ST_walk(time, _step_type);
                break;
        
            case State::WALK :
                _step_type = Step::FULL;
                ST_walk(time, _step_type);
                break;
        
            case State::STOPPING :
                _step_type = Step::FULL;
                 ST_walk(time, _step_type);
                 break;
                
            case State::LASTSTEP :
                _step_type = Step::HALF;
                 ST_walk(time, _step_type);
                 break;
                 
            case State::IDLE :
                _step_type = Step::VOID;
                ST_idle(time);
                break;
            
            default : throw std::runtime_error(std::string("State not recognized"));
    }
}

void virtualConstraintsNode::core(double time)
{
    
//     std::cout << "Entering core with event: " << _event << " during state: " << _current_state << std::endl;
    
    if (_last_event != _event)
    {
        _new_event_time = time;
    }
    
    switch (_event)
    {    
        case Event::IMPACT :
            switch (_current_state)
            {
                case State::IDLE :
                    throw std::runtime_error(std::string("Something wrong. Impact during IDLE"));
                    break;
                   
                case State::WALK :
                    _step_counter++;
                    planner(time);
                    break;
                    
                case State::STARTING :
                    _step_counter++;
                    _previous_state = _current_state;
                    _current_state = State::WALK;
                    planner(time);
                    break;
                    
                case State::STOPPING :
                    _step_counter++;
                    _previous_state = _current_state;
                    _current_state = State::LASTSTEP;
                    _q1_max = 0;
                    planner(time);
                    break;
                    
                case State::LASTSTEP :
                    _step_counter++;
                    _started = 0;
                    _q1_start = _q1_temp;
                    _previous_state = _current_state;
                    _current_state = State::IDLE;
                    planner(time);
                    _steep_coeff = 0;
                    resetter();
                    break;
            }
            break;
        
        case Event::START :
        {
            switch (_current_state)
            {

                case State::IDLE :
                    _previous_state = _current_state;
                    _current_state = State::STARTING;
                    _current_side = _initial_param.get_first_step_side();                    
                    _q1_max = _initial_param.get_max_inclination();
                    _step_counter++;
                    _cycle_counter++;
                    _started = 1;
                    _start_walk = _internal_time + _delay_start;
                    _steep_coeff = (_q1_max - _q1_min)/_step_duration; 
                    planner(time + _delay_start); //_t_before_first_step
                    break;
                    
                default : /*std::cout << "Ignored starting event. Already WALKING" << std::endl; */
                    break;
            }
            break;
        }
        case Event::STOP :
        {
//             _started = 0;
            switch (_current_state)
            {
                case State::IDLE :
                    /* std::cout << "Ignored stopping event. Already in IDLE" << std::endl; */
                    break;
                   
                case State::WALK :
                case State::STARTING :
                    _end_walk = time;
                    _previous_state = _current_state;
                    _current_state = State::STOPPING;
//                     planner(time); //replan as soon as I get the message?
                    break;
                    
                case State::STOPPING :
                    /* std::cout << "Ignored stopping event. Already STOPPING" << std::endl; */
                    break;
            }
            break;
        }
        case Event::EMPTY :
            
            switch (_current_state)
            {
                case State::IDLE :
                    planner(time);
                    break;
                default : 
                    break;
            }
            
            break;
             
        default : 
            throw std::runtime_error(std::string("Event not recognized"));
            break;
    }
}



bool virtualConstraintsNode::initialize(double time) 
{
    straighten_up_action();
    
    /* wait for floating base from gazebo */
    while (!_fb.isReady())
    {
        ros::spinOnce();
        _fb.getPose();
        _fb.getVelocity();
    }
    
    /* add model and robot for foot stabilizer */
    XBot::JointNameMap mapCogimon;
    
    _robot->sense();
    _model->syncFrom(*_robot);
    
    _model->setFloatingBaseState(_fb.getPose(), _fb.getVelocity());
    _model->update();
        

    
    _model->getJointPosition(mapCogimon);
    _model->getCOM(_real_com_pos);
    _model->getCOMVelocity(_real_com_vel);
    
    std::cout << "real com_pos: " << _real_com_pos.transpose() << std::endl;
    std::cout << "cartesio com_pos: " <<_current_pose_ROS.get_com().transpose() << std::endl;
    
    double clearance = _initial_param.get_clearance_step();
    _reset_condition = 0; /*needed for resetting q1*/
    
    _delay_start = 1.5; /*time between instant that received START command is received and instant where robot starts walking. Required for initial lateral shift of the CoM*/
    
    /* current side of the robot */
    _current_side = robot_interface::Side::Double;
//     _current_side = _initial_param.get_first_step_side();
    
    std::cout << "First step: " << _initial_param.get_first_step_side() << std::endl;
    
    /* get stance foot */
    _other_side = (robot_interface::Side)(1 - static_cast<int>(_initial_param.get_first_step_side()));
    

    _initial_sole_y_right = _current_pose_ROS.get_sole(robot_interface::Side::Right).coeff(1);
    _initial_sole_y_left = _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1);
    
    _initial_zmp_y_right = _initial_sole_y_right + _initial_param.get_indent_zmp();
    _initial_zmp_y_left =  _initial_sole_y_left - _initial_param.get_indent_zmp();
    
    _initial_com_position = _current_pose_ROS.get_com();
    _initial_sole_pose = _current_pose_ROS.get_sole_tot(_initial_param.get_first_step_side());
    _final_sole_pose = _initial_sole_pose;
    _final_com_position = _initial_com_position;

    _initial_com_position_fake = _initial_com_position;
    _final_com_position_fake = _final_com_position;
    
    _com_trajectory  = _initial_com_position;
    
    /* GET RIGHT AND LEFT POSITION OF FOOT */
    _right_foot_position = _current_pose_ROS.get_sole_tot(robot_interface::Side::Right);
    _left_foot_position = _current_pose_ROS.get_sole_tot(robot_interface::Side::Left);
    
    /*just to run it once, heat up the process*/
    planner(time); /*void run of planner*/
    Eigen::Vector3d foot_trajectory;
    Eigen::Vector3d fake_pose;
    fake_pose.setZero();
    
    /*just to run it once, heat up the process*/
    foot_trajectory = compute_swing_trajectory(fake_pose, fake_pose, 0, 0, 0, time);

    /* comy */
    double Ts = _dt; /*window resolution*/
    double T = _initial_param.get_duration_preview_window(); /*window length for MpC*/
    double dt = _dt; /*rate of ros*/
    _t_before_first_step = 0; /*preparation time in the first step for the com to swing laterally before stepping */
    
    _q1_max = _initial_param.get_max_inclination(); /*max angle of inclination of the robot*/
    _q1_min = sense_q1(); /* min angle of inclination, starting inclination of the robot*/
    
    _q1_start = 0; /*starting q1, incremented every time that the robot stops (when restart, it needs it)*/
    _step_duration = _initial_param.get_duration_step(); 

    _steep_coeff = 0; //(_q1_max - _q1_min)/_step_duration; /*at first is 0, setted when START command is received*/

    _nominal_full_step = fabs(4* fabs(_current_pose_ROS.get_distance_ankle_to_com(_initial_param.get_first_step_side()).z()) * tan(_q1_max));
    _nominal_half_step = fabs(2* fabs(_current_pose_ROS.get_distance_ankle_to_com(_initial_param.get_first_step_side()).z()) * tan(_q1_max));
    
//     std::cout << "Position foot RIGHT: " << "x: " << _current_pose_ROS.get_sole(robot_interface::Side::Right).coeff(0) 
//                                          << "y: " << _current_pose_ROS.get_sole(robot_interface::Side::Right).coeff(1) 
//                                          << "z: " << _current_pose_ROS.get_sole(robot_interface::Side::Right).coeff(2) << std::endl;
//     std::cout << "Position foot LEFT: "  << "x: " << _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(0) 
//                                          << "y: " << _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1) 
//                                          << "z: " << _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(2) << std::endl;
    std::cout << "Start walk time: " << _initial_param.get_start_time() << " s" <<  std::endl;
    std::cout << "Lean forward: " << _initial_param.get_lean_forward() << " m (Initial angle: " << _initial_q1 << " rad)" << std::endl;
    std::cout << "Step length: " << _nominal_full_step << " m (Max angle of inclination: " << _q1_max << " rad)" <<  std::endl;
    std::cout << "Step duration: " << _step_duration << " s" << std::endl;
    std::cout << "Double stance: " <<  _initial_param.get_double_stance() << " s" << std::endl;
    std::cout << "Steepness: " << _steep_coeff <<  std::endl;
    std::cout << "Real impacts: " << _initial_param.get_switch_real_impact() <<  std::endl;
    std::cout << "ZMP width correction: " << - _initial_param.get_indent_zmp() << " --> ZMP Right: " << _initial_zmp_y_right << " and ZMP Left: " << _initial_zmp_y_right << std::endl;

    _com_y << _initial_com_position(1), 0, 0; /*com trajectory used by mpc: pos, vel, acc*/

    //MpC

    Eigen::MatrixXd Q(1,1);
    Eigen::MatrixXd R(1,1);
    
    Q << _initial_param.get_MPC_Q(); 
    R << _initial_param.get_MPC_R();
    
    _MpC_lat = std::make_shared<item_MpC>(_initial_height, Ts, T, Q, R); /* initializing the MPC for the lateral motion of the CoM */
    
    generate_starting_zmp();
    
    _spatial_window_preview.resize(1);
    _zmp_window_y.resize(1);
    _zmp_window_y.setZero();
    _zmp_window_t.setZero();
    _com_y.setZero();
    _u.setZero();
    

    _init_completed = 1; /* to run initialization only once*/
    
    _last_event = Event::EMPTY; /*initializing the last event to EMPTY*/
    _event = Event::EMPTY; /*initializing the event to EMPTY*/

    
    /* steer */
    _theta_steer = 0; //M_PI/10 // M_PI/3//M_PI/8;
//     _first_step_steer = 3;
//     _last_step_steer = 5;
    
    
    /* set gains of foot stabilizer */
    
//     _kp_foot_stab << 0.0001, 0.0, 0.0;
//     _kd_foot_stab << 0.015, 0.0, 0.0;
    
    _stab->setKp(controller_gains.getKp());
    _stab->setKd(controller_gains.getKd());
    
    /* get ORIENTATION soles */
    _base_R_l_sole = (_current_pose_ROS.get_sole_tot(robot_interface::Side::Right)).linear();
    _base_R_r_sole = (_current_pose_ROS.get_sole_tot(robot_interface::Side::Left)).linear();
        
    
       /*fake cycle*/
//     planner(0);
//     impact_routine();
//     core(0);
    commander(_internal_time);
    

    
    _start_walk =_initial_param.get_start_time(); /* getting start walking from the user param */
   
    _logger->add("zmp_starting", _zmp_starting);
    _logger->add("_y", _spatial_zmp_y);
    std::cout << "Initialization complete." << std::endl;
    
    
    return 1;
                
}

void virtualConstraintsNode::spatial_zmp(double& current_spatial_zmp_y, Eigen::VectorXd &spatial_window_preview, double length_preview_window, double dx, Step type_step)
{
    /**
     * @brief spatial ZMP_y computed at each control cycle
     * 
     **/
    
    double dt = _dt;
    
    _q1_sensed_old = _q1_sensed;
    _q1_sensed = sense_q1();
    
    
    double alpha_old = (_q1_sensed_old - _q1_min)/(_q1_max - _q1_min);
    double alpha = (_q1_sensed - _q1_min)/(_q1_max - _q1_min);
    
//     double vel_alpha = (alpha - alpha_old)/dt;
//     if (alpha < alpha_old)
//         alpha = alpha_old;
//     
    if (alpha < 0)
        alpha = 0;
    
//     double alpha_cmd_old = (_q1_old - 0)/(_q1_max - 0);
//     double alpha_cmd = (_q1 - 0)/(_q1_max - 0);
    
    _logger->add("alpha_sensed", alpha);
//     _logger->add("alpha_cmd", alpha_cmd);
//     _logger->add("vel_alpha", vel_alpha);
    
//     if (fabs(alpha - alpha_old) >= 0.8)
//     {
//         _switched = -_switched;
//     }
    
    
    robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
    double zmp_y;
    
    if (other_side == robot_interface::Side::Right)
    {
        zmp_y = _initial_zmp_y_right;
    }
    else if (other_side == robot_interface::Side::Left)
    {
        zmp_y = _initial_zmp_y_left;
    }

    _logger->add("zmp_y_rec", zmp_y);
    
    if (alpha >= 0)
    {
        if (alpha <= 1)
        {
            current_spatial_zmp_y = zmp_y;
        }
    
    
    }
    
    // ------------------------------------------------
    // window from zmp spatial
    // ------------------------------------------------

    
    double foot_position = _initial_pose.get_sole(other_side).coeff(0);
//     double foot_position = _starting_foot_pos.translation().coeff(0); // TODO
    
    _switched_prev = 1;
    
    
    double size_window = _initial_param.get_duration_preview_window() / dt;
    
    
    double velocity_step_F = _nominal_full_step / _initial_param.get_duration_step();
    double velocity_step_H = _nominal_half_step / _initial_param.get_duration_step();
    double length_preview_window_H = _initial_param.get_duration_step() * velocity_step_H; //_steep_coeff
    double length_preview_window_F = _initial_param.get_duration_step() * velocity_step_F; //_steep_coeff
    double dx_H =  velocity_step_H * dt;
    double dx_F =  velocity_step_F * dt;
    
    double first_max_space;
    double n_steps_future;
    int size_step;
    double length_step;
    Eigen::VectorXd max_spaces;
    
    

    /**
     * generate previewed position of steps in the future
     **/
    /* TODO change here because there are no more FULL and HALF steps */
    if (type_step == Step::FULL)
    {
        length_step = _nominal_full_step;
        
        first_max_space = fabs(length_step) + foot_position;
        size_step = round(fabs(length_step)/dx);
        n_steps_future = (double)size_window / (double)size_step;

        max_spaces.resize(ceil(n_steps_future) + 1); //because 
        
        
        int j = 1;
        for (int i = 0; i < max_spaces.size(); i++) //fill max_spaces with all the max spaces computed in advance
        {
            max_spaces[i] = foot_position + (j * fabs(length_step));
            j++;
        }
    }
    else if (type_step == Step::HALF)
    {
            length_step = _nominal_half_step;
            double lenght_full_step = 2 * length_step;
            
            first_max_space = fabs(length_step) + foot_position;
            size_step = round(fabs(length_step)/dx);
            double size_full_step = round((length_step*2)/dx_F);
            n_steps_future = (double)(size_window - size_step) / (double)(size_full_step) + 1;

            max_spaces.resize(ceil(n_steps_future) + 1);
            
            max_spaces[0] = first_max_space;
            
            for (int i = 1; i < max_spaces.size(); i++)
            {
                max_spaces[i] = first_max_space + (i * fabs(lenght_full_step));
            }
    }
    else if  (type_step == Step::VOID)
    {
        length_step = 0;
    }
    
//    // last step =========================================================
    if (_current_state == State::STOPPING)
    {
        n_steps_future = 2;
        double length_half_step = _nominal_half_step;
        double lenght_full_step = 2 * length_half_step;

        double current_max_space = fabs(lenght_full_step) + foot_position;
        double last_max_space = current_max_space + fabs(length_half_step);
        
        max_spaces.resize(2);
        max_spaces[0] = current_max_space;
        max_spaces[1] = last_max_space;
        
    } 
    else if (_current_state == State::LASTSTEP)
    {
        n_steps_future = 1;
        double current_max_space = fabs(_nominal_half_step) + foot_position;
        
        max_spaces.resize(1);
        max_spaces[0] = current_max_space;
        
    }
//    // ==================================================================

    spatial_window_preview.resize(size_window);
    int j = 0;
    double space = alpha * fabs(length_step) + foot_position;
    
    

    double side_value = current_spatial_zmp_y;
    int n_step_preview = 0;
    if (type_step != Step::VOID)  /*also working, if I want to take out Step::VOID -->     if (_current_state != State::IDLE)*/
    {
        while (j < spatial_window_preview.size())
        {
            if (type_step == Step::HALF && n_step_preview >= 1)
            {
                dx = dx_F;// so that I only have the first step as half step, after the dx return to nominal value (dx_F) so full step
            }
            
            if (_current_state == State::STOPPING && n_step_preview >=1)
            {
                dx = dx_H;// so that I have the last step as half step
            }
            
            if (n_step_preview < max_spaces.size())
            {
                if (space <= max_spaces[n_step_preview])
                {
                    // for each max spaces choose the correct side of the lateral zmp
                    spatial_window_preview[j] = side_value * _switched_prev;
                    space = space + dx;
                    j++; 
                }
                else if (space > max_spaces[n_step_preview])
                {
                    n_step_preview++;
                    _switched_prev = - _switched_prev;
                }
            }
            else
            {
                spatial_window_preview[j] = 0;
                space = space + dx;
                j++;
            }

        }
    }
    else 
    {
        spatial_window_preview.setZero();
    }
    
    _logger->add("window_preview", spatial_window_preview);
}


Eigen::Vector3d virtualConstraintsNode::get_com_velocity()
{
    double dt = _dt;
    Eigen::Vector3d com_vel = (_com_trajectory - _previous_com_trajectory)/dt;  
    
    return com_vel;
    
}

Eigen::Vector3d virtualConstraintsNode::sense_com_velocity()
{
    double dt = _dt;
    Eigen::Vector3d com_pos = _current_pose_ROS.get_com();
    Eigen::Vector3d com_vel = (com_pos - _previous_com_pos)/dt;
    _previous_com_pos = _current_pose_ROS.get_com();
    
    return com_vel;
    
}

Eigen::Vector3d virtualConstraintsNode::sense_foot_velocity()
{
    double dt = _dt;
    Eigen::Vector3d foot_pos = _current_pose_ROS.get_sole(_current_side);
    Eigen::Vector3d foot_vel = (foot_pos - _previous_foot_pos)/dt;
    _previous_foot_pos = _current_pose_ROS.get_sole(_current_side);
    
    return foot_vel;
}

Eigen::Vector3d virtualConstraintsNode::get_foot_velocity()
{
    double dt = _dt;
    Eigen::Vector3d foot_vel = (_foot_trajectory.translation() - _previous_foot_trajectory.translation())/dt;  
    
    return foot_vel;
}



double virtualConstraintsNode::q_handler()
{
    double dt = _dt;
     
    
    if (_started == 1 && _internal_time >= +_start_walk)
    {
double cond_q;
        if (_q1_min <= _q1_max)
        {
            cond_q = (sense_q1() >= _q1_max);
        }
        else if (_q1_min > _q1_max)
        {
            cond_q = (sense_q1() <= _q1_max);
        }
        if (cond_q)
        {
            _q1_temp = _q1_temp;
        }
        else
        {
//         _q1_temp = _q1_start + _steep_coeff*(_internal_time - _start_walk); // basically q = a*t
            _q1_temp = _q1_temp + _steep_coeff*(dt); // basically q = a*t//ver2
        }
    }
//         _q1_temp = _q1_temp + _steep_coeff*(dt); // basically q = a*t//ver2

}



void virtualConstraintsNode::resetter()
{
    generate_starting_zmp();
    _current_world_to_com = _current_pose_ROS.get_world_to_com();
    _current_world_to_com(0) = _current_pose_ROS.get_world_to_com().coeff(0) - _current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2)*tan(_initial_param.get_max_inclination());
//     straighten_up_action();
}

void virtualConstraintsNode::generate_starting_zmp()
{
    
    double initial_step_value;
    double dt = _dt;
    /* get first step size to choose the side of initial swing */
    if (_initial_param.get_first_step_side() == robot_interface::Side::Right)
    {
        initial_step_value = _foot_pos_y_left - _initial_param.get_indent_zmp();
    }
    else if (_initial_param.get_first_step_side() == robot_interface::Side::Left)
    {
        initial_step_value = _foot_pos_y_right + _initial_param.get_indent_zmp();
    }
        
//     int sign_first_stance_step = (_current_pose_ROS.get_sole(_other_side).coeff(1) > 0) - (_current_pose_ROS.get_sole(_other_side).coeff(1) < 0);
     //_current_pose_ROS.get_sole(_other_side).coeff(1) - sign_first_stance_step * _initial_param.get_indent_zmp();
    int window_size = round(_MpC_lat->_window_length / dt);
    _zmp_starting.resize(window_size);
    _zmp_starting.setZero();

    int chunck_size = round((_initial_param.get_duration_step())/dt);
    Eigen::VectorXd zmp_chunck(chunck_size);
    zmp_chunck = initial_step_value * zmp_chunck.setOnes();

    int first_chunck_pos = round(_delay_start/dt);

    _zmp_starting.segment(first_chunck_pos, chunck_size) << zmp_chunck;
    _zmp_starting.segment(first_chunck_pos+chunck_size+1, chunck_size) << - zmp_chunck;
//     _zmp_starting.segment(first_chunck_pos+2*chunck_size+1, chunck_size) << zmp_chunck;
}
