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
//         initialize_cmd_fake_q1(); //TODO
        
        std::string this_node_name = ros::this_node::getName();
        _logger = XBot::MatLogger::getLogger("/tmp/" + this_node_name);
        ros::NodeHandle n;
        
        _step_counter = 0;
        get_param_ros(); //initial parameters from ros
        
        _initial_pose = _current_pose_ROS;
        _initial_step_y = _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1);
        

        _poly_com.set_com_initial_position(_current_pose_ROS.get_com());
        _q1_state = sense_q1();
        
        _terrain_heigth =  _current_pose_ROS.get_sole(_current_side).coeff(2);

//      prepare subscriber node for commands
        _switch_srv = n.advertiseService("/virtual_constraints/walk_switch", &virtualConstraintsNode::cmd_switch, this);
             
//      prepare subscriber node
        _q1_sub = n.subscribe("/q1", 10, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 10); /*publish to /cartesian/com/reference*/
        
        _sole_pubs[robot_interface::Side::Left] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/l_sole/reference", 10); /*publish to /l_sole/reference*/
        _sole_pubs[robot_interface::Side::Right] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 10); /*publish to /r_sole/reference*/
        
//        // required for STABILIZER
        _zmp_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com_stabilizer/zmp/reference", 10);
  
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
        
        straight_com(0) = _current_pose_ROS.get_sole(_current_side).coeff(0) + _initial_param.get_lean_forward();
//         straight_com(1) = _current_pose_ROS.get_sole(_current_side).coeff(1); 
        straight_com(2) = _initial_param.get_crouch();
        
        /*TODO PUT DEFAULT POSITION*/
        _poly_com.set_com_initial_position(straight_com);
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
        _initial_q1 = sense_q1();

        _initial_height = fabs(_current_pose_ROS.get_com().coeff(2) - _current_pose_ROS.get_sole(_current_side).coeff(2));
        _initial_com_to_ankle = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
        //exit
        return 0;
}


bool virtualConstraintsNode::cmd_switch(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
    {
        if (req.data)
        {
            _last_event = _event;
            _event = Event::START;
            res.message = "Started";
            res.success = true;
        }
        else
        {
            _last_event = _event;
            _event = Event::STOP;
            res.message = "Stopped";
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

double virtualConstraintsNode::sense_qlat()
{
    double q1_lat, q2_lat;
//     _current_pose_ROS.sense(); 
    if (_current_side == robot_interface::Side::Double)
    {
        Eigen::Vector3d left_ankle_to_com, right_ankle_to_com;
        
        left_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left);
        right_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right);
        
        q1_lat = atan(left_ankle_to_com(1)/left_ankle_to_com(2));
        q2_lat = atan(right_ankle_to_com(1)/right_ankle_to_com(2));
    }
    else
    {
        Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
        robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
        
        swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
        stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
        
        q1_lat = atan(stance_ankle_to_com(1)/stance_ankle_to_com(2));
        q2_lat = atan(swing_ankle_to_com(1)/swing_ankle_to_com(2));
    }
    
    _logger->add("q_lateral_stance", q1_lat);
    _logger->add("q_lateral_swing", q2_lat);
    
    
    _logger->add("q_lateral_left", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(1)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
    _logger->add("q_lateral_right", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(1)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
    
    return q1_lat;
}


double virtualConstraintsNode::sense_q1()
{   
//         _current_pose_ROS.sense(); // TODO put back?
    double q1, q2;
    Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
    
    if (_current_side == robot_interface::Side::Double)
    {
        Eigen::Vector3d left_ankle_to_com, right_ankle_to_com;
        
        left_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left);
        right_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right);
            
    q1 = atan(left_ankle_to_com(0)/left_ankle_to_com(2));
    q2 = atan(right_ankle_to_com(0)/right_ankle_to_com(2));
    
    }
    else
    {
    Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
    
    robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
    swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
    stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
        
    q1 = atan(stance_ankle_to_com(0)/stance_ankle_to_com(2));
    q2 = atan(swing_ankle_to_com(0)/swing_ankle_to_com(2));
    
    }

    return q1;
}  

double virtualConstraintsNode::sense_q1(double& q2)
{   
//         _current_pose_ROS.sense(); // TODO put back?
    double q1;
    Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
    
    if (_current_side == robot_interface::Side::Double)
    {
        Eigen::Vector3d left_ankle_to_com, right_ankle_to_com;
        
        left_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left);
        right_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right);
            
        q1 = atan(left_ankle_to_com(0)/left_ankle_to_com(2));
        q2 = atan(right_ankle_to_com(0)/right_ankle_to_com(2));
    }
    else
    {
        Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
        
        robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
    //     swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
        stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
            
        q1 = atan(stance_ankle_to_com(0)/stance_ankle_to_com(2));
        q2 = atan(swing_ankle_to_com(0)/swing_ankle_to_com(2));
    
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
    bool cond;
    bool cond_q;
    
    if (_initial_param.get_walking_forward())
    {
        cond_q = (sense_q1() >= _q1_max);
        
        if (_current_state != State::STOPPING)
        {
            cond = fabs(_current_pose_ROS.get_sole(_current_side).coeff(0) - _initial_pose.get_sole(_current_side).coeff(0)) >  0.05;
        }
        else
        {
            cond = 1;
        }
    }
    else
    {
        if (_current_state != State::IDLE)
        {
            cond = _internal_time > (_start_walk + 0.2)  && _impact_cond > 0.2;
        }
        else
        {
            cond = 0;
        }
        
//         cond_q = 1;
//         if (_step_counter > _initial_param.get_max_steps()-1)
//             cond = 0; 
    }
    
    bool cond_step = fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-3;
//     bool cond_com = fabs(fabs(_current_pose_ROS.get_com().coeff(0)) - fabs(_poly_com.get_com_final_position().coeff(0))) <= 1e-3;
    //     bool cond_q = fabs(sense_q1() - _q1_max) <= 1e-2; // so i wait the sensed tilt to be the tilt i want
//     bool cond_q = (sense_q1() >= _q1_max); // same as above
    
    _logger->add("cond_q", cond_q);
    _logger->add("cond_step", cond_step);
    _logger->add("cond", cond);
    if (cond_step && cond_q && cond) //1e-4
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
        real_impacts();
    }
    else
    {
//         _flag_impact =  real_impacts();
        fake_impacts();
    }
}

int virtualConstraintsNode::impact_routine()                
    {
            
            if (impact_detector())
            {
                _last_event = _event;
                _event = Event::IMPACT; // event impact detected for core()
//                 _current_pose_ROS.sense();
               
                robot_interface::Side last_side = _current_side;
//                 std::cout << "Last side: " << last_side << std::endl;
               _initial_pose = _current_pose_ROS;
            // ----------------------------------
                _q1_min = -sense_q1(); /*HACK*/
            // -------------------------------
                _current_side = robot_interface::Side::Double;

                std::cout << "Impact! Current side: " << _current_side << std::endl;
                
                _current_pose_ROS.get_sole(_current_side);
                // ----------------------------------------------------
                _poly_com.set_com_initial_position(_initial_pose.get_com());     
                // ----------------------------------------------------
//                 std::cout << "before: " << sense_q1() << std::endl;
                
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
                
                std::cout << "State changed. Current side: " << _current_side << std::endl;
            
//                 _q1_min = sense_q1();
//                 std::cout << "after: " << sense_q1() << std::endl;
                return 1;
            }
            else
            {
                return 0;
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
    double dt = 0.01;
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
//         
        if (impact_routine())
        {    
//             _q1_min = sense_q1();
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
    
void virtualConstraintsNode::send_zmp(Eigen::Vector3d zmp_command)
    {
        geometry_msgs::PoseStamped cmd_zmp;
        tf::pointEigenToMsg(zmp_command, cmd_zmp.pose.position);
        
        _zmp_pub.publish(cmd_zmp);
    }
    
Eigen::Affine3d virtualConstraintsNode::compute_trajectory(Eigen::Affine3d T_i, Eigen::Affine3d T_f,
                                                double clearance,
                                                double start_time, double end_time, 
                                                double time
                                                )
    {
//     XBot::Utils::FifthOrderPlanning(0, 0, 0, 1, t_start, t_end, time, tau, dtau, ddtau);
    
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
        
        double dt = 0.01; //TODO take it out from here
     
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
        
        spatial_zmp(_current_spatial_zmp_y, _spatial_window_preview, length_preview_window, dx, _step_type);
        _zmp_window_y = _spatial_window_preview;

        if (_current_state == State::STARTING)
        {
// //             this is for the beginning, to move the com before the step
            if (_internal_time < _start_walk)
            {
                _zmp_starting.head(_zmp_starting.size()-1) = _zmp_starting.tail(_zmp_starting.size()-1);
                _logger->add("zmp_start", _zmp_starting);
                _zmp_window_y = _zmp_starting;
            }
        }
//             else
//             {
//                 _zmp_window_y = _spatial_window_preview;
//             }
//         }
//         else if (_current_state == State::STARTING || _current_state == State::WALK ||  _current_state == State::STOPPING ||  _current_state == State::LASTSTEP )

        
        _u = _MpC_lat->_K_fb * _com_y + _MpC_lat->_K_prev * _zmp_window_y;
        _MpC_lat->_integrator->integrate(_com_y, _u, dt, _com_y);
         
        return _com_y;
}

void virtualConstraintsNode::commander(double time)
{
  
        //// send com sagittal
//         Eigen::Vector3d com_trajectory;
        Eigen::Vector3d delta_com;
        _previous_com_trajectory = _com_trajectory;
        _com_trajectory = _poly_com.get_com_initial_position();
                robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
                Eigen::Vector3d com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(other_side);
//                 Eigen::Vector3d com_to_ankle_distance = _initial_com_to_ankle;
        double alpha;
    if (_initial_param.get_use_poly_com())
    {
            if (_initial_param.get_walking_forward())
            {
                _com_trajectory = compute_swing_trajectory(_poly_com.get_com_initial_position(), _poly_com.get_com_final_position(), 0, _poly_com.get_starTime(), _poly_com.get_endTime(), time);

            }
            
    }
    else
    {
            if (_initial_param.get_walking_forward())
            {

                
                
                _com_trajectory = _current_pose_ROS.get_com();
                
                if (_current_state == State::STARTING || _current_state == State::LASTSTEP)
                {
                    delta_com << - com_to_ankle_distance.z() * tan(_q1), 0, 0;
//                     std::cout << delta_com.coeff(0) << std::endl;
//                     double com_final = _poly_com.get_com_initial_position().coeff(0) - (com_to_ankle_distance.z() * tan(_q1_max));
//                     alpha = (com_final -  _poly_com.get_com_initial_position().coeff(0))/_initial_param.get_duration_step();
//                     std::cout << alpha * _q1/_steep_coeff << std::endl;
//                     std::cout << "--------_" << std::endl;
//                     delta_com << alpha * _q1/_steep_coeff, 0, 0; // the same!!
                    _com_trajectory(0) = _poly_com.get_com_initial_position().coeff(0) + delta_com(0);    
                }
                else if (_current_state == State::WALK || _current_state == State::STOPPING)
                {
                    delta_com << - 2* com_to_ankle_distance.z() * tan(_q1), 0, 0;
//                     std::cout << delta_com.coeff(0) << std::endl;
//                     double com_final = _poly_com.get_com_initial_position().coeff(0) - (2 * com_to_ankle_distance.z() * tan(_q1_max));
//                     alpha = (com_final -  _poly_com.get_com_initial_position().coeff(0))/_initial_param.get_duration_step();
//                     delta_com << alpha * _q1/_steep_coeff, 0, 0;
//                     std::cout << alpha * _q1/_steep_coeff << std::endl;
//                     std::cout << "--------_" << std::endl;
                    _com_trajectory(0) = _poly_com.get_com_initial_position().coeff(0) + delta_com(0);
                }
                else if (_current_state == State::IDLE)
                {
                    delta_com << 0, 0, 0;
                    _com_trajectory(0) = _poly_com.get_com_initial_position().coeff(0) + delta_com(0);
                    
                }
            }
        
    }
        
        
           
//         double q2;
//         sense_q1(q2);
//         std::cout << "time: " << _internal_time << std::endl;
//         std::cout << "q1_again: " << atan(com_to_ankle_distance.x()/com_to_ankle_distance.z()) << std::endl;
//         std::cout << "com_initial_position: " << - _poly_com.get_com_initial_position().coeff(0) << std::endl;
//         std::cout << "com_final_position: " << - _poly_com.get_com_final_position().coeff(0) << std::endl;
//         std::cout << "com_to_ankle_distance_1: " << - com_to_ankle_distance.x() << std::endl;
//         std::cout << "com_to_ankle_distance_2: " << _current_pose_ROS.get_com().coeff(0) - _poly_com.get_com_initial_position().coeff(0) <<std::endl;
//         std::cout << "q1: " << _q1 << std::endl;
//         std::cout << "com_trajectory: " << _com_trajectory(0) << std::endl;
//         std::cout << "com_pos: " << _current_pose_ROS.get_com().coeff(0) <<std::endl;
//         std::cout << "tracking_error: " << _com_trajectory(0) - _current_pose_ROS.get_com().coeff(0) <<std::endl;
//         std::cout << "-----------------------------------" << std::endl;


        /// send com lateral
        _com_trajectory(1) = lateral_com(time).coeff(0);
        
        /// send foot
        
        _previous_foot_trajectory = _foot_trajectory;
        
        if (_initial_param.get_walking_forward())
        {
//             foot_trajectory = compute_swing_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time);
            _foot_trajectory = compute_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time);
            
        }
        else
        {
            Eigen::Affine3d keep_position = _poly_step.get_foot_final_pose();
            
            keep_position.translation()[0] = _poly_step.get_foot_initial_pose().translation()[0];
            keep_position.translation()[2] = _poly_step.get_foot_initial_pose().translation()[2];
            _poly_step.set_foot_final_pose(keep_position);
                
            _foot_trajectory = compute_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time);
        
                        

        }

        
        _zmp_ref << _com_trajectory.coeff(0), _zmp_window_y.coeff(0), 0;
        
//         _logger->add("old_com_pos", _old_com_pos);
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
        _logger->add("alpha", alpha);
        _logger->add("q1_cmd", _q1);
        _logger->add("q1_temp", _q1_temp);
        
        
//         _logger->add("q2_sensed", q2);
        _logger->add("q1_sensed", sense_q1());
        _logger->add("q1_max", _q1_max);
        _logger->add("q1_min", _q1_min);
        _logger->add("q1_start", _q1_start);

        _logger->add("vel_q1", _vel_q1);
        _logger->add("steepness", _steep_coeff);
       
        _logger->add("initial_com", _poly_com.get_com_initial_position().coeff(0));
      
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
        
        _poly_step.log(_logger);
       

//         tilt_x_meas();
        
        send_com(_com_trajectory);
        send_step(_foot_trajectory);
        
        
        send_zmp(_zmp_ref);
        
//     }
    
    //burn impact event
    if (_event == Event::IMPACT)
    {
        _event = Event::EMPTY;
        _step_counter++;
        
        if (_step_counter >= _initial_param.get_max_steps()) // -1
        {
            _cycle_counter++;
        }
    }
}
    
bool virtualConstraintsNode::ST_idle(double time)
{
    _initial_com_position = _current_pose_ROS.get_com();
    _initial_sole_pose = _current_pose_ROS.get_sole_tot(_current_side);
    
    _final_sole_pose = _initial_sole_pose;
    _final_com_position = _initial_com_position;
    
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
    _poly_com.set_com_final_position(_final_com_position);
    
    _poly_com.set_starTime(time);
    _poly_com.set_endTime(time+10);
    
    return 1;
};

        
bool virtualConstraintsNode::ST_walk(double time, Step step_type)
{
    _initial_com_position = _current_pose_ROS.get_com();
    _initial_sole_pose = _current_pose_ROS.get_sole_tot(_current_side);
    
    _final_sole_pose = _initial_sole_pose;
    
    _final_com_position = _initial_com_position;
    
    compute_step(step_type);
    
    _poly_step.set_foot_initial_pose(_initial_sole_pose);
    _poly_step.set_foot_final_pose(_final_sole_pose);
    // ---------------------------------------------
    _poly_step.set_starTime(time);
    _poly_step.set_endTime(time + _initial_param.get_duration_step());
    
//     _poly_step.set_starTime(_q1);
//     _poly_step.set_endTime(_q1 + _q1_max); // TODO more or less work so that everything is parametrized as q !!!!!!!!!!!!!
    _poly_step.set_step_clearing(_initial_param.get_clearance_step());
    
//     std::cout << "time_start: " << time << std::endl;
//     std::cout << "time_inside_start: " << _poly_step.get_starTime() << std::endl;
//     std::cout << "time_inside_end: " << _poly_step.get_endTime() << std::endl;
//     std::cout << "step_pos_init: " <<_poly_step.get_foot_initial_pose().translation().transpose() << std::endl;
//     std::cout << "step_pos_final: " <<_poly_step.get_foot_final_pose().translation().transpose() << std::endl;
    // ---------------------------------------------
    _poly_com.set_com_initial_position(_initial_com_position);
    _poly_com.set_com_final_position(_final_com_position);
    
    _poly_com.set_starTime(time);
    _poly_com.set_endTime(time + _initial_param.get_duration_step());
    

    return 1;
};

bool virtualConstraintsNode::compute_step(Step step_type)
{
    switch (step_type)
    {
        case Step::HALF :
        {
//                // position
                double q1 = (_q1_max - _q1_min);
                
                // needed for the first inclination (if any) of the robot. The step needs to travel independently _q1_max
                if (_current_state == State::STARTING)
                {
                    q1 = _q1_max;
                }
                
//                 std::cout << "_q1_min " <<_q1_min << std::endl;
//                 std::cout <<"_q1_max "<< _q1_max << std::endl;
//                 std::cout << q1 << std::endl;
                
                Eigen::Vector3d delta_com;
                delta_com << (- _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max)), 0, 0;
                _final_com_position += delta_com;
                
                /*
                *I can take out HALF and FULL step, since the only thing I choose now is _q1_max.
                *The fact is, since at the beginning it senses a q1 (for instance, q1 = 0, so everything straigth) to 
                *get to _q1_max it only needs an HALF step. At the impact, the sensed q1 is -q1_max, and to ge to -q1_max
                *you need a FULL step. 
                *TODO Take out the steps?      
                 */
                
                Eigen::Vector3d delta_step, final_sole_position;
                delta_step << (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(q1)), 0, 0;
                _final_sole_pose.translation() += delta_step;
//                // orientation
                
                Eigen::Quaterniond sole_orientation;
                
                sole_orientation.x() = 0;
                sole_orientation.y() = 0;
                sole_orientation.z() = 0;
                sole_orientation.w() = 1;
                
                _final_sole_pose.linear() = sole_orientation.normalized().toRotationMatrix();
//                 std::cout << "Half Step:" << std::endl;
//                 std::cout << "Orientation: " << std::endl << _final_sole_pose.rotation() << std::endl;
//                 std::cout << "Position: " << _final_sole_pose.translation().transpose() << std::endl;
                
                 _steep_coeff = (_q1_max - _q1_min)/_step_duration;
                break;
        }
        case Step::FULL :
        {  
            
//                 // position

                double q1 = (_q1_max - _q1_min);
                
                if (_current_state == State::STARTING || _current_state == State::LASTSTEP)
                {
                    q1 = _q1_max - _initial_param.get_lean_forward();
                }
                
//                 std::cout << _q1_min << std::endl;
//                 std::cout << _q1_max << std::endl;
//                 std::cout << q1 << std::endl;

                Eigen::Vector3d delta_com;
                delta_com << (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max)), 0, 0;
                _final_com_position += delta_com;
                    
                Eigen::Vector3d delta_step;
                delta_step << (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(q1)), 0, 0; //2 * 
                _final_sole_pose.translation() += delta_step;
                
//                 // orientation
                
                Eigen::Quaterniond sole_orientation;
                
                sole_orientation.x() = 0;
                sole_orientation.y() = 0;
                sole_orientation.z() = 0;
                sole_orientation.w() = 1;
                
                _final_sole_pose.linear() = sole_orientation.normalized().toRotationMatrix();
//                 std::cout << "Full Step:" << std::endl;
//                 std::cout << "Orientation: " << std::endl << _final_sole_pose.rotation() << std::endl;
//                 std::cout << "Position: " << _final_sole_pose.translation().transpose() << std::endl;
                
                 _steep_coeff = (_q1_max - _q1_min)/_step_duration/2;
               
                break;
        }
        case Step::STEER :
        {  
//                 double R = 0.5; //radius of curvature
                
//                  // POSITION
//              // com
                double distance_com = (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max));
               
                Eigen::Vector3d delta_com;
                delta_com << distance_com, 0, 0; //_lateral_step
                
                _final_com_position += delta_com;
                
//              // step
                double  distance_step = (- 4* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max));
                
                
//                 double theta_step = distance_step/R;
//                 Eigen::Vector3d delta_step;
//                 delta_step << R * sin(theta_step), R * (1 - cos(theta_step)), 0; //_lateral_step
                
                Eigen::Vector3d delta_step;
                double theta_step = M_PI/2;
                
                double distance_new = distance_step * cos(theta_step);
                
                delta_step << distance_new * cos(theta_step), distance_new * sin(theta_step), 0; //_lateral_step
                _final_sole_pose.translation() += delta_step;
                
//                 // ORIENTATION
                
                Eigen::Quaterniond sole_orientation;
                
                sole_orientation.x() = 0;
                sole_orientation.y() = 0;
                sole_orientation.z() = theta_step;
                sole_orientation.w() = 1;
                
                _final_sole_pose.linear() = sole_orientation.normalized().toRotationMatrix();
//                 std::cout << "Full Step:" << std::endl;
//                 std::cout << "Orientation: " << std::endl << _final_sole_pose.rotation() << std::endl;
//                 std::cout << "Position: " << _final_sole_pose.translation().transpose() << std::endl;
                
                break;
        }
        
        default : throw std::runtime_error(std::string("wrong type of step. For now only FULL, HALF and STEER are implemented"));                
    }
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
                    planner(time);
                    break;
                    
                case State::STARTING :
                    _current_state = State::WALK;
                    planner(time);
                    break;
                    
                case State::STOPPING :
                    _current_state = State::LASTSTEP;
                    _q1_max = 0;
                    planner(time);
                    break;
                    
                case State::LASTSTEP :
                    _started = 0;
                    _q1_start = _q1_temp;
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
                    _current_state = State::STARTING;
                    _q1_max = _initial_param.get_max_inclination();
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
    double clearance = _initial_param.get_clearance_step();
    _reset_condition = 0; 
    
    _delay_start = 1.5;
    _current_side = _initial_param.get_first_step_side();
    std::cout << "First step: " << _current_side << std::endl;
    _other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
    

    _initial_sole_y_right = _current_pose_ROS.get_sole(robot_interface::Side::Right).coeff(1);
    _initial_sole_y_left = _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1);
    
    
    _initial_com_position = _current_pose_ROS.get_com();
    _initial_sole_pose = _current_pose_ROS.get_sole_tot(_current_side);
    _final_sole_pose = _initial_sole_pose;
    _final_com_position = _initial_com_position;

    planner(time);
    Eigen::Vector3d foot_trajectory;
    Eigen::Vector3d fake_pose;
    fake_pose.setZero();
    
    // just to run it once, heat up the process
    foot_trajectory = compute_swing_trajectory(fake_pose, fake_pose, 0, 0, 0, time);
    /* comy */

    double Ts = 0.01; //window resolution
    double T = _initial_param.get_duration_preview_window(); //window length for MpC
    double dt = 0.01; //rate of ros
    _t_before_first_step = 0; // preparation time in the first step for the com to swing laterally before stepping 
    
    _q1_min = sense_q1(); // min angle of inclination
    _q1_max = _initial_param.get_max_inclination(); // max angle of inclination
    _q1_start = 0; // starting q1, incremented every time that the robot stops (when restart, it needs it)
    _step_duration = _initial_param.get_duration_step();

//     _steep_coeff = _q1_max/_step_duration;
        _steep_coeff = 0; //(_q1_max - _q1_min)/_step_duration;

//     std::cout << "_steep_coeff: " << _steep_coeff << std::endl;
    _nominal_full_step = fabs(- 4* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max));
    _nominal_half_step = fabs(- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max));
    
    std::cout << "Start walk time: " << _initial_param.get_start_time() << " s" <<  std::endl;
    std::cout << "Lean forward: " << _initial_param.get_lean_forward() << " m (Initial angle: " << _initial_q1 << " rad)" << std::endl;
    std::cout << "Step length: " << _nominal_full_step << " m (Max angle of inclination: " << _q1_max << " rad)" <<  std::endl;
    std::cout << "Step duration: " << _step_duration << " s" << std::endl;
    std::cout << "Double stance: " <<  _initial_param.get_double_stance() << " s" << std::endl;
    std::cout << "Steepness: " << _steep_coeff <<  std::endl;
    std::cout << "Real impacts: " << _initial_param.get_switch_real_impact() <<  std::endl;
    

    _com_y << _initial_com_position(1), 0, 0; //com trajectory used by mpc: pos, vel, acc

    int sign_first_stance_step = (_current_pose_ROS.get_sole(_other_side).coeff(1) > 0) - (_current_pose_ROS.get_sole(_other_side).coeff(1) < 0);

    _first_stance_step = _current_pose_ROS.get_sole(_other_side).coeff(1) - sign_first_stance_step * _initial_param.get_indent_zmp();

    
    
    // generate zmp given start walk and first stance step
//     generate_zmp(_first_stance_step, _initial_param.get_start_time(), _initial_param.get_double_stance(), _initial_param.get_max_steps(), dt, _zmp_t, _zmp_y);
    
    // steer zmp
    
    //     generate_zmp(_first_stance_step, _initial_param.get_start_time(), _initial_param.get_double_stance(), _initial_param.get_max_steps(), dt, _zmp_t_steer, _zmp_y_steer); //TODO once filled, I shouldn't be able to modify them
    

    // generate different ZMP keeping the ZMP constant

       
//         Eigen::VectorXd point_t(2);
//         Eigen::VectorXd point_y_right(2), point_y_left(2);
//         point_t << 0, 10; //TODO horizon for the delay
//         point_y_right << -_first_stance_step, -_first_stance_step;
//         
//         lSpline(point_t, point_y_right, dt, _zmp_t_fake_right, _zmp_y_fake_right);
//         
//         _zmp_y_fake_right[_zmp_y_fake_right.size()-1] = _zmp_y_fake_right[_zmp_y_fake_right.size()-2]; 
//         _zmp_t_fake_left = _zmp_t_fake_right;
//         _zmp_y_fake_left = -_zmp_y_fake_right;
//     
//         _zmp_y_fake_center = _zmp_y_fake_right;
//         _zmp_y_fake_center.setZero();
        
    // get impact position planned in time
//     _planned_impacts.resize(_initial_param.get_max_steps(),1);
//     
//     for (int i = 1; i <= _initial_param.get_max_steps(); i++)
//     {
//         _planned_impacts(i-1) = _initial_param.get_start_time() +  _initial_param.get_duration_step() * i;
//     }
    
//     for (int i = 0; i < (_zmp_t).size(); i++)
//     {
//         _logger->add("zmp_t", (_zmp_t)(i));
//         _logger->add("zmp_y", (_zmp_y)(i));
//     }

    Eigen::MatrixXd Q(1,1);
    Eigen::MatrixXd R(1,1);
    
    Q << _initial_param.get_MPC_Q(); //1000000
    R << _initial_param.get_MPC_R();
    
    
//     _spatial_zmp_y = initialize_spatial_zmp();
    
    _MpC_lat = std::make_shared<item_MpC>(_initial_height, Ts, T, Q, R);
    
    generate_starting_zmp();
    
    _zmp_window_y.setZero();
    _zmp_window_t.setZero();
    _com_y.setZero();
    _u.setZero();
    
        
    _lateral_step = 0;
    _init_completed = 1;
    
    _last_event = Event::EMPTY;
    _event = Event::EMPTY;
    
//     planner(0);
//     impact_routine();
//     core(0);
       commander(_internal_time); //fake commander
//         
//     com displacement given the max angle  
    _com_max.resize(_initial_param.get_max_steps(), 1);
    double d_com = - 2 * _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max);
    
    _com_max[0] = _current_pose_ROS.get_com().coeff(0) - _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max);
    
    for (int i = 1; i < _initial_param.get_max_steps(); i++)
    {
        _com_max[i] = _com_max[i-1] + d_com;
    }
// ---------------------------------------
    
    _start_walk =_initial_param.get_start_time();
   
    _logger->add("zmp_starting", _zmp_starting);
    _logger->add("spatial_zmp_y", _spatial_zmp_y);
    std::cout << "Initialization complete." << std::endl;
        
//     zmp_x_offline(5000);
    
    
    return 1;
                
}

void virtualConstraintsNode::spatial_zmp(double& current_spatial_zmp_y, Eigen::VectorXd &spatial_window_preview, double length_preview_window, double dx, Step type_step)
{
    /**
     * @brief spatial ZMP_y computed at each control cycle
     * 
     **/
    
    double dt = 0.01;
    
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
    int sign_first_stance_step = (_current_pose_ROS.get_sole(other_side).coeff(1) > 0) - (_current_pose_ROS.get_sole(other_side).coeff(1) < 0);
    double zmp_y = _current_pose_ROS.get_sole(other_side).coeff(1) - sign_first_stance_step * _initial_param.get_indent_zmp();

    _logger->add("zmp_y_rec", zmp_y);
    
    if (alpha >= 0)
    {
        if (alpha <= 1)
        {
            current_spatial_zmp_y = zmp_y;
//             current_spatial_zmp_y = _first_stance_step * _switched;
        }
    
    
    }
    
    // ------------------------------------------------
    // window from zmp spatial
    // ------------------------------------------------

    
    double foot_position = _initial_pose.get_sole(other_side).coeff(0);
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
    double dt = 0.01;
    Eigen::Vector3d com_vel = (_com_trajectory - _previous_com_trajectory)/dt;  
    
    return com_vel;
    
}

Eigen::Vector3d virtualConstraintsNode::sense_com_velocity()
{
    double dt = 0.01;
    Eigen::Vector3d com_pos = _current_pose_ROS.get_com();
    Eigen::Vector3d com_vel = (com_pos - _previous_com_pos)/dt;
    _previous_com_pos = _current_pose_ROS.get_com();
    
    return com_vel;
    
}

Eigen::Vector3d virtualConstraintsNode::sense_foot_velocity()
{
    double dt = 0.01;
    Eigen::Vector3d foot_pos = _current_pose_ROS.get_sole(_current_side);
    Eigen::Vector3d foot_vel = (foot_pos - _previous_foot_pos)/dt;
    _previous_foot_pos = _current_pose_ROS.get_sole(_current_side);
    
    return foot_vel;
}

Eigen::Vector3d virtualConstraintsNode::get_foot_velocity()
{
    double dt = 0.01;
    Eigen::Vector3d foot_vel = (_foot_trajectory.translation() - _previous_foot_trajectory.translation())/dt;  
    
    return foot_vel;
}



double virtualConstraintsNode::q_handler()
{
    double dt = 0.01;
    
    // velocity of the CoM is computed at each step based on _q1_max and _q1_min
//     if (_current_state == State::STARTING || _current_state == State::LASTSTEP)
//     {
//         _steep_coeff = (_q1_max - _q1_min)/_step_duration;
//     }
//     else if (_current_state == State::WALK)
//     {
//         _steep_coeff = (_q1_max - _q1_min)/_step_duration/2;
//     }
     
    
    if (_started == 1 && _internal_time >= +_start_walk)
    {
        if (sense_q1() > _q1_max)
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


// void virtualConstraintsNode::q_max_handler()
// {
//     if (_current_state == State::LASTSTEP)
//     {
//         _q1_max = 0;
//     }
//     else
//     {
//         _q1_max = _initial_param.get_max_inclination(); // max angle of inclination
//     }
// }

// void virtualConstraintsNode::val_getter()
// {
//     
// }


void virtualConstraintsNode::resetter()
{
    generate_starting_zmp();
//     straighten_up_action();
}

void virtualConstraintsNode::generate_starting_zmp()
{
    double dt = 0.01;
    int sign_first_stance_step = (_current_pose_ROS.get_sole(_other_side).coeff(1) > 0) - (_current_pose_ROS.get_sole(_other_side).coeff(1) < 0);
    double initial_step_value = _current_pose_ROS.get_sole(_other_side).coeff(1) - sign_first_stance_step * _initial_param.get_indent_zmp();
    
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

// void virtualConstraintsNode::generate_zmp(double y_start, double t_start, double double_stance, int num_points, double dt, Eigen::VectorXd& zmp_t, Eigen::VectorXd& zmp_y)
// {
// //         int num_points = _initial_param.get_max_steps();
//         double t_end = t_start + _step_duration*num_points;
//         //TODO qui potrei mettere anche un t_windows_end
//         Eigen::VectorXd y, times;
//         
//         y.resize(num_points*2,1);
//         times.resize(num_points*2,1);
//             
//         int myswitch = 1;
//         int i = 0;
//         int j = 0;
//         for (i = 0; i<num_points; i++)
//         {
//             times(j) = t_start + _step_duration* i + double_stance*(i+1);    
//             times(j+1) = t_start + _step_duration* (i+1) + double_stance*(i+1);
//             
//             y(j) = myswitch * y_start;
//             y(j+1) = myswitch * y_start;
//             myswitch = -1 * myswitch;
//             j = j+2;
//         }
//         
//         
//         Eigen::VectorXd y_tot(y.size() + 3);
//         Eigen::VectorXd times_tot(times.size() + 3);
//         
//         y_tot << 0, 0, y, 0;
//         times_tot << 0, t_start, times, t_end + double_stance*(i+1);
// 
// //         std::cout << "y_tot: " << y_tot.transpose() << std::endl;
// //         std::cout << "times_tot: " << times_tot.transpose() << std::endl;
//         
//         lSpline(times_tot, y_tot, dt, zmp_t, zmp_y);
//     }

// void virtualConstraintsNode::zmp_window(Eigen::VectorXd zmp_t, Eigen::VectorXd zmp_y, double window_start, double window_end, Eigen::VectorXd &zmp_window_t, Eigen::VectorXd &zmp_window_y)
//     {
// 
//         
//         int window_size = round((window_end - window_start))/_MpC_lat->_Ts; //+1
// 
//         zmp_window_t.setLinSpaced(window_size, window_start, window_end);
//         
//         int i = 0;
//         while (i < zmp_t.size() && zmp_t(i) < window_start)
//         {
//             i++;
//         }
// //         
//         int j = 0;
//         while (j < zmp_t.size() && zmp_t(j) < window_end)
//         {
//             j++;
//         }
//         
//         zmp_window_y.resize(window_size,1);
// 
//         zmp_window_y.setZero();
//         zmp_window_y.segment(0, j-i) = (zmp_y).segment(i, j-i);
//         zmp_window_y[zmp_window_y.size()-1] = zmp_window_y[zmp_window_y.size()-2]; //ADDED
// 
//     }

// Eigen::VectorXd virtualConstraintsNode::initialize_spatial_zmp()
// {
//     /**
//      * @brief initialize spatial ZMP_y
//      * for now the dx is _steep_coeff * dt (where _steep_coeff actually is q_max/duration_step)
//     **/
//     
//     double N = 5; //number of chunks in the zmp trajectory
//     
//     double length_step = fabs(- 4* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max));
// 
//     double dt = 0.01;
//     double dx = _steep_coeff * dt; // v * dt
//     int chunk_length = floor(length_step/dx);
//     
//     Eigen::VectorXd spatial_zmp_y(5*chunk_length);
//     spatial_zmp_y.setZero();
//     
//     add_zmp_y_chunk(spatial_zmp_y, 0.2, 0.01, robot_interface::Side::Left);
//     add_zmp_y_chunk(spatial_zmp_y, 0.2, 0.01,robot_interface::Side::Right);
//     add_zmp_y_chunk(spatial_zmp_y, 0.2, 0.01, robot_interface::Side::Left);
//     add_zmp_y_chunk(spatial_zmp_y, 0.2, 0.01, robot_interface::Side::Right);
//     add_zmp_y_chunk(spatial_zmp_y, 0.2, 0.01, robot_interface::Side::Left);
//     
//     return spatial_zmp_y;
//     
//     
// }


// void virtualConstraintsNode::add_zmp_y_chunk(Eigen::VectorXd& spatial_zmp_y, double length_step, double dx, robot_interface::Side zmp_side)
// {
//     /**
//      * @brief add to spatial_zmp_y a chunk (length of a step) of the zmp desired trajectory of the ZMP_y
//     **/
//     double step_y;
//     switch (zmp_side)
//     {
//         case robot_interface::Side::Double :
//             step_y = 0;
//             break;
//         case robot_interface::Side::Right :
//             step_y = _initial_sole_y_right - _initial_param.get_indent_zmp();
//             break;
//         case robot_interface::Side::Left :
//             step_y = _initial_sole_y_left + _initial_param.get_indent_zmp();
//             break;
//     }
// 
//     int chunk_length = floor(length_step/dx);
// 
//     Eigen::VectorXd chunk(chunk_length);
//     
//     chunk = step_y * chunk.setOnes();
//     
//     Eigen::VectorXd temp_zmp(spatial_zmp_y.size());
//     temp_zmp << spatial_zmp_y.tail(spatial_zmp_y.size() - chunk_length), chunk;
//     
//     spatial_zmp_y << temp_zmp;
//     
// }

// void virtualConstraintsNode::zmp_x_offline(int s_max)
// {
//     double dt = 0.01;
//     double com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2);
//     
//     
//     Eigen::VectorXd com_max(_initial_param.get_max_steps());
//     double d_com = - 2 * _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max);
//     
//     com_max[0] = _current_pose_ROS.get_com().coeff(0) - _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q1_max);
//     
//     for (int i = 1; i < _initial_param.get_max_steps(); i++)
//     {
//         com_max[i] = com_max[i-1] + d_com;
//     }
//     
//     
//     
//     
//     Eigen::VectorXd com_x(s_max);
//     Eigen::VectorXd zmp_x(s_max);
//     
//     double q1 = sense_q1();
//     double com_x_initial =  _current_pose_ROS.get_com().coeff(0);
//     
//     _logger->add("q1", q1);
//     int n_step = 0;
//     double dx = 0;
//     int s = 0;
//     while (s < s_max)
//     {
//         q1 = q1 + (_steep_coeff * dt);
//         
//        
//         if (n_step < 1)
//         {
//             dx = - com_to_ankle_distance * tan(q1);
//         }
//         else if (n_step >= 1)
//         {
//             dx = - 2* com_to_ankle_distance * tan(q1);
//         }
//         
//         com_x[s] = com_x_initial + dx;
//         
//         if (com_x[s] >= com_max[n_step])
//         {
//             q1 = 0;
//             com_x_initial = com_x[s-4];
//             n_step++;
//             
//         }
//         zmp_x[s] = com_x[s];
//         s = s+1;
//         
//         _logger->add("q1", q1);
//         
//     }
//     
//     _logger->add("zmp_x_offline", zmp_x);
//     _logger->add("com_x_offline", com_x);
//     _logger->add("n_step", n_step);
//     _logger->add("com_max", com_max);
//     
// }

// void virtualConstraintsNode::zmp_x_online(int s_max)
// {
//     double dt = 0.01;
//     double com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2);
//     
//     Eigen::VectorXd com_x(s_max);
//     Eigen::VectorXd zmp_x(s_max);
//     
//     double q1 = sense_q1();
//     double com_x_initial =  _current_pose_ROS.get_com().coeff(0);
//     
// //     _logger->add("q1", q1);
//     int n_step = 0;
//     double dx = 0;
//     int s = 0;
//     while (s < s_max)
//     {
//         q1 = q1 + (_steep_coeff * dt);
//         
//        
//         if (n_step < 1)
//         {
//             dx = - com_to_ankle_distance * tan(q1);
//         }
//         else if (n_step >= 1)
//         {
//             dx = - 2* com_to_ankle_distance * tan(q1);
//         }
//         
//         com_x[s] = com_x_initial + dx;
//         
//         if (com_x[s] >= _com_max[n_step])
//         {
//             q1 = 0;
//             com_x_initial = com_x[s-4];
//             n_step++;
//             
//         }
//         zmp_x[s] = com_x[s];
//         s = s+1;
//         
//         _logger->add("q1", q1);
//         
//     }
//     
//     _logger->add("zmp_x_offline", zmp_x);
//     _logger->add("com_x_offline", com_x);
//     _logger->add("n_step", n_step);
//     _logger->add("com_max", _com_max);
// }



// void virtualConstraintsNode::tilt_x_meas()
// {
//     robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
//     double distance1 = _current_pose_ROS.get_distance_ankle_to_com(other_side).coeff(0);
//     
//     double com_x = _current_pose_ROS.get_com().coeff(0);
//     double sole_x = _current_pose_ROS.get_sole(other_side).coeff(0);
//     
//     double distance2;
//     
//     distance2 = com_x - sole_x;    
//     
//     _logger->add("distance1", distance1);
//     _logger->add("distance2", distance2);
//     
//     
// }


// double virtualConstraintsNode::calc_zmp_x(double delta_com)
// {
//     /**
//      * @brief spatial zmp x computed at each control cycle
//      * 
//      **/
//     
//     _current_pose_ROS.sense();
//     double com_x = _current_pose_ROS.get_com().coeff(0);
//     double zmp_x;
//     
//     
//     double dt = 0.01;
//     double h = _initial_height; //TODO current or initial?
//     double w = sqrt(9.8/h);
//     
//     _vel_q1 = (_q1 - _q1_old)/dt;
//     if (_vel_q1 > _steep_coeff/2)
//     {
//         zmp_x = com_x;
//     }
//     else
//     {
//         if (delta_com < 1e-5)
//         {
//             delta_com = 0.001;
//         }
//         zmp_x = com_x + pow(_steep_coeff, 2) / (delta_com * pow(w, 2));
//     }
//     return zmp_x;
// }