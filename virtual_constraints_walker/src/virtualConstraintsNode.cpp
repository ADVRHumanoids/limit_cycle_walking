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
        
        
//      prepare subscriber node
        _q1_sub = n.subscribe("/q1", 10, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 10); /*publish to /cartesian/com/reference*/
        
        _sole_pubs[robot_interface::Side::Left] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/l_sole/reference", 10); /*publish to /l_sole/reference*/
        _sole_pubs[robot_interface::Side::Right] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 10); /*publish to /r_sole/reference*/
        

        
  
    }

bool virtualConstraintsNode::get_param_ros()
    {
        ros::NodeHandle nh_priv("~");
        int max_steps;
        double clearance_heigth, duration, drop, indentation_zmp, double_stance, start_time, lean_forward, slope_delay_impact, max_inclination, mpc_Q, mpc_R, lat_step, threshold_delay;
        std::vector<double> thresholds_impact_right(2), thresholds_impact_left(2);
        
        bool real_impacts, walking_forward, use_poly_com; 
        
        std::string first_side;

        /*default parameters*/
        double default_drop = -0.12;
        double default_clearance_heigth = 0.1;
        double default_duration = 2;
        std::string default_first_side = "Left";
        int default_max_steps = 10;
        double default_indentation_zmp = 0;
        double default_double_stance = 0;
        double default_start_time = 1;
        double default_lean_forward = 0;
        std::vector<double> default_thresholds_impact_right = {50, 100};
        std::vector<double> default_thresholds_impact_left = {50, 100};
        bool default_real_impacts = 0;
        double default_slope_delay_impact;
        bool default_walking_forward = 0;
        double default_max_inclination = 0.1;
        double default_mpc_Q = 1000000;
        double default_mpc_R = 1;
        double default_lat_step = 0;
        double default_threshold_delay = 0;
        bool default_use_poly_com = 1;
        
        drop = nh_priv.param("initial_crouch", default_drop);
        max_steps = nh_priv.param("max_steps", default_max_steps);
        duration = nh_priv.param("duration_step", default_duration);
        clearance_heigth = nh_priv.param("clearance_step", default_clearance_heigth);
        first_side = nh_priv.param("first_step_side", default_first_side);
        indentation_zmp = nh_priv.param("indent_zmp", default_indentation_zmp);
        double_stance = nh_priv.param("double_stance_duration", default_double_stance);
        start_time = nh_priv.param("start_time", default_start_time);
        lean_forward = nh_priv.param("lean_forward", default_lean_forward);
        thresholds_impact_right = nh_priv.param("force_sensor_threshold_right", default_thresholds_impact_right);
        thresholds_impact_left = nh_priv.param("force_sensor_threshold_left", default_thresholds_impact_left);
        real_impacts = nh_priv.param("real_impacts", default_real_impacts);
        slope_delay_impact = nh_priv.param("slope_delay_impact", default_slope_delay_impact); // 0 is max Steepness
        walking_forward = nh_priv.param("walking_forward", default_walking_forward); 
        max_inclination = nh_priv.param("max_inclination", default_max_inclination); 
        mpc_Q = nh_priv.param("mpc_Q", default_mpc_Q); 
        mpc_R = nh_priv.param("mpc_R", default_mpc_R); 
        lat_step = nh_priv.param("lateral_step", default_lat_step); 
        threshold_delay = nh_priv.param("threshold_delay", default_threshold_delay);
        use_poly_com = nh_priv.param("use_poly_com", default_use_poly_com);
        
        _initial_param.set_crouch(drop);
        _initial_param.set_max_steps(max_steps);
        _initial_param.set_duration_step(duration);
        _initial_param.set_clearance_step(clearance_heigth);
        _initial_param.set_indent_zmp(indentation_zmp);
        _initial_param.set_double_stance(double_stance);
        _initial_param.set_start_time(start_time);
        _initial_param.set_lean_forward(lean_forward);
        _initial_param.set_threshold_impact_right(thresholds_impact_right);
        _initial_param.set_threshold_impact_left(thresholds_impact_left);
        _initial_param.set_switch_real_impact(real_impacts);
        _initial_param.set_slope_delay_impact(slope_delay_impact);
        _initial_param.set_walking_forward(walking_forward);
        _initial_param.set_max_inclination(max_inclination);
        _initial_param.set_MPC_Q(mpc_Q);
        _initial_param.set_MPC_R(mpc_R);
        _initial_param.set_lateral_step(lat_step);
        _initial_param.set_threshold_delay(threshold_delay);
        _initial_param.set_use_poly_com(use_poly_com);
        
        if (first_side == "Left")
                _initial_param.set_first_step_side(robot_interface::Side::Left);
        else if (first_side == "Right")
                _initial_param.set_first_step_side(robot_interface::Side::Right);
        else std::cout << "unknown side starting command" << std::endl;
    }   

Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
        
        straight_com(0) = _current_pose_ROS.get_sole(_current_side).coeff(0) + _initial_param.get_lean_forward(); /*TODO*/
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

        _initial_height = fabs(_current_pose_ROS.get_com().coeff(2) - _current_pose_ROS.get_sole(_current_side).coeff(2)); //TODO
        //exit
        return 0;
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
        _current_pose_ROS.sense(); 
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
        _current_pose_ROS.sense();
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
        
        _logger->add("left_foot_to_com", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("right_foot_to_com", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
        
        _logger->add("q_sagittal_stance", q1); //TODO it's not sagittal in the double stance
        _logger->add("q_sagittal_swing", q2);
        
        _logger->add("q_sagittal_left", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(0)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("q_sagittal_right", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(0)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
   
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
    
void virtualConstraintsNode::update_pose(Eigen::Vector3d *current_pose, Eigen::Vector3d update) 
    {   
//       // update current_pose with update
        *current_pose = *current_pose + update;
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
    double cond;
    
    if (_initial_param.get_walking_forward())
    {
        cond = fabs(_current_pose_ROS.get_sole(_current_side).coeff(0) - _initial_pose.get_sole(_current_side).coeff(0)) >  0.05;
    }
    else
    {
        cond = _internal_time > (_initial_param.get_start_time() + 0.2)  && _impact_cond > 0.2;
        if (_step_counter > _initial_param.get_max_steps()-1)
            cond = 0; 
    }
    
    if (fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-4  && cond)
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
                _current_pose_ROS.sense();
               
                robot_interface::Side last_side = _current_side;
//                 std::cout << "Last side: " << last_side << std::endl;
               _initial_pose = _current_pose_ROS;
                _current_side = robot_interface::Side::Double;

                std::cout << "Impact! Current side: " << _current_side << std::endl;
                
                _current_pose_ROS.get_sole(_current_side);
                // ----------------------------------------------------
                _poly_com.set_com_initial_position(_current_pose_ROS.get_com());          //ADDED
                // ----------------------------------------------------
                

                    if (last_side == robot_interface::Side::Left)
                        _current_side = robot_interface::Side::Right;
                    else if (last_side == robot_interface::Side::Right)
                        _current_side = robot_interface::Side::Left;
                    else ROS_INFO("wrong side");
                    
                    std::cout << "State changed. Current side: " << _current_side << std::endl;
                
                    return 1;
            }
            else
            {
                return 0;
            }
    }


    
void virtualConstraintsNode::first_q1()
    {
        ROS_INFO("waiting for command...");
        std::cout << "Initial state: " << _current_side << std::endl;
        while (!_check_received)
        {
            _current_pose_ROS.sense();
            
            if (_check_received)
            {
                _q1_state = _q1_cmd;
            }
                
        }
        ROS_INFO("command received! q1 = %f", _q1_state);
    }
    
void virtualConstraintsNode::exe(double time)
{       
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

//  // -------------for q1------------------------------------- 
    double q1_temp = 0;    
    if (_internal_time >= _initial_param.get_start_time() + _t_before_first_step)
    {
        q1_temp = _steep_coeff*(_internal_time - _initial_param.get_start_time() - _t_before_first_step); // basically q = a*t
    }
//  // ---------------------------------------------------------  

    if (_internal_time >= _initial_param.get_start_time() && _cycle_counter == 0)
    {
        _last_event = _event;
        _event = Event::START;
        _cycle_counter = 1;
    };
    
    if (_cycle_counter == 2)
    {
        _last_event = _event;
        _event = Event::STOP;
    };
        
        if (impact_routine())
        {    
            _q_min = sense_q1();
            _reset_condition = q1_temp;
            
            _reset_time = _internal_time;
        }

//  // -------------for q1------------------------------------- 
        _q1 = q1_temp - _reset_condition;
        
        if (_q1 > _q_max) 
        {
            _q1 = _q_max;
        }
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

void virtualConstraintsNode::generate_zmp(double y_start, double t_start, double double_stance, int num_points, double dt, Eigen::VectorXd& zmp_t, Eigen::VectorXd& zmp_y)
{
//         int num_points = _initial_param.get_max_steps();
        double t_end = t_start + _step_duration*num_points;
        //TODO qui potrei mettere anche un t_windows_end
        Eigen::VectorXd y, times;
        
        y.resize(num_points*2,1);
        times.resize(num_points*2,1);
            
        int myswitch = 1;
        int i = 0;
        int j = 0;
        for (i = 0; i<num_points; i++)
        {
            times(j) = t_start + _step_duration* i + double_stance*(i+1);    
            times(j+1) = t_start + _step_duration* (i+1) + double_stance*(i+1);
            
            y(j) = myswitch * y_start;
            y(j+1) = myswitch * y_start;
            myswitch = -1 * myswitch;
            j = j+2;
        }
        
        
        Eigen::VectorXd y_tot(y.size() + 3);
        Eigen::VectorXd times_tot(times.size() + 3);
        
        y_tot << 0, 0, y, 0;
        times_tot << 0, t_start, times, t_end + double_stance*(i+1);

//         std::cout << "y_tot: " << y_tot.transpose() << std::endl;
//         std::cout << "times_tot: " << times_tot.transpose() << std::endl;
        
        lSpline(times_tot, y_tot, dt, zmp_t, zmp_y);
    }

void virtualConstraintsNode::zmp_window(Eigen::VectorXd zmp_t, Eigen::VectorXd zmp_y, double window_start, double window_end, Eigen::VectorXd &zmp_window_t, Eigen::VectorXd &zmp_window_y)
    {

        
        int window_size = round((window_end - window_start))/_MpC_lat->_Ts + 1;

        zmp_window_t.setLinSpaced(window_size, window_start, window_end);
        
        int i = 0;
        while (i < zmp_t.size() && zmp_t(i) < window_start)
        {
            i++;
        }
//         
        int j = 0;
        while (j < zmp_t.size() && zmp_t(j) < window_end)
        {
            j++;
        }
        
        zmp_window_y.resize(window_size,1);

        zmp_window_y.setZero();
        zmp_window_y.segment(0, j-i) = (zmp_y).segment(i, j-i);
        zmp_window_y[zmp_window_y.size()-1] = zmp_window_y[zmp_window_y.size()-2]; //ADDED

    }
    
Eigen::Vector3d virtualConstraintsNode::lateral_com(double time)
{
        
        double dt = 0.01; //TODO take it out from here
        _entered_forward = 0;
        
        if (_event == Event::IMPACT && _step_counter <= _initial_param.get_max_steps() - 1)  // jump in time, going to closer planned impact
        {

            _entered_forward = 1;  
            _shift_time = time - _planned_impacts(_step_counter) - dt; //_planned_impacts(ceil((time - _initial_param.get_start_time())/_initial_param.get_duration_step()))
            
            _period_delay = 0;

                
        }
//         // -----------------------------------------------------------------------------------------------------------------------------
        
        double window_start = time - _shift_time;
        
        
        _entered_delay = 0;
        
        if (_step_counter <= _initial_param.get_max_steps()-1)
        {
            if (_current_state != State::IDLE && time > _planned_impacts(_step_counter) + _shift_time)
            {
                
                _entered_delay = 1;
                _period_delay = time - _planned_impacts(_step_counter) + _shift_time; // HOW MUCH TIME IT IS STAYING HERE
                
                
        
                if (_step_counter % 2 == 0)
                {
                            window_start = time - (_planned_impacts(_step_counter) + _shift_time); 
                            zmp_window(_zmp_t_fake_right, _zmp_y_fake_right, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
                }
                else
                {
                            window_start = time - (_planned_impacts(_step_counter) + _shift_time);
                            zmp_window(_zmp_t_fake_left, _zmp_y_fake_left, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
                }

            }
            else // if it's not entered inside delay
            {
                zmp_window(_zmp_t, _zmp_y, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
            }
        }
        
        if (_step_counter > _initial_param.get_max_steps()-1)
        {
            zmp_window(_zmp_t_fake_center, _zmp_y_fake_center, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
        }


        _u = _MpC_lat->_K_fb * _com_y + _MpC_lat->_K_prev * _zmp_window_y;
        _MpC_lat->_integrator->integrate(_com_y, _u, dt, _com_y);
         
        return _com_y;    
}

void virtualConstraintsNode::commander(double time)
{
  
        //// send com sagittal
        Eigen::Vector3d com_trajectory;
        com_trajectory = _poly_com.get_com_initial_position();

            
    if (_initial_param.get_use_poly_com())
    {
            if (_initial_param.get_walking_forward())
            {
                com_trajectory = compute_swing_trajectory(_poly_com.get_com_initial_position(), _poly_com.get_com_final_position(), 0, _poly_com.get_starTime(), _poly_com.get_endTime(), time);
            }
    }
    else
    {
            if (_initial_param.get_walking_forward())
            {
                robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
                Eigen::Vector3d com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(other_side);
                Eigen::Vector3d delta_com;
                com_trajectory = _current_pose_ROS.get_com();
                
                if (_current_state == State::STARTING || _current_state == State::STOPPING)
                {
                    delta_com << - com_to_ankle_distance.z() * tan(_q1), 0, 0;
                    com_trajectory(0) = _poly_com.get_com_initial_position().coeff(0) + delta_com(0);
                }
                else if (_current_state == State::WALK)
                {
                    delta_com << - 2* com_to_ankle_distance.z() * tan(_q1), 0, 0;
                    com_trajectory(0) = _poly_com.get_com_initial_position().coeff(0) + delta_com(0);
                }
                else if (_current_state == State::IDLE)
                {
                    delta_com << 0, 0, 0;
                    com_trajectory(0) = _poly_com.get_com_initial_position().coeff(0) + delta_com(0);
                    
                }
            }
        
    }
        /// send com lateral
        com_trajectory(1) = lateral_com(time).coeff(0);
        
        /// send foot
        Eigen::Affine3d foot_trajectory;
        
               if (_initial_param.get_walking_forward())
        {
//             foot_trajectory = compute_swing_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time);
            foot_trajectory = compute_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time);
        }
        else
        {
            Eigen::Affine3d keep_position = _poly_step.get_foot_final_pose();
            
            keep_position.translation()[0] = _poly_step.get_foot_initial_pose().translation()[0];
            keep_position.translation()[2] = _poly_step.get_foot_initial_pose().translation()[2];
             _poly_step.set_foot_final_pose(keep_position);
                
            foot_trajectory = compute_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time);
        }

        _logger->add("com_trajectory", com_trajectory);
        _logger->add("foot_trajectory", foot_trajectory.translation());
        
        _logger->add("time", _internal_time); // TODO wrong dimension wrt to logger foot and com
        
        _logger->add("ft_left", _current_pose_ROS.get_ft_sole(robot_interface::Side::Left));
        _logger->add("ft_right", _current_pose_ROS.get_ft_sole(robot_interface::Side::Right));

        _logger->add("landed_left", static_cast<int>(_current_phase_left));
        _logger->add("landed_right",  static_cast<int>(_current_phase_right));
    
        _logger->add("zmp_ref", _zmp_window_y.coeff(0));
        _logger->add("window_tot", _zmp_window_y);
        
        _logger->add("com", _com_y);
        _logger->add("u", _u);
        _logger->add("zmp", _MpC_lat->_C_zmp*_com_y);
        
//         _logger->add("flag_impact", _flag_impact);
        
        _logger->add("entered_period_delay", _entered_period_delay);
        _logger->add("sense_q1", sense_q1());
        _logger->add("period_delay", _period_delay);
        _logger->add("entered_forward", _entered_forward);
        _logger->add("entered_delay", _entered_delay);
        
        send_com(com_trajectory);
        send_step(foot_trajectory);
//     }
    
    //burn impact event
    if (_event == Event::IMPACT)
    {
        _event = Event::EMPTY;
        _step_counter++;
        
        if (_step_counter >= _initial_param.get_max_steps()-1)
        {
            std::cout << "entered_cycle" << std::endl;
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
    _poly_step.set_step_clearing(_initial_param.get_clearance_step());
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
                Eigen::Vector3d delta_com;
                delta_com << (- _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0;
                _final_com_position += delta_com;

                Eigen::Vector3d delta_step, final_sole_position;
                delta_step << (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0;
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
                
                break;
        }
        case Step::FULL :
        {  
            
//                 // position
                Eigen::Vector3d delta_com;
                delta_com << (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0; // _lateral_step
                _final_com_position += delta_com;
                    
                Eigen::Vector3d delta_step;
                delta_step << (- 4* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0; //_lateral_step
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
                
                
                break;
        }
        case Step::STEER :
        {  
//                 double R = 0.5; //radius of curvature
                
//                  // POSITION
//              // com
                double distance_com = (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max));
               
                Eigen::Vector3d delta_com;
                delta_com << distance_com, 0, 0; //_lateral_step
                
                _final_com_position += delta_com;
                
//              // step
                double  distance_step = (- 4* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max));
                
                
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
                ST_walk(time, Step::HALF);
                break;
        
            case State::WALK :
                ST_walk(time, Step::FULL);
                break;
        
            case State::STOPPING :
                 ST_walk(time, Step::HALF);
                 break;
                 
            case State::IDLE :
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
                    _current_state = State::IDLE;
                    break;
            }
            break;
        
        case Event::START :
            switch (_current_state)
            {

                case State::IDLE :
                    _current_state = State::STARTING;
                    planner(time); //_t_before_first_step
                    break;
                    
                default : /*std::cout << "Ignored starting event. Already WALKING" << std::endl; */
                    break;
            }
            break;
         
        case Event::STOP :
            switch (_current_state)
            {
                case State::IDLE :
                    /* std::cout << "Ignored stopping event. Already in IDLE" << std::endl; */
                    break;
                   
                case State::WALK :
                    _end_walk = time;
                    _current_state = State::STOPPING;
                    
                    planner(time);
                    break;
                    
                case State::STARTING :
                    _end_walk = time;
                    _current_state = State::STOPPING;
                    planner(time);
                    break;
                    
                case State::STOPPING :
                    /* std::cout << "Ignored stopping event. Already STOPPING" << std::endl; */
                    break;
            }
            break;
            
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
    

    _current_side = _initial_param.get_first_step_side();
    std::cout << "First step: " << _current_side << std::endl;
    _other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));

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
    double T = 5; //window length for MpC
    double dt = 0.01; //rate of ros
    _t_before_first_step = 0; // preparation time in the first step for the com to swing laterally before stepping 
    
    _q_min = sense_q1(); // min angle of inclination
    _q_max = _initial_param.get_max_inclination(); // max angle of inclination

    _step_duration = _initial_param.get_duration_step();

    _steep_coeff = _q_max/_step_duration;

    std::cout << "Start walk time: " << _initial_param.get_start_time() <<  std::endl;
    std::cout << "Max angle of inclination: " << _q_max <<  std::endl;
    std::cout << "Step duration: " << _step_duration <<  std::endl;
    std::cout << "Double stance: " <<  _initial_param.get_double_stance() << std::endl;
    std::cout << "Steepness: " << _steep_coeff <<  std::endl;
    std::cout << "Real impacts: " << _initial_param.get_switch_real_impact() <<  std::endl;
    

    _com_y << _initial_com_position(1), 0, 0; //com trajectory used by mpc: pos, vel, acc

    int sign_first_stance_step = (_current_pose_ROS.get_sole(_other_side).coeff(1) > 0) - (_current_pose_ROS.get_sole(_other_side).coeff(1) < 0);

    double first_stance_step = _current_pose_ROS.get_sole(_other_side).coeff(1) - sign_first_stance_step * _initial_param.get_indent_zmp();


    // generate zmp given start walk and first stance step
    generate_zmp(first_stance_step, _initial_param.get_start_time(), _initial_param.get_double_stance(), _initial_param.get_max_steps(), dt, _zmp_t, _zmp_y);
    
    // steer zmp
    
    //     generate_zmp(first_stance_step, _initial_param.get_start_time(), _initial_param.get_double_stance(), _initial_param.get_max_steps(), dt, _zmp_t_steer, _zmp_y_steer); //TODO once filled, I shouldn't be able to modify them
    

    // generate different ZMP keeping the ZMP constant
        Eigen::VectorXd point_t(2);
        Eigen::VectorXd point_y_right(2), point_y_left(2);
        
        point_t << 0, 10; //TODO horizon for the delay
        point_y_right << -first_stance_step, -first_stance_step;
        
        lSpline(point_t, point_y_right, dt, _zmp_t_fake_right, _zmp_y_fake_right);
        
        _zmp_y_fake_right[_zmp_y_fake_right.size()-1] = _zmp_y_fake_right[_zmp_y_fake_right.size()-2]; 
        _zmp_t_fake_left = _zmp_t_fake_right;
        _zmp_y_fake_left = -_zmp_y_fake_right;
    
    
    // get impact position planned in time
    _planned_impacts.resize(_initial_param.get_max_steps(),1);
    
    for (int i = 1; i <= _initial_param.get_max_steps(); i++)
    {
        _planned_impacts(i-1) = _initial_param.get_start_time() +  _initial_param.get_duration_step() * i;
    }
    
    for (int i = 0; i < (_zmp_t).size(); i++)
    {
        _logger->add("zmp_t", (_zmp_t)(i));
        _logger->add("zmp_y", (_zmp_y)(i));
    }

    Eigen::MatrixXd Q(1,1);
    Eigen::MatrixXd R(1,1);
    
    Q << _initial_param.get_MPC_Q(); //1000000
    R << _initial_param.get_MPC_R();
    
    
    
    
    _MpC_lat = std::make_shared<item_MpC>(_initial_height, Ts, T, Q, R);
    
    zmp_window(_zmp_t, _zmp_y, 0, _MpC_lat->_window_length + 0, _zmp_window_t, _zmp_window_y);
    
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
        
    std::cout << "Initialization complete." << std::endl;
        
//     _starting_time = time;
    return 1;
                
}
