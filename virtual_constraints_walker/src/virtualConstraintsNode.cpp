#include <virtualConstraintsNode.h>
#include <atomic>

#include <XmlRpcValue.h>

#define initialized ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    

virtualConstraintsNode::virtualConstraintsNode(int argc, char **argv, const char *node_name) : 
    mainNode(argc, argv, node_name)
    {

        
        _logger = XBot::MatLogger::getLogger("/tmp/virtual_constraints");
        ros::NodeHandle n;
        
        _step_counter = 0;
        
        param();
        get_param_ros();
        
        _initial_pose = _current_pose_ROS;  
        _step.set_data_step( _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_com(), _current_pose_ROS.get_com(), 0,getTime(),getTime()+2);
        
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
        /*TODO IF NO PARAM are set by user, default values in constructors or params always present and I set them internally?*/
        ros::NodeHandle nh;
        int max_steps;
        double clearance_heigth, duration, drop;
        std::string first_side;

        if (nh.getParam("/virtual_constraints/initial_crouch", drop))
        {
            _initial_param.set_crouch(drop);
        } 
//         else nh.setParam("/virtual_constraints/initial_crouch", drop);
        
        if (nh.getParam("/virtual_constraints/max_steps", max_steps))
        {
            _initial_param.set_max_steps(max_steps);
        }  
//         else nh.setParam("/virtual_constraints/max_steps", max_steps);
        
        if (nh.getParam("/virtual_constraints/clearance_step", clearance_heigth))
        {
            _initial_param.set_clearance_step(clearance_heigth);
        } 
//         else nh.setParam("/virtual_constraints/clearance_step", clearance_heigth);
        
        if (nh.getParam("/virtual_constraints/duration_step", duration))
        {
            _initial_param.set_duration_step(duration);
        }
//         else nh.setParam("/virtual_constraints/duration_step", duration);
        if (nh.getParam("/virtual_constraints/first_step_side", first_side))
        {
            if (first_side == "Left")
                _initial_param.set_first_step_side(robot_interface::Side::Left);
            else if (first_side == "Right")
                _initial_param.set_first_step_side(robot_interface::Side::Right);   
            else std::cout << "unknown side starting command" << std::endl;
        }
    }
            
double virtualConstraintsNode::getTime()
    {ros::NodeHandle n;
        double time = ros::Time::now().toSec();
        return time;
    }


int virtualConstraintsNode::straighten_up_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
    {
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_com("cartesian/com/reach", true); /*without /goal!!*/
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_com.waitForServer(); //will wait for infinite time
        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        cartesian_interface::ReachPoseGoal goal; 
        geometry_msgs::Pose cmd_initial;
        
        tf::pointEigenToMsg(this->straighten_up_goal(), cmd_initial.position);
        float cmd_initial_time;
        cmd_initial_time = 1;
   
        goal.frames.push_back(cmd_initial); /*wants geometry_msgs::Pose*/
        goal.time.push_back(cmd_initial_time);

        ac_com.sendGoal(goal);
    
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
        sense_q1();
        //exit
        return 0;
}

Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
        
        straight_com(0) =  _current_pose_ROS.get_sole(_current_side).coeff(0);
        straight_com(2) = _initial_param.get_crouch();
        _step.set_data_step( _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_sole(_current_side), straight_com, straight_com, 0, getTime(), getTime()+2);
        return straight_com;
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
        _current_pose_ROS.sense(); 
        Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
        robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
        
        swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
        stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
           
        double qlat_stance = atan(stance_ankle_to_com(1)/stance_ankle_to_com(2));
        double qlat_swing = atan(swing_ankle_to_com(1)/swing_ankle_to_com(2));
        
        _logger->add("q_lateral_swing", qlat_swing);
        _logger->add("q_lateral_stance", qlat_stance);
        
        _logger->add("q_lateral_left", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(1)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("q_lateral_right", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(1)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
        
        return qlat_stance;
    }
    
double virtualConstraintsNode::sense_q1()
    {   
        _current_pose_ROS.sense(); 
        Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
        robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
        
        swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
        stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
            
        double q1 = atan(stance_ankle_to_com(0)/stance_ankle_to_com(2));
        double q2 = atan(swing_ankle_to_com(0)/swing_ankle_to_com(2));

        
        _logger->add("left_foot_to_com: ", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("right_foot_to_com", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
        
        _logger->add("q_sagittal_stance", q1);
        _logger->add("q_sagittal_swing", q2);
        
        _logger->add("q_sagittal_left", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(0)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("q_sagittal_right", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(0)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
   
        return q1;
    }  

void virtualConstraintsNode::update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update) 
    {   
//       // update current_pose with update
        *current_pose = *current_pose + update;
    }
    
void virtualConstraintsNode::calc_step(double q1, Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step)
    {
        Eigen::Vector3d com_to_ankle_distance;
        
        com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side); /*if take out minus the robot steps back*/

        /* virtual constraints - very simple */
        *delta_com << - 2* com_to_ankle_distance.z() * tan(q1), 0, 0; /*calc x com distance from given angle q1*/
        *delta_step << - 4* com_to_ankle_distance.z() * tan(q1), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)
            
            
        if (_step_counter == 0)
        {
            *delta_com  << (- com_to_ankle_distance.z() * tan(q1)); //+ _current_pose_ROS.get_com().coeff(0), 0, 0; /*calc x com distance from given angle q1*/
            *delta_step << (- 2* com_to_ankle_distance.z() * tan(q1)); //+ _current_pose_ROS.get_l_sole().coeff(0), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)
        }
                    
        _logger->add("delta_com", *delta_com);
        _logger->add("delta_step", *delta_step);
    }

bool virtualConstraintsNode::impact_detected()                
    {
//
            if (fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-5 &&  
                fabs( _current_pose_ROS.get_sole(_current_side).coeff(0) -  _initial_pose.get_sole(_current_side).coeff(0))>  0.1)
            {

                _current_pose_ROS.sense();

                robot_interface::Side last_side = _current_side;
               _initial_pose = _current_pose_ROS;
               _step_counter++;
                _current_side = robot_interface::Side::Double;
                std::cout << "Impact! Current state: " << _current_side << std::endl;
                

                if (_step_counter < _initial_param.get_max_steps())
                {
                    if (last_side == robot_interface::Side::Left)
                        _current_side = robot_interface::Side::Right;
                    else if (last_side == robot_interface::Side::Right)
                        _current_side = robot_interface::Side::Left;
                    else ROS_INFO("wrong side");
                //                 _current_side = (Side)(1 - _current_side);  /*here I change the Side after the impact */
                    
                std::cout << "State changed. Current side: " << _current_side << std::endl;
                return 1;
                }
                else 
                {
                    return 0;
                }
                
            }
            else
            {
                return 0;
            }
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
                _current_side = _initial_param.get_first_step_side();
                std::cout << "State changed. First side: " << _current_side << std::endl;
            }
                
        }
        ROS_INFO("command received! q1 = %f", _q1_state);
    }
void virtualConstraintsNode::update_step()
    {
        Eigen::Vector3d initial_com_position, initial_sole_position;
        Eigen::Vector3d final_com_position, final_sole_position;
        Eigen::Vector3d delta_com, delta_step;
        
        double delta_com_y;
        
        _current_pose_ROS.sense();
        
        initial_com_position = _current_pose_ROS.get_com();
        final_com_position = _current_pose_ROS.get_com();

        initial_sole_position = _current_pose_ROS.get_sole(_current_side);
        final_sole_position = _current_pose_ROS.get_sole(_current_side);

        calc_step(_q1_state, &delta_com, &delta_step);
        delta_com_y = lateral_com();
        
        delta_com[1] = delta_com_y;
        
        update_position(&final_com_position, delta_com);    /*incline*/
        update_position(&final_sole_position, delta_step);  /*step*/
        
        
        
        double clearing = _initial_param.get_clearance_step();
        double starTime = getTime();
        double endTime = starTime + _initial_param.get_duration_step();
        
        _step.set_data_step(initial_sole_position, final_sole_position, initial_com_position, final_com_position, clearing, starTime, endTime);
    }

double virtualConstraintsNode::lateral_com()
    {
        _current_pose_ROS.sense();
        double qlat = sense_qlat();
        double delta_com_y;
        
        Eigen::Vector3d ankle_to_com_distance;

        ankle_to_com_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side);

        /* virtual constraints - very simple */
        if (_current_side == robot_interface::Side::Double)
        {
            delta_com_y = 0;
        }
        delta_com_y = ankle_to_com_distance.z() * tan(qlat); /*calc x com distance from given angle q1*/
  
                    
        _logger->add("delta_com_lateral", delta_com_y);
        return delta_com_y;
    }
    
void virtualConstraintsNode::run() 
    {
        if (initialized) /*initial tilt*/
        {
            first_q1();
            update_step();
            
        }
        
        sense_qlat();
        sense_q1();
        
        if (impact_detected())
        {
            update_step();
        }
        
        Eigen::Vector3d foot_trajectory, com_trajectory;
        foot_trajectory = compute_swing_trajectory(_step.get_foot_initial_pose(), _step.get_foot_final_pose(), _step.get_clearing(), _step.get_starTime(), _step.get_endTime(), ros::Time::now().toSec());
        com_trajectory = compute_swing_trajectory(_step.get_com_initial_pose(), _step.get_com_final_pose(), 0, _step.get_starTime(), _step.get_endTime(), ros::Time::now().toSec());
        
        _step.log(_logger);
        
        _logger->add("com_trajectory", com_trajectory);
        _logger->add("foot_trajectory", foot_trajectory);
        
        send_step(foot_trajectory, com_trajectory);
    }
    
void virtualConstraintsNode::send_step(Eigen::Vector3d foot_command, Eigen::Vector3d com_command)
    {
        geometry_msgs::PoseStamped cmd_com, cmd_sole;
        tf::pointEigenToMsg(com_command, cmd_com.pose.position);
        tf::pointEigenToMsg(foot_command, cmd_sole.pose.position);
        
        _com_pub.publish(cmd_com);
        _sole_pubs[_current_side].publish(cmd_sole);
    }

Eigen::Vector3d virtualConstraintsNode::compute_swing_trajectory(const Eigen::Vector3d& start, 
                                                                 const Eigen::Vector3d& end, 
                                                                 double clearance,
                                                                 double t_start, 
                                                                 double t_end, 
                                                                 double time, 
                                                                 Eigen::Vector3d* vel,
                                                                 Eigen::Vector3d* acc)
{
    Eigen::Vector3d ret;
    
    double tau = std::min(std::max((time - t_start)/(t_end - t_start), 0.0), 1.0);
    double alpha = compute_swing_trajectory_normalized_xy(time_warp(tau, 2.0));
    
    ret.head<2>() = (1-alpha)*start.head<2>() + alpha*end.head<2>();
    ret.z() = start(2) + compute_swing_trajectory_normalized_z(end.z()/clearance, time_warp(tau, 2.0))*clearance; /*porcodio*/
    
    
    return ret;
}

double virtualConstraintsNode::compute_swing_trajectory_normalized_xy(double tau, double* __dx, double* __ddx)
{
    double x, dx, ddx;
    XBot::Utils::FifthOrderPlanning(0, 0, 0, 1, 0, 1, tau, x, dx, ddx);

//     if(__dx) *__dx = dx;
//     if(__ddx) *__ddx = ddx;
    return x;
}

double virtualConstraintsNode::compute_swing_trajectory_normalized_z(double final_height, double tau, double* dx, double* ddx)
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









// int virtualConstraintsNode::initial_shift_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
//     {
//         actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_com("cartesian/com/reach", true); /*without /goal!!*/
//         actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_step("cartesian/l_sole/reach", true); /*without /goal!!*/
//         double q1 = 0.1; /*STARTING ANGLE!!!!!*/
//         ROS_INFO("Waiting for action server to start.");
//         // wait for the action server to start
//         ac_com.waitForServer();
//         ac_step.waitForServer(); //will wait for infinite time
//         ROS_INFO("Action server started, sending goal.");
//         // send a goal to the action
//         cartesian_interface::ReachPoseGoal goal_com, goal_step;
// //      TODO there is a bug in cartesian/com/reach, adding waypoints won't work     
//         geometry_msgs::Pose cmd_shift_pos, cmd_shift_foot;
//         
//         Eigen::Vector3d pose_com = _current_pose_ROS.get_com();
//         pose_com(0) = (_current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2) * tan(q1)) + _current_pose_ROS.get_com().coeff(0);
//         tf::pointEigenToMsg(pose_com, cmd_shift_pos.position);
//         
//         Eigen::Vector3d pose_step =  _current_pose_ROS.get_sole(_current_side);
//         pose_step(0) = (2 * _current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2) * tan(q1)) + _current_pose_ROS.get_sole(_current_side).coeff(0);
//         tf::pointEigenToMsg(pose_step, cmd_shift_foot.position);
//         
//         float cmd_initial_time;
//         cmd_initial_time = 1;
// 
//         
//         
//         goal_com.frames.push_back(cmd_shift_pos); /*wants geometry_msgs::Pose*/
//         goal_com.time.push_back(cmd_initial_time);
//         
//         goal_step.frames.push_back(cmd_shift_foot); /*wants geometry_msgs::Pose*/
//         goal_step.time.push_back(cmd_initial_time);
//         
//         ac_com.sendGoal(goal_com);
//         ac_step.sendGoal(goal_step);
//     
//         //wait for the action to return
//         bool finished_before_timeout = ac_step.waitForResult(ros::Duration(5.0));
// 
//         if (finished_before_timeout)
//         {
//             actionlib::SimpleClientGoalState state = ac_step.getState();
//             ROS_INFO("Action finished: %s", state.toString().c_str());
//         }
//         else
//             ROS_INFO("Action did not finish before the time out.");
// 
//         /* fill initial pose with pose after straighten_up_action */
// //         _current_pose_ROS.sense();
//         _initial_pose = _current_pose_ROS; 
// //         sense_q1();
//         //exit
//         return 0;
// }