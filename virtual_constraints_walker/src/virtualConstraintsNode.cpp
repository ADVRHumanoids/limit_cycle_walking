#include <virtualConstraintsNode.h>
#include <atomic>

#define initialized ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    

virtualConstraintsNode::virtualConstraintsNode(int argc, char **argv, const char *node_name) : 
    mainNode(argc, argv, node_name)
    {
        _logger = XBot::MatLogger::getLogger("/tmp/virtual_constraints");
        ros::NodeHandle n;
        
        _step_counter = 0;
        
        _initial_pose = _current_pose_ROS;  
//         _previous_initial_pose = _current_pose_ROS;
        
        _step.set_data_step(_current_pose_ROS.get_l_sole(),_current_pose_ROS.get_l_sole(), _current_pose_ROS.get_com(), _current_pose_ROS.get_com(), 0,getTime(),getTime()+2);
        
        _q1_state = sense_q1();
        
        _terrain_heigth = _initial_pose.get_l_sole().coeff(2);
        
//      prepare subscriber node
        _q1_sub = n.subscribe("/q1", 10, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 10); /*publish to /cartesian/com/reference*/
        
        _l_sole_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/l_sole/reference", 10); /*publish to /l_sole/reference*/
        _r_sole_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 10); /*publish to /r_sole/reference*/
        
        
//         _check_received = false;       %nope because it could be disconnected from q1
//         while (!_check_received)
//         {
//           ros::spinOnce();
//           ROS_INFO("%d", _check_received);
//         }
    }
    
double virtualConstraintsNode::getTime()
    {ros::NodeHandle n;
        double time = ros::Time::now().toSec();
        return time;
    }

int virtualConstraintsNode::initial_tilt_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
    {
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_com("cartesian/com/reach", true); /*without /goal!!*/
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_com.waitForServer(); //will wait for infinite time
        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        cartesian_interface::ReachPoseGoal goal;  
        geometry_msgs::Pose cmd_initial;
//         _current_pose_ROS.sense();
        Eigen::Vector3d pose = _current_pose_ROS.get_com();
        pose(0) = (_current_pose_ROS.get_distance_l_ankle_to_com().coeff(2) * tan(0.2)) + _current_pose_ROS.get_com().coeff(0);
        
        tf::pointEigenToMsg(pose, cmd_initial.position);
        
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
//         _current_pose_ROS.sense();
//         _initial_pose = _current_pose_ROS; 
        sense_q1();
        //exit
        return 0;
}

int virtualConstraintsNode::initial_stride_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
    {
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_step("cartesian/l_sole/reach", true); /*without /goal!!*/
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_step.waitForServer(); //will wait for infinite time
        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        cartesian_interface::ReachPoseGoal goal;
//      TODO there is a bug in cartesian/com/reach, adding waypoints won't work     
        geometry_msgs::Pose cmd_initial;
//         _current_pose_ROS.sense();
        Eigen::Vector3d pose = _current_pose_ROS.get_l_sole();
        pose(0) = (2 * _current_pose_ROS.get_distance_l_ankle_to_com().coeff(2) * tan(0.2)) + _current_pose_ROS.get_l_sole().coeff(0);
        tf::pointEigenToMsg(pose, cmd_initial.position);
        float cmd_initial_time;
        cmd_initial_time = 1;

        
        
        goal.frames.push_back(cmd_initial); /*wants geometry_msgs::Pose*/
        goal.time.push_back(cmd_initial_time);
        
        ac_step.sendGoal(goal);
    
        //wait for the action to return
        bool finished_before_timeout = ac_step.waitForResult(ros::Duration(5.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_step.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        /* fill initial pose with pose after straighten_up_action */
//         _current_pose_ROS.sense();
        _initial_pose = _current_pose_ROS; 
//         sense_q1();
        //exit
        return 0;
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
//      TODO there is a bug in cartesian/com/reach, adding waypoints won't work     
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
//         _initial_pose = _current_pose_ROS; 
        sense_q1();
        //exit
        return 0;
}

Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
//         std::cout << straight_com.transpose() << std::endl;
        straight_com(0) = _current_pose_ROS.get_l_sole().coeff(0);
        straight_com(2) = -0.2;
        _step.set_data_step(_current_pose_ROS.get_l_sole(),_current_pose_ROS.get_l_sole(), straight_com, straight_com, 0, getTime(), getTime()+2);
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
    
double virtualConstraintsNode::sense_q1()
    {   
        _current_pose_ROS.sense(); 
        Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;

        if (_current_side == Side::Left)
        {
            swing_ankle_to_com = _current_pose_ROS.get_distance_l_ankle_to_com();
            stance_ankle_to_com = _current_pose_ROS.get_distance_r_ankle_to_com();
        }
        else if (_current_side == Side::Right)
        {
            swing_ankle_to_com = _current_pose_ROS.get_distance_r_ankle_to_com(); 
            stance_ankle_to_com = _current_pose_ROS.get_distance_l_ankle_to_com();
        }
        else ROS_INFO("wrong side");
//         com = _current_pose_ROS.get_com() - _initial_pose.get_com();
       
//         com = _initial_pose.get_com();
        double q1 = atan(swing_ankle_to_com(0)/swing_ankle_to_com(2)); /*calc angle q1*/
        double q2 = atan(stance_ankle_to_com(0)/stance_ankle_to_com(2));

//         double q1 = atan2(swing_ankle_to_com(2),swing_ankle_to_com(0)); /*calc angle q1*/
//         double q2 = atan2(stance_ankle_to_com(2),stance_ankle_to_com(0));
        
        std::cout << "q1 (from swing) is: " << q1 << std::endl;
        std::cout << "q2 (from stance) is: " << q2 << std::endl;
        
        _logger->add("distance_r_ankle_to_com", _current_pose_ROS.get_distance_r_ankle_to_com());
        _logger->add("distance_l_ankle_to_com", _current_pose_ROS.get_distance_l_ankle_to_com());
        _logger->add("q1", q1);
        _logger->add("q2", q2);
        
        return q1;
    }  


void virtualConstraintsNode::update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update) 
    {   
//       // update current_pose with update
//         std::cout << "previous position: "<< *current_pose << std::endl;
        *current_pose = *current_pose + update;
//         std::cout << "updated position: "<< *current_pose << std::endl;
    }
    
void virtualConstraintsNode::calc_step(double q1, Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step) /* TODO not really nice implementation I guess ???*/
    {
//         _current_pose_ROS.sense();
        Eigen::Vector3d ankle_to_com_distance;
        
        if (_current_side == Side::Left)
            ankle_to_com_distance = _current_pose_ROS.get_distance_l_ankle_to_com();
        else if (_current_side == Side::Right)
            ankle_to_com_distance = _current_pose_ROS.get_distance_r_ankle_to_com();
        

//         double lenght_leg = (ankle_to_com_distance.cwiseProduct(Eigen::Vector3d(1,0,1))).norm();
//         *delta_com << lenght_leg * sin(q1), 0, -(lenght_leg - lenght_leg * cos(q1));
//         *delta_step << 2 * lenght_leg * sin(q1), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)

        /* virtual constraints - very simple */
        *delta_com << 2* ankle_to_com_distance.z() * tan(fabs(q1)), 0, 0; /*calc x com distance from given angle q1*/
//         *delta_step << 2* lenght_leg * sin(q1), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)
        *delta_step << 4* ankle_to_com_distance.z() * tan(fabs(q1)), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)
        
        _logger->add("delta_com", *delta_com);
        _logger->add("delta_step", *delta_step);
        
        std::cout << *delta_com << std::endl;
        std::cout << *delta_step << std::endl;
//         std::cout << "distance: " << ankle_to_com_distance.z() << std::endl;
//         std::cout << "leg length: " << lenght_leg << std::endl;
//         std::cout << "d = h * tg(q1): " << ankle_to_com_distance.z() * tan(q1) << std::endl;
//         std::cout << "d = l * sin(q1): " <<  lenght_leg * sin(q1) << std::endl;
        
//         double x = (*delta_com)[0];
//         double l = sqrt(x*x + ankle_to_com_distance.z()*ankle_to_com_distance.z());
//         std::cout << "d_enovo= l * sin(q1): " <<  l * sin(q1) << std::endl;
    }

bool virtualConstraintsNode::impact_detected()                
    {
//         double q1_walk = sense_q1();
//         double difference = 0.2 - fabs(q1_walk);
//         _logger->add("difference", difference);
        
        
        if (_current_side == Side::Left)
        {   
//             if (fabs(q1_walk) >= 0.2 &&
            if (fabs(fabs(_current_pose_ROS.get_l_sole().coeff(2)) - fabs(_terrain_heigth)) <= 1e-6 &&  
                fabs(_current_pose_ROS.get_l_sole().coeff(0) - _initial_pose.get_l_sole().coeff(0))>  0.1)
            {
                ROS_INFO("state changed: RIGHT");
                _step_counter++;
                double q1 = sense_q1();
                if (q1 <= 0)
                    _q1_state = fabs(q1); /* update q1 */
               std::cout << _q1_state << std::endl;
                    
                _current_pose_ROS.sense();
                _initial_pose = _current_pose_ROS;
//                 ROS_INFO("new left position: %f", _initial_pose.get_l_sole().coeff(0))
//                 ROS_INFO("new right position: %f", _initial_pose.get_r_sole().coeff(0))
                _current_side = Side::Right;
                return 1;
            }
        }
        else if (_current_side == Side::Right)
        {
//             if (fabs(q1_walk) >= 0.2 &&
            if (fabs(fabs(_current_pose_ROS.get_r_sole().coeff(2)) - fabs(_terrain_heigth)) <= 1e-6 &&  
                fabs(_current_pose_ROS.get_r_sole().coeff(0) - _initial_pose.get_r_sole().coeff(0)) >  0.1)
            {  
                ROS_INFO("state changed: LEFT");
                _step_counter++;
                double q1 = sense_q1();
                if (q1 <= 0)
                    _q1_state = fabs(q1);
                
//                 _previous_initial_pose = _initial_pose;
                _initial_pose = _current_pose_ROS;
                _current_side = Side::Left;
                
                return 1;
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
        
//         if (last_q1_step != _q1_step)
        if (fabs(last_q1_step - _q1_step) >= 1e-10) /* TODO this is very sad*/ /*it's a problem of initialization*/
        {
            flag_q1 = true;
        }

        return flag_q1;
    }
    
void virtualConstraintsNode::first_q1()
    {
        ROS_INFO("waiting for command...");
        while (!_check_received)
        {
            _current_pose_ROS.sense();
            
            if (_check_received)
                _q1_state = _q1_cmd;
        }
        ROS_INFO("command received! q1 = %f", _q1_state);
    }
    
void virtualConstraintsNode::update_step_left()
    {
        // when a new q1 is detected, this function updates the step
        ROS_INFO("entered_left");
        
        Eigen::Vector3d com_position, l_sole_position;
        Eigen::Vector3d delta_com, delta_step;
        com_position = _initial_pose.get_com(); /*if I put current pose, my q1 is relative to the reached pose*/
        l_sole_position = _initial_pose.get_l_sole();
        
//         std::cout << "porcodio: " << _q1_state << std::endl;
        
        calc_step(_q1_state, &delta_com, &delta_step);
        
        update_position(&com_position, delta_com);      /*incline*/
        update_position(&l_sole_position, delta_step);  /*step*/
        
        double clearing = 0.1;
        double starTime = getTime();
        double endTime = starTime + 2;
        
        std::cout << "starting position: " << _initial_pose.get_r_sole().transpose() << std::endl;
        std::cout << "ending position: " << l_sole_position.transpose() << std::endl;
        
        std::cout << "starting com: " << _initial_pose.get_com().transpose() << std::endl;
        std::cout << "ending com: " << com_position.transpose() << std::endl;
        
        _step.set_data_step(_current_pose_ROS.get_l_sole(), l_sole_position, _current_pose_ROS.get_com(), com_position, clearing, starTime, endTime);
    }
    
void virtualConstraintsNode::update_step_right()
    {
// when a new q1 is detected, this function updates the step
        ROS_INFO("entered_right");
        
        Eigen::Vector3d com_position, r_sole_position;
        Eigen::Vector3d delta_com, delta_step;
        com_position = _initial_pose.get_com(); /*if I put current pose, my q1 is relative to the reached pose*/ /*_initial_pose*/
        r_sole_position = _initial_pose.get_r_sole();
        
        calc_step(_q1_state, &delta_com, &delta_step);
        
        update_position(&com_position, delta_com);      /*incline*/
        update_position(&r_sole_position, delta_step);  /*step*/
        
        std::cout << "starting position: " << _initial_pose.get_r_sole().transpose() << std::endl;
        std::cout << "ending position: " << r_sole_position.transpose() << std::endl;
//         
        std::cout << "starting com: " << _initial_pose.get_com().transpose() << std::endl;
        std::cout << "ending com: " << com_position.transpose() << std::endl;
        
        
        double clearing = 0.1;
        double starTime = getTime();
        double endTime = starTime + 2;
        
        _step.set_data_step(_current_pose_ROS.get_r_sole(), r_sole_position, _current_pose_ROS.get_com(), com_position,  clearing, starTime, endTime);
    }
        
void virtualConstraintsNode::run() 
    {
//         if (initialized) /*initial tilt*/
//         {
//             
//             first_q1();
//             if (_current_side == Side::Left)
//             {
//                 update_step_left();
//             }
//             else if (_current_side == Side::Right)
//                 update_step_right();
//             else ROS_INFO("wrong update, exiting");
//         }
        
        sense_q1();
        
//         if (0)
        if (impact_detected() || initialized)
        {
                if (_current_side == Side::Left)
                    update_step_left();
                else if (_current_side == Side::Right)
                    update_step_right();
                else ROS_INFO("wrong update, exiting");
        }
        

//         while (_step_counter >=1);  
        
        Eigen::Vector3d foot_trajectory, com_trajectory;
        foot_trajectory = compute_swing_trajectory(_step.get_foot_initial_pose(), _step.get_foot_final_pose(), _step.get_clearing(), _step.get_starTime(), _step.get_endTime(), ros::Time::now().toSec());
        com_trajectory = compute_swing_trajectory(_step.get_com_initial_pose(), _step.get_com_final_pose(), 0, _step.get_starTime(), _step.get_endTime(), ros::Time::now().toSec());
        
        _step.log(_logger);
        
        if (_current_side == Side::Left)
        {
            send_step_left(foot_trajectory, com_trajectory);
        }
        else if (_current_side == Side::Right)
        {
            send_step_right(foot_trajectory, com_trajectory);
        }
        
    }
    
void virtualConstraintsNode::send_step_left(Eigen::Vector3d foot_command, Eigen::Vector3d com_command)
    {       
        geometry_msgs::PoseStamped cmd_com, cmd_l_sole;
        tf::pointEigenToMsg(foot_command, cmd_l_sole.pose.position);
        tf::pointEigenToMsg(com_command, cmd_com.pose.position);
        
        _com_pub.publish(cmd_com);
        _l_sole_pub.publish(cmd_l_sole);
    }

void virtualConstraintsNode::send_step_right(Eigen::Vector3d foot_command, Eigen::Vector3d com_command)
    {       
        geometry_msgs::PoseStamped cmd_com, cmd_r_sole;
        tf::pointEigenToMsg(foot_command, cmd_r_sole.pose.position);
        tf::pointEigenToMsg(com_command, cmd_com.pose.position);    
        
        _com_pub.publish(cmd_com);
        _r_sole_pub.publish(cmd_r_sole);
    }
// void virtualConstraintsNode::send_trajectory(Eigen::Vector3d foot_trajectory)
//     {
//         
//         
//         geometry_msgs::PoseStamped cmd_com, cmd_l_sole;
// //         tf::pointEigenToMsg(com_position, cmd_com.pose.position);
//         tf::pointEigenToMsg(foot_trajectory, cmd_l_sole.pose.position);
// //         _com_pub.publish(cmd_com);
//         
//         _l_sole_pub.publish(cmd_l_sole);
//         
//     }

void virtualConstraintsNode::send_point()
    {
        geometry_msgs::PoseStamped cmd_l_sole;
        Eigen::Vector3d command(0.2, 0.1, -0.9);
        tf::pointEigenToMsg(command, cmd_l_sole.pose.position);
        _l_sole_pub.publish(cmd_l_sole);
        
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