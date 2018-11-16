#include <virtualConstraintsNode.h>
#include <atomic>

#define initialized ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    

virtualConstraintsNode::virtualConstraintsNode(int argc, char **argv, const char *node_name) : 
    mainNode(argc, argv, node_name)
    {
        ros::NodeHandle n;
        
        _initial_pose = _current_pose_ROS;
        _step.set_data_step(_current_pose_ROS.get_l_sole(),_current_pose_ROS.get_l_sole(),0,getTime(),getTime()+2);
        

        _q1_sub = n.subscribe("/q1", 10, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 10); /*publish to /cartesian/com/reference*/
        
        _l_sole_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/l_sole/reference", 10); /*publish to /l_sole/reference*/
        _r_sole_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 10); /*publish to /r_sole/reference*/
    }
    
double virtualConstraintsNode::getTime()
    {
        double time = ros::Time::now().toSec();
        return time;
    }
    
int virtualConstraintsNode::straighten_up_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
    {
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac("cartesian/com/reach", true); /*without /goal!!*/
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac.waitForServer(); //will wait for infinite time
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
        
        ac.sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        //exit
        return 0;
}
    
Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
//         std::cout << straight_com.transpose() << std::endl;
        straight_com(0) = 0.0;
        straight_com(2) = -0.2;
        return straight_com;
    }
    
void virtualConstraintsNode::q1_callback(const std_msgs::Float64 msg_rcv) //this is called by ros
    {
       _q1_state = msg_rcv.data;
    }

double virtualConstraintsNode::get_q1()
    {
        return _q1_state;
    }
    
double virtualConstraintsNode::calc_q1()
    {          
       Eigen::Vector3d ankle_to_com, com;
       ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com();
       com = _current_pose_ROS.get_com();
       
       double q1 = atan(com(0)/ankle_to_com(2)); /*calc angle q1*/
       ROS_INFO("q1 is: %f", q1);
       return q1;
    }  


void virtualConstraintsNode::update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update) 
    {   
//         update current_pose with update
        *current_pose = *current_pose + update;
    }
    
void virtualConstraintsNode::calc_step(double q1,  Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step) /* TODO not really nice implementation I guess ???*/
    {
        Eigen::Vector3d ankle_to_com_distance = _current_pose_ROS.get_distance_ankle_to_com();
        
        double lenght_leg = ankle_to_com_distance.norm();
        
        *delta_com << ankle_to_com_distance.z() * tan(q1), 0, 0; /*calc x com distance from given angle q1*/
        *delta_step << 2* lenght_leg * sin(q1), 0, 0; /*calc step distance given q1*/ 
    }

// void virtualConstraintsNode::impact_detected()                
//     {
//         double step_distance;
//         double q1 = this.get_q1();
//         this.calc_step(q1, &x_com_distance, &step_distance);
//         if (_l_sole_state.x ==  0.
//     }

// Eigen::Vector3d virtualConstraintsNode::step_state::get_state()
//     {
//         
//     }

// virtualConstraintsNode::step_state::step_state(robot_position _current_pose_ROS)
//     {
//         _starting_position = _current_pose_ROS.l_sole;
//         _ending_position = _starting_position;
//         _previous_ending_position = _ending_position;
//     }
// 
// Eigen::Vector3d virtualConstraintsNode::step_state::get_state()
//     {
//         
//     }
//                

// void virtualConstraintsNode::idle()
//     {
//         Eigen::Vector3d step_initial_pose =_current_pose_ROS.get_l_sole(); 
//         Eigen::Vector3d step_final_pose = step_initial_pose;
//         double step_start_time, step_final_time;
//         double clearing = 0;
//         double starTime = getTime();
//         double endTime = starTime + 2;
        
//     }
    
void virtualConstraintsNode::update_command_step(Eigen::Vector3d initial_pose, Eigen::Vector3d final_pose, double clearing, double startTime, double endTime)
    {
        double step_clearing;
        Eigen::Vector3d step_initial_pose, step_final_pose;
        double step_start_time, step_final_time;
        step_clearing = clearing;
        
        step_initial_pose = initial_pose;
        step_final_pose = final_pose;
        
        step_start_time = startTime;
        step_final_time = endTime;
        
        
        
    }

bool virtualConstraintsNode::new_q1()
    {
        bool flag_q1 = false;
        double last_q1_step = 0;
        last_q1_step = _q1_step;
        _q1_step = get_q1();
        

        
        if (last_q1_step - _q1_step >= 0.001)
        {
            flag_q1 = true;
        }
        
        ROS_INFO("last q1 = %f", last_q1_step);
        ROS_INFO("q1 = %f", _q1_step);
        ROS_INFO("q1 - q1_last = %f", _q1_step-last_q1_step);
        ROS_INFO("flag %d", flag_q1);
        return flag_q1;
    }
    
void virtualConstraintsNode::update_step()
    {
                ROS_INFO("entered");
                
                Eigen::Vector3d com_position, l_sole_position;
                Eigen::Vector3d delta_com, delta_step;
                com_position = _initial_pose.get_com(); /*if I put current pose, my q1 is relative to the reached pose*/
                l_sole_position = _initial_pose.get_l_sole();
                
                calc_step(_q1_step, &delta_com, &delta_step);
                
                update_position(&com_position, delta_com);       /*incline*/
                update_position(&l_sole_position, delta_step);  /*step*/
                
                std::cout << "starting position: " << _current_pose_ROS.get_l_sole().transpose() << std::endl;
                std::cout << "ending position: " << l_sole_position.transpose() << std::endl;
                
                double clearing = 0.2;
                double starTime = getTime();
                double endTime = starTime + 2;
                
                _step.set_data_step(_current_pose_ROS.get_l_sole(), l_sole_position, clearing, starTime, endTime);
    }
        
void virtualConstraintsNode::calc_trajectory() 
    {
        /*TODO right now if the node q1 starts before the virtualConstraintsNode, something wrong happen*/
        /*TODO COM isn't moving - there is no trajectory for com*/
        /*TODO enter automatically first time, not good*/
        if (new_q1())
        {
        update_step();
        }
        
        Eigen::Vector3d foot_trajectory;
        foot_trajectory = compute_swing_trajectory(_step.get_initial_pose(), _step.get_final_pose(), _step.get_clearing(), _step.get_starTime(), _step.get_endTime(), ros::Time::now().toSec());
        send_step(foot_trajectory);
    }
    
void virtualConstraintsNode::send_step(Eigen::Vector3d command)
    {       
        geometry_msgs::PoseStamped cmd_com, cmd_l_sole;
//      tf::pointEigenToMsg(com_position, cmd_com.pose.position);
        tf::pointEigenToMsg(command, cmd_l_sole.pose.position);
        
//         _com_pub.publish(cmd_com);
        _l_sole_pub.publish(cmd_l_sole);
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

















// void virtualConstraintsNode::foot_position()
//     {
//         this.update_current_pose_ROS();
//         _starting_position = _current_pose_ROS.l_sole;
//         _ending_position = _starting_position;
//         _previous_ending_position = _ending_position;
//     } 



// void virtualConstraintsNode::calc_trajectory()
//     {
// 
//         
//         double last_q1_step = 0;
//         Eigen::Vector3d com_position, l_sole_position;
//         Eigen::Vector3d delta_com, delta_step;
//         this.update_initial_pose();
//         
//         if (_flag == true)
//         {
//             _clearing = 0;
//             _startTime = this.getTime();
//             _endTime = _startTime + 2;
//             this.foot_position();
//             _flag = false;
//         }
//         
// 
// 
//         last_q1_step = _q1_step;
//         _q1_step = this.get_q1();
//         
// 
//         
//         if (last_q1_step !=_q1_step)
//         {
//             ROS_INFO("entered");
//             this._current_pose_ROS;
//             _clearing = 0.1;
//             
//             this.update_current_pose_ROS();
//             com_position = _initial_pose.com; /*if I put current pose, my q1 is relative to the reached pose*/
//             l_sole_position = _initial_pose.l_sole;
//             
//             
// 
//             
//             this.calc_step(_q1_step, &delta_com, &delta_step);
//             
//             this.update_position(&com_position, delta_com);       /*incline*/
//             this.update_position(&l_sole_position, delta_step);  /*step*/
//             
//             
//             _previous_ending_position = _ending_position;
//             
//             
//             _starting_position = _current_pose_ROS.l_sole;
//             _ending_position = l_sole_position;
//             
//             _startTime = this.getTime();
//             _endTime = _startTime + 8;
//         }
// 
//         Eigen::Vector3d foot_trajectory;
//         foot_trajectory = compute_swing_trajectory(_starting_position, _ending_position, _clearing, _startTime, _endTime, ros::Time::now().toSec());
//         
//         geometry_msgs::PoseStamped cmd_com, cmd_l_sole;
//         tf::pointEigenToMsg(com_position, cmd_com.pose.position);
//         tf::pointEigenToMsg(foot_trajectory, cmd_l_sole.pose.position);
// 
//         _com_pub.publish(cmd_com);
//         _l_sole_pub.publish(cmd_l_sole);
//         
//     }