#include <virtualConstraintsNode.h>
#include <atomic>

#define initialized ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    

virtualConstraintsNode::virtualConstraintsNode(int argc, char **argv, const char *node_name) : 
    mainNode(argc, argv, node_name)
    {
        ros::NodeHandle n;
 
//      prepare subscribers node
        _cartesian_solution_sub = n.subscribe("/cartesian/solution", 1000, &virtualConstraintsNode::joints_state_callback, this); /*subscribe to cartesian/solution topic*/
        _com_sub = n.subscribe("/cartesian/com/state", 1000, &virtualConstraintsNode::com_state_callback, this); /*subscribe to cartesian/solution topic*/
        _l_sole_sub = n.subscribe("/cartesian/l_sole/state", 1000, &virtualConstraintsNode::l_sole_state_callback, this); /*subscribe to cartesian/solution topic*/
        _r_sole_sub = n.subscribe("/cartesian/r_sole/state", 1000, &virtualConstraintsNode::r_sole_state_callback, this); /*subscribe to cartesian/solution topic*/
        
        _q1_sub = n.subscribe("/q1", 1000, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 1000); /*publish to /cartesian/com/reference*/
        
        _l_sole_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/l_sole/reference", 1000); /*publish to /l_sole/reference*/
        _r_sole_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 1000); /*publish to /r_sole/reference*/
        
//      prepare listener node
       
        _ankle_to_com_listener.waitForTransform("ci/l_ankle", "ci/com", ros::Time::now(), ros::Duration(3.0)); /*why here??*/
        _l_to_r_foot_listener.waitForTransform("ci/l_ankle", "ci/r_ankle", ros::Time::now(), ros::Duration(3.0));
        
//      prepare action


    }
// virtualConstraintsNode::State virtualConstraintsNode::State::getState()
//     {
// 
//     }
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
        Eigen::Vector3d straight_com = _com_state;
        straight_com(0) = 0.0;
        straight_com(2) = -0.2;
        return straight_com;
    }
    
void virtualConstraintsNode::q1_callback(const std_msgs::Float64 msg_rcv)
    {
       _q1_state = msg_rcv.data;
    }
void virtualConstraintsNode::joints_state_callback(const sensor_msgs::JointState msg_rcv) //this is called by ros
    {
//         TODO: here do all state, also orientation
        _joints_state = msg_rcv.position;
    }

void virtualConstraintsNode::com_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        Eigen::Affine3d affineMatrix;
       
        tf::poseMsgToEigen(msg_rcv.pose, affineMatrix);
        tf::pointMsgToEigen(msg_rcv.pose.position, _com_state);
    }

void virtualConstraintsNode::l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        tf::pointMsgToEigen(msg_rcv.pose.position,_l_sole_state);
    }
    
void virtualConstraintsNode::r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        tf::pointMsgToEigen(msg_rcv.pose.position, _r_sole_state);
    }

Eigen::Vector3d virtualConstraintsNode::listen_distance_ankle_to_com()
    {
        tf::Vector3 distance;
        Eigen::Vector3d ankle_to_com_distance;
        
        _ankle_to_com_listener.lookupTransform("ci/l_ankle", "ci/com", ros::Time(0), _ankle_to_com_transform);
        distance = _ankle_to_com_transform.getOrigin();
        tf::vectorTFToEigen(distance, ankle_to_com_distance);
        return ankle_to_com_distance;
    }

Eigen::Vector3d virtualConstraintsNode::listen_distance_l_to_r_foot()
    {
        tf::Vector3 distance;
        Eigen::Vector3d l_to_r_foot_distance;
        _l_to_r_foot_listener.lookupTransform("ci/l_ankle", "ci/r_ankle", ros::Time(0), _l_to_r_foot_transform);
        distance = _l_to_r_foot_transform.getOrigin();
        tf::vectorTFToEigen(distance, l_to_r_foot_distance);
        return l_to_r_foot_distance;
    }
    
void virtualConstraintsNode::get_initial_pose()
    {   
            if (initialized)
            {
            _initial_pose.com = _com_state;
            _initial_pose.l_sole = _l_sole_state;
            _initial_pose.r_sole = _r_sole_state;
            _initial_pose.ankle_to_com = this->listen_distance_ankle_to_com();
//             virtualConstraintsNode::intial_state_robot::com = _com_state;              /*is this bad coding practice? (or wrong, even)*/
//             virtualConstraintsNode::intial_state_robot::l_sole = _l_sole_state;
//             virtualConstraintsNode::intial_state_robot::r_sole = _r_sole_state;
            
            };
    }

void virtualConstraintsNode::get_current_pose()
    {   

            _current_pose.com = _com_state;
            _current_pose.l_sole = _l_sole_state;
            _current_pose.r_sole = _r_sole_state;
            _current_pose.ankle_to_com = this->listen_distance_ankle_to_com();

    }
    
double virtualConstraintsNode::get_q1()
    {
        return _q1_state;
    }
    
double virtualConstraintsNode::calc_q1()
    {          
       Eigen::Vector3d ankle_to_com;
       ankle_to_com = this->listen_distance_ankle_to_com();
       double q1 = atan(_com_state(0)/ankle_to_com(2)); /*calc angle q1*/
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
        Eigen::Vector3d ankle_to_com_distance = this->listen_distance_ankle_to_com();
        
        double lenght_leg = ankle_to_com_distance.norm();
        
        *delta_com << ankle_to_com_distance.z() * tan(q1), 0, 0; /*calc x com distance from given angle q1*/
        *delta_step << 2* lenght_leg * sin(q1), 0, 0; /*calc step distance given q1*/ 
    }

// void virtualConstraintsNode::impact_detected()
//     {
//         double step_distance;
//         double q1 = this->get_q1();
//         this->calc_step(q1, &x_com_distance, &step_distance);
//         if (_l_sole_state.x ==  0.
//     }

// Eigen::Vector3d virtualConstraintsNode::step_state::get_state()
//     {
//         
//     }


void virtualConstraintsNode::foot_position()
    {
        
        this->get_current_pose();
        _starting_position = _current_pose.l_sole;
        _ending_position = _starting_position;
        _previous_ending_position = _ending_position;
    } 
    
void virtualConstraintsNode::calc_trajectory() 
    {
        /*TODO right now if the node q1 starts before the virtualConstraintsNode, something wrong happen*/
        /*TODO COM isn't moving - there is no trajectory for com*/
        /*TODO enter automatically first time, not good*/
        
        double last_q1_step = 0;
        Eigen::Vector3d com_position, l_sole_position;
        Eigen::Vector3d delta_com, delta_step;
        this->get_initial_pose();
        
        if (_flag == true)
        {
            _clearing = 0;
            _startTime = this->getTime();
            _endTime = _startTime + 2;
            this->foot_position();
            _flag = false;
        }
        


        last_q1_step = _q1_step;
        _q1_step = this->get_q1();
        

        
        if (last_q1_step !=_q1_step)
        {
            ROS_INFO("entered");
            this->_current_pose;
            _clearing = 0.1;
            
            this->get_current_pose();
            com_position = _initial_pose.com; /*if I put current pose, my q1 is relative to the reached pose*/
            l_sole_position = _initial_pose.l_sole;
            
            

            
            this->calc_step(_q1_step, &delta_com, &delta_step);
            
            this->update_position(&com_position, delta_com);       /*incline*/
            this->update_position(&l_sole_position, delta_step);  /*step*/
            
            
            _previous_ending_position = _ending_position;
            
            
            _starting_position = _current_pose.l_sole;
            _ending_position = l_sole_position;
            
            _startTime = this->getTime();
            _endTime = _startTime + 8;
        }

        Eigen::Vector3d foot_trajectory;
        foot_trajectory = compute_swing_trajectory(_starting_position, _ending_position, _clearing, _startTime, _endTime, ros::Time::now().toSec());
        
        geometry_msgs::PoseStamped cmd_com, cmd_l_sole;
        tf::pointEigenToMsg(com_position, cmd_com.pose.position);
        tf::pointEigenToMsg(foot_trajectory, cmd_l_sole.pose.position);

        _com_pub.publish(cmd_com);
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
//         this->get_current_pose();
//         _starting_position = _current_pose.l_sole;
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
//         this->get_initial_pose();
//         
//         if (_flag == true)
//         {
//             _clearing = 0;
//             _startTime = this->getTime();
//             _endTime = _startTime + 2;
//             this->foot_position();
//             _flag = false;
//         }
//         
// 
// 
//         last_q1_step = _q1_step;
//         _q1_step = this->get_q1();
//         
// 
//         
//         if (last_q1_step !=_q1_step)
//         {
//             ROS_INFO("entered");
//             this->_current_pose;
//             _clearing = 0.1;
//             
//             this->get_current_pose();
//             com_position = _initial_pose.com; /*if I put current pose, my q1 is relative to the reached pose*/
//             l_sole_position = _initial_pose.l_sole;
//             
//             
// 
//             
//             this->calc_step(_q1_step, &delta_com, &delta_step);
//             
//             this->update_position(&com_position, delta_com);       /*incline*/
//             this->update_position(&l_sole_position, delta_step);  /*step*/
//             
//             
//             _previous_ending_position = _ending_position;
//             
//             
//             _starting_position = _current_pose.l_sole;
//             _ending_position = l_sole_position;
//             
//             _startTime = this->getTime();
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