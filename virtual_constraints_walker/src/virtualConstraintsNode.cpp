#include <virtualConstraintsNode.h>
#include <atomic>

#define initialized ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    

virtualConstraintsNode::virtualConstraintsNode(int argc, char **argv, const char *node_name) : mainNode(argc, argv, node_name)
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
        _com_state = msg_rcv.pose.position;
    }

void virtualConstraintsNode::l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        _l_sole_state = msg_rcv.pose.position;
    }
    
void virtualConstraintsNode::r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        _r_sole_state = msg_rcv.pose.position;
    }

tf::Vector3 virtualConstraintsNode::listen_distance_ankle_to_com()
    {
        tf::Vector3 ankle_to_com_distance;

        _ankle_to_com_listener.lookupTransform("ci/l_ankle", "ci/com", ros::Time(0), _ankle_to_com_transform);
  
        ankle_to_com_distance = _ankle_to_com_transform.getOrigin();
//         ROS_INFO("var: %f", ankle_to_com_distance.z());
        return ankle_to_com_distance;
    }

tf::Vector3 virtualConstraintsNode::listen_distance_l_to_r_foot()
    {
        tf::Vector3 l_to_r_foot_distance;

        _l_to_r_foot_listener.lookupTransform("ci/l_ankle", "ci/r_ankle", ros::Time(0), _l_to_r_foot_transform);
        
        l_to_r_foot_distance = _l_to_r_foot_transform.getOrigin();
//         ROS_INFO("var: %f", l_to_r_foot_distance.x());
        
        return l_to_r_foot_distance;
    }
    
void virtualConstraintsNode::get_initial_pose()
    {   
        if (initialized) {
            _initial_pose.com = _com_state;
            _initial_pose.l_sole = _l_sole_state;
            _initial_pose.r_sole = _r_sole_state;
            _initial_pose.ankle_to_com = this->listen_distance_ankle_to_com();
//             virtualConstraintsNode::intial_state_robot::com = _com_state;              /*is this bad coding practice? (or wrong, even)*/
//             virtualConstraintsNode::intial_state_robot::l_sole = _l_sole_state;
//             virtualConstraintsNode::intial_state_robot::r_sole = _r_sole_state;
        };
    }
    
double virtualConstraintsNode::get_q1()
    {
        return _q1_state;
    }
    
    

geometry_msgs::PoseStamped virtualConstraintsNode::update_x_position(geometry_msgs::Point current_pose, double update_x) 
    {   
        geometry_msgs::PoseStamped msg_cmd;
        
        msg_cmd.pose.position = current_pose;
        msg_cmd.pose.position.x = current_pose.x + update_x;
        return msg_cmd;
    }

void virtualConstraintsNode::left_move()
    {
       geometry_msgs::PoseStamped cmd_com, cmd_l_sole;
       double x_com_distance, step_distance;
       double q1 = 0;
       
       this->get_initial_pose(); /*TODO as soon as I start the node, it moves! I don't want!*/
       
//        double q1 = -atan(_initial_pose.com.x/_initial_pose.com_heigth.getOrigin().z());
       
       q1 = this->get_q1();
       
       this->step(q1, &x_com_distance, &step_distance);
//        cmd_com = this->update_x_position(_initial_pose.com, this->incline());
//        cmd_r_sole = this->update_x_position(_initial_pose.r_sole, this->step());
       cmd_com = this->update_x_position(_initial_pose.com, x_com_distance);
       cmd_l_sole = this->update_x_position(_initial_pose.l_sole, step_distance);
       
       
//        ROS_INFO("initial_com_pose: %f", _initial_pose.com.x);
//        ROS_INFO("current_com_pose: %f", cmd_com.pose.position.x);
       
//        ROS_INFO("initial_step_pose: %f", _initial_pose.com.pose.position.x);
//        ROS_INFO("current_step_pose: %f", cmd_com.pose.position.x);
//        ROS_INFO("q1: %f", q1);
       _com_pub.publish(cmd_com);
       _l_sole_pub.publish(cmd_l_sole);
       
    } 
    
void virtualConstraintsNode::right_move()
    {
       geometry_msgs::PoseStamped cmd_com, cmd_r_sole;
       double x_com_distance, step_distance;
       tf::Vector3 distance_foots;
       
       this->get_initial_pose(); /*TODO as soon as I start the node, it moves! I don't want!*/
       
//        double q1 = -atan(_initial_pose.com.x/_initial_pose.com_heigth.getOrigin().z());
       
       double q1 = this->get_q1();
 
       this->step(q1, &x_com_distance, &step_distance);
//        cmd_com = this->update_x_position(_initial_pose.com, this->incline());
//        cmd_r_sole = this->update_x_position(_initial_pose.r_sole, this->step());
       cmd_com = this->update_x_position(_initial_pose.com, x_com_distance);
       cmd_r_sole = this->update_x_position(_initial_pose.r_sole, step_distance);
       
       distance_foots = this->listen_distance_l_to_r_foot();
       ROS_INFO("distance is %f", distance_foots.x());
       _com_pub.publish(cmd_com);
       _r_sole_pub.publish(cmd_r_sole);
       
    } 
    
void virtualConstraintsNode::step(double q1, double *x_com_distance, double *step_distance) /*not really nice implementation I guess*/
    {
        tf::Vector3 ankle_to_com_distance;
        ankle_to_com_distance = this->listen_distance_ankle_to_com();
        
        double lenght_leg = ankle_to_com_distance.length();
        
        *x_com_distance = ankle_to_com_distance.z() * tan(q1); /*calc x com distance from given angle q1*/
        *step_distance = 2* lenght_leg * sin(q1); /*calc step distance given q1*/
//         step_distance = 2*(_initial_pose.com.x + *x_com_distance); /*calc step distance constrained to x-position of com*/
//         step_distance = 2*(_com_state.x + *x_com_distance); /*TODO is this right??*/
    }

// void virtualConstraintsNode::impact_detected()
//     {
//         double step_distance;
//         double q1 = this->get_q1();
//         this->step(q1, &x_com_distance, &step_distance);
//         if (_l_sole_state.x ==  0.
//     }
    
    
    
// ===============================================
// double virtualConstraintsNode::incline()
//     {
//         double q1, z_distance, x_distance;
//         
//         z_distance = this->listen_z_distance_ankle_com();
// 
//         q1 = this->get_q1();

// //         q1 = PI/20;
//         x_distance = z_distance * tan(q1);
// 
//         return x_distance;
//     }
//     
// double virtualConstraintsNode::step()
//     {
//         
//         double step_distance;
//         double x_distance = this->incline();
//         
//         step_distance = 2*(_initial_pose.com.x + x_distance);
//            
//         return step_distance;
//          
//     }