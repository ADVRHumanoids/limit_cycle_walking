#include <virtualConstraintsNode.h>
#include <atomic>

#define initialized ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    

virtualConstraintsNode::virtualConstraintsNode(int argc, char **argv, const char *node_name) : mainNode(argc, argv, node_name)
    {
        ros::NodeHandle n;

        tf::TransformListener ankle_to_com_listener;
        
//      prepare subscribers node
        _cartesian_solution_sub = n.subscribe("/cartesian/solution", 1000, &virtualConstraintsNode::joints_state_callback, this); /*subscribe to cartesian/solution topic*/
        _com_sub = n.subscribe("/cartesian/com/state", 1000, &virtualConstraintsNode::com_state_callback, this); /*subscribe to cartesian/solution topic*/
        _l_sole_sub = n.subscribe("/cartesian/l_sole/state", 1000, &virtualConstraintsNode::l_sole_state_callback, this); /*subscribe to cartesian/solution topic*/
        _r_sole_sub = n.subscribe("/cartesian/r_sole/state", 1000, &virtualConstraintsNode::r_sole_state_callback, this); /*subscribe to cartesian/solution topic*/
        _q1_sub = n.subscribe("/q1", 1000, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 1000); /*publish to /cartesian/com/reference*/
        _r_sole_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 1000); /*publish to /cartesian/com/reference*/

//      prepare listener node
        ankle_to_com_listener.waitForTransform("ci/l_ankle", "ci/com", ros::Time::now(), ros::Duration(3.0));
        ankle_to_com_listener.lookupTransform("ci/l_ankle", "ci/com", ros::Time(0), _ankle_to_com_transform);
        
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
    
void virtualConstraintsNode::get_initial_pose()
    {   
        if (initialized) {
            _initial_pose.com = _com_state;
            _initial_pose.l_sole = _l_sole_state;
            _initial_pose.r_sole = _r_sole_state;
//             virtualConstraintsNode::intial_state_robot::com = _com_state; 
//             virtualConstraintsNode::intial_state_robot::l_sole = _l_sole_state;
//             virtualConstraintsNode::intial_state_robot::r_sole = _r_sole_state;
        };
    }
    
double virtualConstraintsNode::get_q1()
    {
//         double q1_state;
//         q1_state = _joints_state[_joint_number];
    
//         ROS_INFO("var is %f", _q1_state);
        return _q1_state;
    }
    
    
    
void virtualConstraintsNode::publish_x_position_com() 
    {   
        double pos = this->incline();
        
        geometry_msgs::PoseStamped msg_cmd;
        this->get_initial_pose(); /*TODO how not to call this one everytime but make it run once when the class is created?*/
        
//         ROS_INFO("initial com_state: %f", _initial_pose.pose.position.x);
        msg_cmd.pose.position = _initial_pose.com;
        msg_cmd.pose.position.x = _initial_pose.com.x + pos;
        
//         ROS_INFO("initial_com_state: %f", _initial_pose.com.x);
//         ROS_INFO("com_state: %f", _com_state.x);
//         ROS_INFO("new com_state: %f",  msg_cmd.pose.position.x);

        _com_pub.publish(msg_cmd);
    }
    
void virtualConstraintsNode::publish_x_position_r_sole()
    {
        double pos = this->step();
        
        geometry_msgs::PoseStamped msg_cmd;
        this->get_initial_pose();
        
        msg_cmd.pose.position = _initial_pose.r_sole;
        msg_cmd.pose.position.x = _initial_pose.r_sole.x + pos;
        
        _r_sole_pub.publish(msg_cmd);
               
    }
    
double virtualConstraintsNode::listen_z_distance_ankle_com()
    {
        double z_distance;
        z_distance = _ankle_to_com_transform.getOrigin().z();
//         ROS_INFO("z distance from ankle to com is: %f", z_distance);
        return z_distance;
        
    }
    
double virtualConstraintsNode::incline()
    {
        double q1, z_distance, x_distance;
        
        z_distance = this->listen_z_distance_ankle_com();

        q1 = this->get_q1();
//         q1 = PI/20;
        x_distance = z_distance * tan(q1);

        return x_distance;
    }
    
    
    
double virtualConstraintsNode::step()
    {
        this->get_initial_pose();
        
        double step_distance;
        double x_distance = this->incline();
        
        step_distance = 2*(_initial_pose.com.x + x_distance);
        
//         ROS_INFO("initial_com_state: %f", _initial_pose.com.x);
//         ROS_INFO("com_state: %f", _com_state.x);
//         ROS_INFO("disance_x: %f", x_distance);
//         ROS_INFO("com_state + x_distance: %f", _com_state.x + x_distance);
//         ROS_INFO("step_distance: %f", step_distance);
//         ROS_INFO("----------------------------------");
//            ROS_INFO("initial_r_sole_state_x %f", _initial_pose.r_sole.x);
//            ROS_INFO("initial_r_sole_state_y %f", _initial_pose.r_sole.y);
//            ROS_INFO("initial_r_sole_state_z %f", _initial_pose.r_sole.z);
           
        return step_distance;
         
    }