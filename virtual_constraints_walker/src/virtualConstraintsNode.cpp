#include <virtualConstraintsNode.h>

    
virtualConstraintsNode::virtualConstraintsNode(int argc, char **argv, const char *node_name) : mainNode(argc, argv, node_name)
    {    
        ros::NodeHandle n;

        tf::TransformListener ankle_to_com_listener;
        
//      prepare subscribers node
        _cartesian_solution_sub = n.subscribe("/cartesian/solution", 1000, &virtualConstraintsNode::joints_state_callback, this); /*subscribe to cartesian/solution topic*/
        _com_sub = n.subscribe("/cartesian/com/state", 1000, &virtualConstraintsNode::com_state_callback, this); /*subscribe to cartesian/solution topic*/

//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 1000); /*publish to /cartesian/com/reference*/

//      prepare listener node
        ankle_to_com_listener.waitForTransform("ci/l_ankle", "ci/com", ros::Time::now(), ros::Duration(3.0));
        ankle_to_com_listener.lookupTransform("ci/l_ankle", "ci/com", ros::Time(0), _ankle_to_com_transform);
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
    
double virtualConstraintsNode::get_q1()
    {
        double q1_state;
        q1_state = _joints_state[_joint_number];
    
        ROS_INFO("var is %f", q1_state);
        return q1_state;
    }
    
    
//     virtual_constraints::calc_VC_legs ()
//     {
//         double pos;
// //         pos = 2 * leg_length * sin(q1_state);
//         pos = -0.2;
//         return pos;
//     }
    
    
void virtualConstraintsNode::publish_x_position_com(double pos) 
    {
        geometry_msgs::PoseStamped msg_cmd;
        msg_cmd.pose.position = _com_state;
        msg_cmd.pose.position.x = _com_state.x + pos; //wrong
//         ROS_INFO("%f", _com_state);
//         ROS_INFO("%f", msg_cmd.pose.position.x);
        _com_pub.publish(msg_cmd);
    }
    
double virtualConstraintsNode::listen_z_distance_ankle_com()
    {
        double z_distance;
        z_distance = _ankle_to_com_transform.getOrigin().z();
//         ROS_INFO("val is: %f",  _distance);
        return z_distance;
        
    }
    
void virtualConstraintsNode::run()
    {
        double q1, z_distance, pos;
        
        z_distance = this->listen_z_distance_ankle_com();
        ROS_INFO("distance_z: %f",  z_distance);
//         q1 = this->get_q1();
        q1 = PI/20;

        pos = z_distance * tan(q1);
        ROS_INFO("distance_x: %f",  pos);
        this->publish_x_position_com(pos);
        
    }