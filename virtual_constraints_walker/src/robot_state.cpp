#include <virtualConstraintsNode.h>

virtualConstraintsNode::robot_position::robot_position()
    {
    }
  
virtualConstraintsNode::robot_position::robot_position(ros::NodeHandle n)  
    { 
//      prepare subscribers node
        _cartesian_solution_sub = n.subscribe("/cartesian/solution", 10, &virtualConstraintsNode::robot_position::joints_state_callback, this); /*subscribe to cartesian/solution topic*/
        _com_sub = n.subscribe("/cartesian/com/state", 10, &virtualConstraintsNode::robot_position::com_state_callback, this); /*subscribe to cartesian/solution topic*/
        _l_sole_sub = n.subscribe("/cartesian/l_sole/state", 10, &virtualConstraintsNode::robot_position::l_sole_state_callback, this); /*subscribe to cartesian/solution topic*/
        _r_sole_sub = n.subscribe("/cartesian/r_sole/state", 10, &virtualConstraintsNode::robot_position::r_sole_state_callback, this); /*subscribe to cartesian/solution topic*/

//      prepare listener node       
        _ankle_to_com_listener.waitForTransform("ci/l_ankle", "ci/com", ros::Time::now(), ros::Duration(3.0)); /*why here??*/
        _l_to_r_foot_listener.waitForTransform("ci/l_ankle", "ci/r_ankle", ros::Time::now(), ros::Duration(3.0));
    }
    
void virtualConstraintsNode::robot_position::joints_state_callback(const sensor_msgs::JointState msg_rcv) //this is called by ros
    {
//         TODO: here do all state, also orientation
        _joints_state = msg_rcv.position;
    }

void virtualConstraintsNode::robot_position::com_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        Eigen::Affine3d affineMatrix; /*TODO remember to transform everything in affine matrix*/
       
        tf::poseMsgToEigen(msg_rcv.pose, affineMatrix);
        tf::pointMsgToEigen(msg_rcv.pose.position, _com_state);
        std::cout << _com_state.transpose() << std::endl;
    }

void virtualConstraintsNode::robot_position::l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        tf::pointMsgToEigen(msg_rcv.pose.position,_l_sole_state);
    }
    
void virtualConstraintsNode::robot_position::r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        tf::pointMsgToEigen(msg_rcv.pose.position, _r_sole_state);
    }

Eigen::Vector3d virtualConstraintsNode::robot_position::listen_distance_ankle_to_com()
    {
        tf::Vector3 distance;
        Eigen::Vector3d ankle_to_com_distance;
        
        _ankle_to_com_listener.lookupTransform("ci/l_ankle", "ci/com", ros::Time(0), _ankle_to_com_transform);
        distance = _ankle_to_com_transform.getOrigin();
        tf::vectorTFToEigen(distance, ankle_to_com_distance);
        return ankle_to_com_distance;
    }

Eigen::Vector3d virtualConstraintsNode::robot_position::listen_distance_l_to_r_foot()
    {
        tf::Vector3 distance;
        Eigen::Vector3d l_to_r_foot_distance;
        _l_to_r_foot_listener.lookupTransform("ci/l_ankle", "ci/r_ankle", ros::Time(0), _l_to_r_foot_transform);
        distance = _l_to_r_foot_transform.getOrigin();
        tf::vectorTFToEigen(distance, l_to_r_foot_distance);
        return l_to_r_foot_distance;
    }

    
    