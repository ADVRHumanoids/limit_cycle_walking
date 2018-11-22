// #include <virtualConstraintsNode.h>
#include <robot_interface_ROS.h>
#include <atomic>

#define COUNT_ONCE ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    
    
robot_interface_ROS::robot_interface_ROS()
    { 
        ros::NodeHandle n;
        
//      prepare subscribers node
        _subs.push_back(n.subscribe("/cartesian/solution", 10, &robot_interface_ROS::joints_state_callback, this)); /*subscribe to cartesian/solution topic*/
        _subs.push_back(n.subscribe("/cartesian/com/state", 10, &robot_interface_ROS::com_state_callback, this)); /*subscribe to cartesian/solution topic*/
        _subs.push_back(n.subscribe("/cartesian/l_sole/state", 10, &robot_interface_ROS::l_sole_state_callback, this)); /*subscribe to cartesian/solution topic*/
        _subs.push_back(n.subscribe("/cartesian/r_sole/state", 10, &robot_interface_ROS::r_sole_state_callback, this)); /*subscribe to cartesian/solution topic*/

//      prepare listener node       
        l_ankle_to_com_listener.waitForTransform("ci/l_sole", "ci/com", ros::Time(0), ros::Duration(3.0)); /*ros::Time::now()*/
        r_ankle_to_com_listener.waitForTransform("ci/r_sole", "ci/com", ros::Time(0), ros::Duration(3.0)); /*ros::Time::now()*/
        _l_to_r_foot_listener.waitForTransform("ci/l_sole", "ci/r_sole", ros::Time(0), ros::Duration(3.0));
        

         bool all_check = false;

        
        ROS_INFO("waiting for robot..");
//         while (_callback_counter < _subs.size())
        while (!all_check)
        {
        
            if (_check_1 && _check_2 && _check_3 && _check_4)
            {
                all_check = true;
            }
            sense();

        }
        ROS_INFO("connected!");
    }

void robot_interface_ROS::sense()
    {
        ros::spinOnce();
        _distance_l_ankle_to_com = listen_distance_l_ankle_to_com();
        _distance_r_ankle_to_com = listen_distance_r_ankle_to_com();
        _distance_l_to_r_foot = listen_distance_l_to_r_foot();
    }
    
void robot_interface_ROS::joints_state_callback(const sensor_msgs::JointState msg_rcv) //this is called by ros
    {
//         TODO: here do all state, also orientation
        _joints_state = msg_rcv.position;
//         if COUNT_ONCE {_callback_counter++;};
        _check_1 = true;
        
    }

void robot_interface_ROS::com_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        Eigen::Affine3d affineMatrix; /*TODO remember to transform everything in affine matrix*/
       
        tf::poseMsgToEigen(msg_rcv.pose, affineMatrix);
        tf::pointMsgToEigen(msg_rcv.pose.position, _com_state);
//         if COUNT_ONCE {_callback_counter++;};
        _check_2 = true;
    }

void robot_interface_ROS::l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        tf::pointMsgToEigen(msg_rcv.pose.position,_l_sole_state);
//         if COUNT_ONCE {_callback_counter++;};
        _check_3 = true;
    }
    
void robot_interface_ROS::r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        tf::pointMsgToEigen(msg_rcv.pose.position, _r_sole_state);
//         if COUNT_ONCE {_callback_counter++;};
        _check_4 = true;
    }

Eigen::Vector3d robot_interface_ROS::listen_distance_l_ankle_to_com()
    {
        tf::Vector3 distance;
        Eigen::Vector3d l_ankle_to_com_distance;
        
        l_ankle_to_com_listener.lookupTransform("ci/l_sole", "ci/com", ros::Time(0), l_ankle_to_com_transform); /*ros::Time(0)*/
        distance = l_ankle_to_com_transform.getOrigin();
        tf::vectorTFToEigen(distance, l_ankle_to_com_distance);
        return l_ankle_to_com_distance;
    }

Eigen::Vector3d robot_interface_ROS::listen_distance_r_ankle_to_com()
    {
        tf::Vector3 distance;
        Eigen::Vector3d r_ankle_to_com_distance;
        
        r_ankle_to_com_listener.lookupTransform("ci/r_sole", "ci/com", ros::Time(0), r_ankle_to_com_transform);
        distance = r_ankle_to_com_transform.getOrigin();
        tf::vectorTFToEigen(distance, r_ankle_to_com_distance);
        return r_ankle_to_com_distance;
    }

Eigen::Vector3d robot_interface_ROS::listen_distance_l_to_r_foot()
    {
        tf::Vector3 distance;
        Eigen::Vector3d l_to_r_foot_distance;
        _l_to_r_foot_listener.lookupTransform("ci/l_sole", "ci/r_sole", ros::Time(0), _l_to_r_foot_transform);
        distance = _l_to_r_foot_transform.getOrigin();
        tf::vectorTFToEigen(distance, l_to_r_foot_distance);
        return l_to_r_foot_distance;
    }

    
    