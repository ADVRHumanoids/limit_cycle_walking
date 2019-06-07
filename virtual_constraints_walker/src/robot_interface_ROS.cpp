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
    _subs.push_back(n.subscribe("/cartesian/com/state", 10, &robot_interface_ROS::com_state_callback, this)); 
    _subs.push_back(n.subscribe("/cartesian/l_sole/state", 10, &robot_interface_ROS::l_sole_state_callback, this)); 
    _subs.push_back(n.subscribe("/cartesian/r_sole/state", 10, &robot_interface_ROS::r_sole_state_callback, this));
    _subs.push_back(n.subscribe("/cartesian/Waist/state", 10, &robot_interface_ROS::waist_state_callback, this));
    
    //  STABILIZER
//         _subs.push_back(n.subscribe("/cartesian/com_stabilizer/zmp/reference", 10, &robot_interface_ROS::zmp_callback, this));
    
    _subs.push_back(n.subscribe("/xbotcore/cogimon/ft/l_leg_ft", 10, &robot_interface_ROS::l_sole_ft_callback, this));
    _subs.push_back(n.subscribe("/xbotcore/cogimon/ft/r_leg_ft", 10, &robot_interface_ROS::r_sole_ft_callback, this));
    
    _subs.push_back(n.subscribe("/xbotcore/imu/imu_link", 10, &robot_interface_ROS::imu_callback, this));
    
//      prepare listener node
    _world_to_com_listener.waitForTransform("ci/com", "ci/world_odom", ros::Time(0), ros::Duration(3.0));
    l_com_to_ankle_listener.waitForTransform("ci/l_ankle", "ci/com", ros::Time(0), ros::Duration(3.0)); /*ros::Time::now()*/
    r_com_to_ankle_listener.waitForTransform("ci/r_ankle", "ci/com", ros::Time(0), ros::Duration(3.0)); /*ros::Time::now()*/
    _l_to_r_foot_listener.waitForTransform("ci/l_sole", "ci/r_sole", ros::Time(0), ros::Duration(3.0));
    

        bool all_check = false;


    
    ROS_INFO("waiting for robot..");
    _check_1 = _check_2 = _check_3 = _check_4 = _check_5 = _check_6 = false;
    
    _check_5 = true; //TODO take them out
    _check_6 = true;
    
    while (!all_check)
    {
    
        if (_check_1 && _check_2 && _check_3 && _check_4 && _check_5 && _check_6)
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
    _ankle_to_com[Side::Left] = listen_l_ankle_to_com();
    _ankle_to_com[Side::Right] = listen_r_ankle_to_com();
    _world_to_com = listen_world_to_com();
    _l_to_r_foot = listen_l_to_r_foot();
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
    tf::poseMsgToEigen(msg_rcv.pose, _com_state);
//         if COUNT_ONCE {_callback_counter++;};
    _check_2 = true;
}

void robot_interface_ROS::l_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
{
    tf::poseMsgToEigen(msg_rcv.pose, (_sole_state[Side::Left])); /*TODO: is this a good implementation??*/
//         if COUNT_ONCE {_callback_counter++;};
    _check_3 = true;
}
    
void robot_interface_ROS::r_sole_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
{
    tf::poseMsgToEigen(msg_rcv.pose, _sole_state[Side::Right]); /*TODO: is this a good implementation??*/
//         if COUNT_ONCE {_callback_counter++;};
    _check_4 = true;
}

void robot_interface_ROS::l_sole_ft_callback(const geometry_msgs::WrenchStamped msg_rcv) //this is called by ros
{
    tf::wrenchMsgToEigen(msg_rcv.wrench, _sole_ft[Side::Left]); /*TODO: is this a good implementation??*/
//         if COUNT_ONCE {_callback_counter++;};
    _check_5 = true;
}

void robot_interface_ROS::r_sole_ft_callback(const geometry_msgs::WrenchStamped msg_rcv) //this is called by ros
{
    tf::wrenchMsgToEigen(msg_rcv.wrench, _sole_ft[Side::Right]); /*TODO: is this a good implementation??*/
//         if COUNT_ONCE {_callback_counter++;};
    _check_6 = true;
}

void robot_interface_ROS::waist_state_callback(const geometry_msgs::PoseStamped msg_rcv)
{
    tf::poseMsgToEigen(msg_rcv.pose, _waist_state);
//         if COUNT_ONCE {_callback_counter++;};
    _check_7 = true;
}

void robot_interface_ROS::imu_callback(const sensor_msgs::Imu msg_rcv)
{  
    tf::quaternionMsgToEigen(msg_rcv.orientation, _imu_state.orientation);
    tf::vectorMsgToEigen(msg_rcv.angular_velocity, _imu_state.angular_velocity);
    tf::vectorMsgToEigen(msg_rcv.linear_acceleration, _imu_state.linear_acc);
        
//         if COUNT_ONCE {_callback_counter++;};
    _check_8 = true;
}

// void robot_interface_ROS::zmp_callback(const geometry_msgs::PoseStamped msg_rcv)
//     {
//         tf::poseMsgToEigen(msg_rcv.pose, _zmp_state);
//         _check_7 = true;
//     }

Eigen::Affine3d robot_interface_ROS::listen_world_to_com()
{
    tf::Pose distance;
    Eigen::Affine3d world_to_com;
    
//         _world_to_com_listener.lookupTransform("ci/com", "ci/world_odom", ros::Time(0), _world_to_com_transform); /*ros::Time(0)*/
    _world_to_com_listener.lookupTransform("ci/world_odom", "ci/com", ros::Time(0), _world_to_com_transform); /*ros::Time(0)*/
    distance = _world_to_com_transform;
    
    tf::poseTFToEigen(distance,world_to_com);
    return world_to_com;
}

Eigen::Affine3d robot_interface_ROS::listen_l_ankle_to_com()
{
    tf::Pose distance;
    Eigen::Affine3d l_ankle_to_com;
            
    l_com_to_ankle_listener.lookupTransform("ci/com", "ci/l_ankle", ros::Time(0), _l_com_to_ankle_transform); /*ros::Time(0)*/
    distance = _l_com_to_ankle_transform;
    
//         tf::transformTFToEigen(l_com_to_ankle_transform, l_ankle_to_com);
    tf::poseTFToEigen(distance,l_ankle_to_com);
    return l_ankle_to_com;
}

Eigen::Affine3d robot_interface_ROS::listen_r_ankle_to_com()
{
    tf::Pose distance;
    Eigen::Affine3d r_ankle_to_com;
    
    r_com_to_ankle_listener.lookupTransform("ci/com", "ci/r_ankle", ros::Time(0), _r_com_to_ankle_transform);
    distance = _r_com_to_ankle_transform;
    
    tf::poseTFToEigen(distance, r_ankle_to_com);
    return r_ankle_to_com;
}

Eigen::Affine3d robot_interface_ROS::listen_l_to_r_foot()
{
    tf::Pose distance;
    Eigen::Affine3d l_to_r_foot;
    
    _l_to_r_foot_listener.lookupTransform("ci/l_sole", "ci/r_sole", ros::Time(0), _l_to_r_foot_transform);
    distance = _l_to_r_foot_transform;
    
    tf::poseTFToEigen(distance, l_to_r_foot);
    return l_to_r_foot;
}

    
    