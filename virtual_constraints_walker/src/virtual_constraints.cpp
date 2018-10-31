#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <XBotCore/CommandAdvr.h>
#include <tf/transform_listener.h>
#include "cartesian_interface/CartesianInterface.h"
#include <sensor_msgs/JointState.h>
// #include <math.h>

#define PI 3.141592653589793238463

class mainNode 
{
protected:
    mainNode(int argc, char **argv, const char *node_name) 
    {
        ros::init(argc, argv, node_name);
    }
};


class virtualConstraintsNode : mainNode {
public:
    
    virtualConstraintsNode(int argc, char **argv, const char *node_name) : 
    mainNode(argc, argv, node_name)
    {    
        ros::NodeHandle n;

        tf::TransformListener ankle_to_com_listener;
        
        
        cartesian_solution_sub = n.subscribe("/cartesian/solution", 1000, &virtualConstraintsNode::joints_state_callback, this); /*subscribe to cartesian/solution topic*/
        com_sub = n.subscribe("/cartesian/com/state", 1000, &virtualConstraintsNode::com_state_callback, this); /*subscribe to cartesian/solution topic*/
        
        com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 1000); /*publish to /cartesian/com/reference*/
        ankle_to_com_listener.waitForTransform("ci/l_ankle", "ci/com", ros::Time::now(), ros::Duration(3.0));
        ankle_to_com_listener.lookupTransform("ci/l_ankle", "ci/com", ros::Time(0), ankle_to_com_transform);
    }
    

    void joints_state_callback(const sensor_msgs::JointState msg_rcv) //this is called by ros
    {
//         TODO: here do all state, also orientation
        joints_state = msg_rcv.position;
    }

    void com_state_callback(const geometry_msgs::PoseStamped msg_rcv) //this is called by ros
    {
        com_state = msg_rcv.pose.position;
    }
    
    double get_q1()
    {
        double q1_state;
        q1_state = joints_state[10];
    
        ROS_INFO("var is %f", q1_state);
        return q1_state;
    }
    
    
//     double calc_VC_legs ()
//     {
//         double pos;
// //         pos = 2 * leg_length * sin(q1_state);
//         pos = -0.2;
//         return pos;
//     }
    
    
    void publish_x_position_com(double pos) 
    {
        geometry_msgs::PoseStamped msg_cmd;
        msg_cmd.pose.position = com_state;
        msg_cmd.pose.position.x = pos;
//         ROS_INFO("%f", msg_cmd.pose.position.x);
        com_pub.publish(msg_cmd);
    }
    
    double listen_z_distance_ankle_com()
    {
        double z_distance;
        z_distance = ankle_to_com_transform.getOrigin().z();
        return z_distance;
//         ROS_INFO("val is: %f",  transform.getOrigin().z());
    }
    
    void run()
    {
        double q1, z_distance, pos;
        
        z_distance = this->listen_z_distance_ankle_com();
        ROS_INFO("distance_z: %f",  z_distance);
//         q1 = this->get_q1();
        q1 = PI/12;

        pos = z_distance * tan(q1);
        ROS_INFO("distance_x: %f",  pos);
        this->publish_x_position_com(pos);
        
    }
protected:
    
    ros::Subscriber cartesian_solution_sub;
    ros::Subscriber com_sub;
    ros::Publisher com_pub;
    
    std::vector<double> joints_state;
    
    geometry_msgs::Point com_state;
    
    tf::StampedTransform ankle_to_com_transform;
    
    int joint_number = 10; /*ankle_pitch_angle*/
    
};



int main(int argc, char **argv)
{
 
     
    
    
    virtualConstraintsNode VC(argc, argv, "virtual_constraints");
    
    while (ros::ok()) {
        
    ros::spinOnce();
//     VC.get_q1();
//     VC.listen();
    
    
//     VC.publish();
    VC.run();
//     loop_rate.sleep();
    }
    
    
    
//     ros::spinOnce();
//     loop_rate.sleep();
    
  
//     ros::NodeHandle n;
//       
//     tf::TransformListener listener;
//     
//     listener.waitForTransform("ci/com", "ci/l_ankle", ros::Time::now(), ros::Duration(3.0));
//     
//     
//     
//     
//     
//     ros::Publisher CoM_talker = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 1000);
//     
//     
//     ros::Rate loop_rate(10);    
//       
//       
// int count = 0;
// 
//   while (ros::ok())
//   { 
//     double leg_length;
//     geometry_msgs::PoseStamped msg_cmd;
//     tf::StampedTransform transform;
// 
//     listener.lookupTransform("ci/com", "ci/l_ankle", ros::Time(0), transform);
// 
// 
//     leg_length = transform.getOrigin().z(); /*get distance from com to l_ankle*/
//    
//    
//     
//     
//     int pos = 2 * leg_length * sin(q1);
// 
//     //     pos.push_back(angle);
// 
//     msg_cmd.pose.position.x = pos;
// 
// 
//     //     ROS_INFO("%d", msg.pose.position.x);
// 
//     CoM_talker.publish(msg_cmd);
// 
//     
//     ros::spinOnce();
//     loop_rate.sleep();
//     ++count;

}