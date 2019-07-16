#include <ros/ros.h>
#include <footStabilizer/footStabilizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <visualization_msgs/Marker.h>
#include <cartesian_interface/ros/RosImpl.h>

class item_fb {
        
    public:
        
        item_fb() 
        {
            _pos.matrix().setZero(); 
            _vel.setZero(); 
            _check_1 = 0;
            _check_2 = 0;
        }

        Eigen::Affine3d getPose() {return _pos;};
        Eigen::Matrix<double, 6, 1> getVelocity() {return _vel;};
        
        void pos_callback(const geometry_msgs::PoseStamped msg_rcv)
        {
            tf::poseMsgToEigen(msg_rcv.pose, _pos);
            _check_1 = 1;
        }
        
        void vel_callback(const geometry_msgs::TwistStamped msg_rcv)
        {
            tf::twistMsgToEigen(msg_rcv.twist, _vel);
            _check_2 = 1;
        }
        
        bool isReady() {return (_check_1 && _check_2);};
    private:
        Eigen::Affine3d _pos;
        Eigen::Matrix<double, 6, 1> _vel;
        bool _check_1, _check_2;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "compute_cp");
    ros::NodeHandle nh("xbotcore");
    
//     XBot::Cartesian::RosImpl ci;
    
    auto cfg = XBot::ConfigOptionsFromParamServer(nh);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    auto model = XBot::ModelInterface::getModel(cfg);
    
    ros::Rate loop_rate(100); //TODO set it from the robot
    double dt = 0.01;
   
    item_fb fb;
   
    /* build stabilizer*/
    footStabilizer stab(dt);
    
    

    XBot::JointNameMap mapCogimon;
    
    ros::Publisher _cp_pub, _com_pub, _vis_pub; 
    ros::Subscriber _com_sub, _fb_pos_sub, _fb_vel_sub;
    
    _cp_pub = nh.advertise<geometry_msgs::PointStamped>("/compute_cp/cp_measured", 10);
    _com_pub = nh.advertise<nav_msgs::Odometry>("/compute_cp/com_measured", 10);
    _vis_pub = nh.advertise<visualization_msgs::Marker>( "cp", 0 );
    
//     ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/xbotcore/floating_base_position");
    
    _fb_pos_sub = nh.subscribe("/xbotcore/floating_base_position", 10, &item_fb::pos_callback, &fb);
    _fb_vel_sub = nh.subscribe("/xbotcore/floating_base_velocity", 10, &item_fb::vel_callback, &fb);
    
    std::cout << "INITIALIZING ..." << std::endl;
    
    while (!fb.isReady())
    {
        ros::spinOnce();
        fb.getPose();
        fb.getVelocity();
    }
    
    
    
    Eigen::Vector3d com_pos, com_vel;
    Eigen::Vector3d cp;
    
    
    com_pos.setZero();
    com_vel.setZero();
    cp.setZero();
    
    /* first run for INITIALIZATION (so that cp and previous cp are the same)*/
    robot->sense();
    model->syncFrom(*robot);
    
    model->setFloatingBaseState(fb.getPose(), fb.getVelocity());
    model->update();
        
    model->getCOM(com_pos);
    model->getCOMVelocity(com_vel);
    
    model->getJointPosition(mapCogimon);
    
    stab.update(com_pos, com_vel);
    Eigen::Vector3d initial_cp = stab.getCP();
    
    stab.setCPRef(initial_cp);
    stab.update(com_pos, com_vel);
    
    std::cout << "DONE!" << std::endl;
    /* -------------------------------------------------------------------- */
        
    while (ros::ok())
    {
        ros::spinOnce();
        
        robot->sense();
        model->syncFrom(*robot);
        
        model->setFloatingBaseState(fb.getPose(), fb.getVelocity());
        model->update();
        
        model->getCOM(com_pos);
        model->getCOMVelocity(com_vel);
//         model->getJointPosition(mapCogimon);
        
        stab.update(com_pos, com_vel);
        cp = stab.getCP();
        Eigen::Vector3d pos_ankle = stab.getPos();
        
        mapCogimon["LAnklePitch"] += pos_ankle[0];
        mapCogimon["RAnklePitch"] += pos_ankle[0];
        robot->setPositionReference(mapCogimon);        
        robot->move();
        
        
        
        
        /* ------------------- visualization and topics ------------------------ */ 
//         std::cout << "fb_pos: " << fb.getPose().matrix() << std::endl; 
//         std::cout << "fb_vel: " << fb.getVelocity().transpose() << std::endl;
        
        std::cout << "pos ankle: " << pos_ankle.transpose() << std::endl;
//         std::cout << "com position: " << com_pos.transpose() << std::endl;
//         std::cout << "com velocity: " << com_vel.transpose() << std::endl;
//         std::cout << "cp: " << cp.transpose() << std::endl;
        std::cout << "------------------------" << std::endl;
        
        nav_msgs::Odometry cmd_com;
        cmd_com.pose.pose.position.x = com_pos[0];
        cmd_com.pose.pose.position.y = com_pos[1];
        cmd_com.pose.pose.position.z = com_pos[2];
//         
        cmd_com.twist.twist.linear.x = com_vel[0];
        cmd_com.twist.twist.linear.y = com_vel[1];
        cmd_com.twist.twist.linear.z = com_vel[2];
        
        geometry_msgs::PointStamped cmd_cp;
        tf::pointEigenToMsg(cp, cmd_cp.point);
        
        _com_pub.publish(cmd_com);
        _cp_pub.publish(cmd_cp);
        
        
        visualization_msgs::Marker CP_marker;
        std::string child_frame_id = "world_gazebo";

        Eigen::Vector3d cp_vis(cp);
        
//         cp_vis[2] = 
        CP_marker.header.frame_id = child_frame_id;
        CP_marker.header.stamp = ros::Time::now();
        CP_marker.ns = "CP";
        CP_marker.id = 0;
        CP_marker.type = visualization_msgs::Marker::SPHERE;
        CP_marker.action = visualization_msgs::Marker::ADD;

        CP_marker.pose.orientation.x = 0.0;
        CP_marker.pose.orientation.y = 0.0;
        CP_marker.pose.orientation.z = 0.0;
        CP_marker.pose.orientation.w = 1.0;
        CP_marker.pose.position.x = cp[0];
        CP_marker.pose.position.y = cp[1];
        CP_marker.pose.position.z = cp[2];

        CP_marker.color.a = 1.0;
        CP_marker.color.r = 0.0;
        CP_marker.color.g = 1.0;
        CP_marker.color.b = 1.0;

        CP_marker.scale.x = 0.02;
        CP_marker.scale.y = 0.02;
        CP_marker.scale.z = 0.02;

        _vis_pub.publish(CP_marker);
        
        loop_rate.sleep();
    }
    
}