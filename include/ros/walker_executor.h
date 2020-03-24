#ifndef WALKER_ROS_H
#define WALKER_ROS_H

#include <ros/ros.h>
#include <walker/walker.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <XBotInterface/RobotInterface.h>

#include <std_srvs/SetBool.h>
#include <limit_cycle_walking/SetCommands.h>

#include <actionlib/client/simple_action_client.h>
#include <cartesian_interface/ReachPoseAction.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotLogger/MatLogger.hpp>

class WalkerExecutor {
public:

    WalkerExecutor();
    void run();
    bool homing();
    
    void log(XBot::MatLogger::Ptr logger);

    ~WalkerExecutor();

private:

    bool updateRobotState();

    void init_services();
    void init_load_model_and_robot();
    void init_load_robot_state();
    void init_load_cartesian_interface();
    void init_load_walker();

    /* this should be done after homing */
    void init_initialize_walker();


    bool run_service(std_srvs::SetBoolRequest& req,
                    std_srvs::SetBoolResponse& res);

    /* TODO maybe doing these as messages ??*/
    bool set_qmax_service(limit_cycle_walking::SetCommandsRequest& req,
               limit_cycle_walking::SetCommandsResponse& res);

    bool set_theta_service(limit_cycle_walking::SetCommandsRequest& req,
                   limit_cycle_walking::SetCommandsResponse& res);

    bool set_cmd_service(limit_cycle_walking::SetCommandsRequest& req,
                   limit_cycle_walking::SetCommandsResponse& res);

    bool homing_service(std_srvs::SetBoolRequest& req,
                std_srvs::SetBoolResponse& res);

    ros::NodeHandle _nh;

    ros::ServiceServer _run_walker_srv;
    ros::ServiceServer _set_q_max_cmd_srv, _set_theta_cmd_srv, _set_cmd_srv, _homing_srv;

    Walker::Ptr _wlkr;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;

    mdof::RobotState _state;
    mdof::RobotState _ref;

    std::string _path_to_cfg;
    std::vector<std::string> _feet_links;

    Eigen::VectorXd _q, _qdot, _qddot;

    XBot::MatLogger::Ptr _logger;

    double _period, _time;

    bool _set_flag, _stop_flag, _start_flag;

    std::array<Eigen::Affine3d, 2> _world_T_ankle;
};

#endif // WALKER_ROS_H




//////// current one ////////////

//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "virtual_constraints");


//    virtualConstraintsNode VC;

//    /*synchronize this node to the cartesian interface*/
//    ros::service::waitForService("cartesian/get_task_list");
//    ros::Rate loop_rate(100); //TODO set it from the robot
//    robot_interface_ROS& robot = VC.get_robot(); /*or -->  VC.get_robot().sense();*/


//    while (ros::ok())
//    {
//        VC.get_robot().sense();
//        VC.exe(ros::Time::now().toSec());
//        loop_rate.sleep();
//    }
//}


