#include <ros/ros.h>
#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <zmp_sensor/zmpSensor.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <cartesian_interface/ros/RosImpl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "measureZMP");
    ros::NodeHandle n;
    
    XBot::Cartesian::RosImpl ci;

    
    ros::Rate loop_rate(100); //TODO set it from the robot
    double dt = 0.01;
    ros::Publisher _zmp_pub, _zmp_l_pub, _zmp_r_pub;
    
    _zmp_pub = n.advertise<geometry_msgs::PoseStamped>("measureZMP/zmp_measured", 0);
    _zmp_l_pub = n.advertise<geometry_msgs::PoseStamped>("measureZMP/zmp_l_measured", 0);
    _zmp_r_pub = n.advertise<geometry_msgs::PoseStamped>("measureZMP/zmp_r_measured", 0);
    
    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto robot = XBot::RobotInterface::getRobot(cfg);
    auto model = XBot::ModelInterface::getModel(cfg);
    
    
    zmpSensor::Ptr zmp_sensor;
    XBot::ForceTorqueSensor::ConstPtr FT_sensor_L;
    XBot::ForceTorqueSensor::ConstPtr FT_sensor_R;
    
    try 
    {
        FT_sensor_L = robot->getForceTorque().at("l_leg_ft");
        FT_sensor_R = robot->getForceTorque().at("r_leg_ft");
    }
    catch (std::out_of_range e)
    {
        throw std::runtime_error("Force/Torque sensor does not exist, specify a valid one.");
    }
    
    Eigen::Matrix<double, 6, 1> FT_foot_L;
    Eigen::Matrix<double, 6, 1> FT_foot_R;
    
    Eigen::Affine3d w_T_sole_L, w_T_sole_R;
    model->getPose("l_sole", w_T_sole_L);
    model->getPose("r_sole", w_T_sole_R);
    
    
    Eigen::Affine3d w_T_ankle_L, w_T_ankle_R;
    model->getPose("l_ankle", w_T_ankle_L);
    model->getPose("r_ankle", w_T_ankle_R);
    
    zmp_sensor = std::make_shared<zmpSensor>
                    (
                        FT_sensor_L,
                        FT_sensor_R,
                        w_T_sole_L,
                        w_T_sole_R, 
                        w_T_ankle_L, 
                        w_T_ankle_R,
                        dt
                    );
    
    while (ros::ok())
    {
        robot->sense();
//         model->syncFrom(*robot, XBot::Sync::Position, XBot::Sync::MotorSide);
        
        model->syncFrom(*robot);
        
//         model->getPose("l_sole", w_T_sole_L);
//         model->getPose("r_sole", w_T_sole_R);
        
        ci.getPoseReference("l_sole", w_T_sole_L);
        ci.getPoseReference("r_sole", w_T_sole_R);
        
        zmp_sensor->update(w_T_sole_L, w_T_sole_R);
        
        
        geometry_msgs::PoseStamped zmp_l_measured;
        tf::pointEigenToMsg(zmp_sensor->getZMP_L(), zmp_l_measured.pose.position);
        _zmp_l_pub.publish(zmp_l_measured);
        
        geometry_msgs::PoseStamped zmp_r_measured;
        tf::pointEigenToMsg(zmp_sensor->getZMP_R(), zmp_r_measured.pose.position);
        _zmp_r_pub.publish(zmp_r_measured);
        
        geometry_msgs::PoseStamped zmp_measured;
        tf::pointEigenToMsg(zmp_sensor->getZMP(), zmp_measured.pose.position);
        _zmp_pub.publish(zmp_measured);
        
        
        loop_rate.sleep();
    }
}   