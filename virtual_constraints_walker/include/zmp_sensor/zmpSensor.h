#ifndef _XBOT_UTILS_H_
#define _XBOT_UTILS_H_

#include <eigen3/Eigen/Dense>

#include <zmp_sensor/computeZMP.h>

#include <XBotInterface/ModelInterface.h>
#include <robot_interface.h>

class zmpSensor {

    
public:
    
    typedef std::shared_ptr<zmpSensor> Ptr;
    
    zmpSensor(XBot::ForceTorqueSensor::ConstPtr ftSensorL,
            XBot::ForceTorqueSensor::ConstPtr ftSensorR,
            Eigen::Affine3d w_T_Lfoot,
            Eigen::Affine3d w_T_Rfoot,
            Eigen::Affine3d w_T_Lankle,
            Eigen::Affine3d w_T_Rankle,
            const double dt);
    
    
    void update(Eigen::Affine3d w_T_Lfoot, Eigen::Affine3d w_T_Rfoot);
    
    Eigen::Vector3d getZMP_L() {return _ZMP_L;};
    Eigen::Vector3d getZMP_R() {return _ZMP_R;};
    Eigen::Vector3d getZMP() {return _ZMP;};
      
//     void setFootSize(double foot_lenght, double foot_width) {_foot_lenght = foot_lenght;
//                                                              _foot_width = foot_width;};
    
    void setFz_min(double Fz_min) {_Fz_min = Fz_min;};
    
    ~zmpSensor() {_logger->flush();};
    
private:
    
    double _dt; 
    Eigen::Matrix<double, 6, 1> _FT_foot_R, _FT_foot_L; 
    
    Eigen::Affine3d _w_T_Lfoot, _w_T_Rfoot;
    Eigen::Affine3d _w_T_Lankle, _w_T_Rankle;
    
    Eigen::Affine3d _ankle_T_foot_R;
    Eigen::Affine3d _ankle_T_foot_L;
    
    double _foot_lenght, _foot_width;
    double _Fz_min;
    
    Eigen::Vector3d _ZMP_R, _ZMP_L;
    Eigen::Vector3d _ZMP;
    
    XBot::ForceTorqueSensor::ConstPtr _ftSensorL;
    XBot::ForceTorqueSensor::ConstPtr _ftSensorR;
    XBot::MatLogger::Ptr _logger;
    
    
};

#endif