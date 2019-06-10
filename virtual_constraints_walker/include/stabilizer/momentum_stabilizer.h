#ifndef MomentumStabilizer_H
#define MomentumStabilizer_H

#include <Eigen/Dense>
#include <XBotInterface/ModelInterface.h>
#include <robot_interface.h>

namespace vc
{

    class MomentumStabilizer
    {
    public:
        
        typedef std::shared_ptr<MomentumStabilizer> Ptr;
        
    //     MomentumStabilizer(XBot::ImuSensor::ConstPtr imu,
    //                         const double dt);
        
        MomentumStabilizer(robot_interface::imuSensor::Ptr imu, 
                        const double dt);
        
        Eigen::Vector3d getOmega() const;
        
        void setGains(double kp, double kd);
    
        void update();
        
        
    private:
        
        
        Eigen::Vector3d _omega;
        double _omega_max;
        double _kp, _kd;
        double _dt;
        
    //     XBot::ImuSensor::ConstPtr _imu;
        robot_interface::imuSensor::Ptr _imu;
        XBot::MatLogger::Ptr _logger;
    };
}

#endif