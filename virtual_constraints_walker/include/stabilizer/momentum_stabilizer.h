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
        
        ~MomentumStabilizer() {_logger->flush();};
        
        Eigen::Vector3d getOmega() const;
        
        void setGains(Eigen::Vector3d kp, Eigen::Vector3d kd);
        void setWaistReference(Eigen::Vector3d z_waist_ref);
        void update();
        
        
    private:
        
        
        Eigen::Vector3d _omega;
        double _omega_roll_max;
        double _omega_pitch_max;
        Eigen::Vector3d _kp, _kd;
        double _dt;
        
        Eigen::Vector3d _imu_z_waist_ref;
    //     XBot::ImuSensor::ConstPtr _imu;
        
        robot_interface::imuSensor::Ptr _imu;
        XBot::MatLogger::Ptr _logger;
    };
}

#endif