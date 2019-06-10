#include <stabilizer/momentum_stabilizer.h>

vc::MomentumStabilizer::MomentumStabilizer(robot_interface::imuSensor::Ptr imu, const double dt) :
    _imu{imu},
    _dt(dt),
    _omega_max(1.0),
    _kp(1.0),
    _kd(0.1)
{
    _logger = XBot::MatLogger::getLogger("/tmp/momentum_stabilizer_log");
    _omega.setZero();
}

// MomentumStabilizer::MomentumStabilizer(XBot::ImuSensor::ConstPtr imu, const double dt) :
//     _imu(imu),
//     _dt(dt),
//     _omega_max(1.0),
//     _kp(1.0),
//     _kd(0.1)
// {
//     _logger = XBot::MatLogger::getLogger("/tmp/momentum_stabilizer_log");
//     _omega.setZero();
// }

void vc::MomentumStabilizer::setGains(double kp, double kd)
{
    
  _kp = kp;
  _kd = kd;
  
}

void vc::MomentumStabilizer::update()
{
    
    // get world z-axis w.r.t. imu frame
    Eigen::Matrix3d w_R_imu;
    _imu->getOrientation(w_R_imu);
    Eigen::Vector3d imu_z_world = w_R_imu.transpose().col(2);
    
    // waist z-axis is 0 0 1
    Eigen::Vector3d imu_z_waist(0.0, 0.0, 1.0);
    
    
    // get angular vel from imu
    Eigen::Vector3d omega_imu;
    _imu->getAngularVelocity(omega_imu);
    // integrate
    Eigen::Vector3d omega_dot = _kp * imu_z_waist.cross(imu_z_world) + _kd * omega_imu;
    omega_dot.z() = 0.0;
    
    // integrate
    _omega = _omega + omega_dot * _dt;
    
    // anti windup
    _omega = _omega.array().min(_omega_max).max(_omega_max);
    
    
    // log 
    _logger->add("imu_z_world", imu_z_world);
    _logger->add("imu_z_waist", imu_z_waist);
    _logger->add("omega_imu", omega_imu);
    _logger->add("omega_dot", omega_dot);
    _logger->add("omega", _omega);
    
}

Eigen::Vector3d vc::MomentumStabilizer::getOmega() const
{
    return _omega;
}
