#include <stabilizer/momentum_stabilizer.h>

vc::MomentumStabilizer::MomentumStabilizer(robot_interface::imuSensor::Ptr imu, const double dt) :
    _imu(imu),
    _dt(dt),
    _omega_roll_max(5.0),
    _omega_pitch_max(5.0),
    _imu_z_waist_ref(0.0, 0.0, 1.0),
    _kp(1.0, 1.0, 1.0),
    _kd(0.1, 0.1, 0.1)
{
    _logger = XBot::MatLogger::getLogger("/tmp/momentum_stabilizer_log");
    _omega.setZero();
    
//     Eigen::Matrix3d w_R_imu;
//     _imu->getOrientation(w_R_imu);
//     _imu_z_waist_ref = w_R_imu.transpose().col(2);
    
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

void vc::MomentumStabilizer::setGains(Eigen::Vector3d kp, Eigen::Vector3d kd)
{
    
    _kp = kp;
    _kd = kd;
  
}

void vc::MomentumStabilizer::setWaistReference(Eigen::Vector3d z_waist_ref)
{
    // reference waist z-axis
    _imu_z_waist_ref =  z_waist_ref;
}

void vc::MomentumStabilizer::update()
{
    
    // get world z-axis w.r.t. imu frame
    Eigen::Matrix3d w_R_imu;

    _imu->getOrientation(w_R_imu);
    Eigen::Vector3d imu_z_world = w_R_imu.transpose().col(2);
    
//     std::cout << "imu_z_world: " << imu_z_world.transpose() << std::endl;
    // waist z-axis is 0 0 1
//     Eigen::Vector3d imu_z_waist(0.0, 0.0, 1.0);
    
    // orientation of the imu w.r.t. to z world
//     Eigen::Vector3d imu_orient = imu_z_waist.cross(imu_z_world);
    
    Eigen::Vector3d imu_orient = _imu_z_waist_ref.cross(imu_z_world);
    
//     std::cout << "_imu_z_waist_ref: " << _imu_z_waist_ref.transpose() << std::endl;
//     std::cout << "imu_orient: " << imu_orient.transpose() << std::endl;
    
    // get angular vel from imu
    Eigen::Vector3d omega_imu;
    _imu->getAngularVelocity(omega_imu);
    
    
    
    // controller
    Eigen::Vector3d omega_dot = - Eigen::Matrix3d(_kp.asDiagonal()) * imu_orient - Eigen::Matrix3d(_kd.asDiagonal()) * omega_imu;
    
    omega_dot.z() = 0.0;
    
//     std::cout << "omega_dot: " << omega_dot.transpose() << std::endl;
    // integrate
    _omega = _omega + omega_dot * _dt;
    
    // anti windup
//     _omega = _omega.array().min(_omega_max).max( - _omega_max);
    
    if (_omega(0) <= -_omega_roll_max)
        _omega(0)  = - _omega_roll_max;
    else if (_omega(0) >= _omega_roll_max)
        _omega(0)  = _omega_roll_max;

    if (_omega(1) <= -_omega_pitch_max)
        _omega(1)  = - _omega_pitch_max;
    else if (_omega(1) >= _omega_pitch_max)
        _omega(1)  = _omega_pitch_max;
    
    
    // log 
    _logger->add("imu_z_world", imu_z_world);
    _logger->add("imu_z_waist", _imu_z_waist_ref);
    _logger->add("imu_orient", imu_orient);
    _logger->add("omega_imu", omega_imu);
    _logger->add("omega_dot", omega_dot);
    _logger->add("omega", _omega);
    
}

Eigen::Vector3d vc::MomentumStabilizer::getOmega() const
{
    return _omega;
}
