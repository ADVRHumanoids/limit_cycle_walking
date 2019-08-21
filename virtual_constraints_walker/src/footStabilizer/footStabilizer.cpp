#include<footStabilizer/footStabilizer.h>

footStabilizer::footStabilizer(double dt) :
    _dt(dt),
    _kp(0.001, 0.001, 0.001),
    _kd(0.007, 0.007, 0.007)
{ 
    _logger = XBot::MatLogger::getLogger("/tmp/foot_stabilizer_log");
    _omega.setZero();
    _theta.setZero();
    _cp.setZero();
    _cp_previous.setZero();
    _cp_ref.setZero();
}


Eigen::Vector3d footStabilizer::computeCapturePoint(const Eigen::Vector3d& com_position,
                                                    const Eigen::Vector3d& com_velocity)
{
    Eigen::Vector3d cp(3); cp.setZero();
    
    cp[0] = com_position[0] + com_velocity[0]*sqrt(com_position[2]/grav);
    cp[1] = com_position[1] + com_velocity[1]*sqrt(com_position[2]/grav);
    cp[2] = 0.0;

    return cp;
}

Eigen::Vector3d footStabilizer::computeInstantaneousCapturePoint(const double height_com,
                                                                 const Eigen::Vector3d& com_velocity)                                
{
    Eigen::Vector3d cp(3); cp.setZero();
    
    cp[0] = com_velocity[0]*sqrt(height_com/grav);
    cp[1] = com_velocity[1]*sqrt(height_com/grav);
    cp[2] = 0.0;

    return cp;
}

void footStabilizer::update(const Eigen::Vector3d& com_pos, const Eigen::Vector3d& com_velocity)
{
    _cp_previous = _cp;
    _cp = computeCapturePoint(com_pos, com_velocity);
    
    Eigen::Vector3d cp_dot = (_cp - _cp_previous)/_dt; 

    _theta = controller(_cp, cp_dot, _cp_ref);
}

void footStabilizer::update(const double height_com, const Eigen::Vector3d& com_velocity)
{
    _cp_instantaneous_previous = _cp_instantaneous;
    _cp_instantaneous = computeInstantaneousCapturePoint(height_com, com_velocity);

    Eigen::Vector3d cp_instantaneous_dot = (_cp_instantaneous - _cp_instantaneous_previous)/_dt; 
    
    _theta = controller(_cp_instantaneous, cp_instantaneous_dot, _cp_instantaneous_ref);
}

Eigen::Vector3d footStabilizer::controller(const Eigen::Vector3d cp, const Eigen::Vector3d cp_dot, const Eigen::Vector3d cp_ref)
{
    
    //     std::cout << "Kp: " << _kp.transpose() << std::endl;
    //     std::cout << "Kd: " << _kd.transpose() << std::endl;

    Eigen::Vector3d theta; theta.setZero();
    theta = Eigen::Matrix3d(_kp.asDiagonal()) * (cp - cp_ref) + Eigen::Matrix3d(_kd.asDiagonal()) * cp_dot;
    return theta;
    
    //     std::cout << "cp error: " << (cp - cp_ref).transpose() << std::endl;
}

Eigen::Vector3d footStabilizer::getTheta() const
{
    return _theta;
}
Eigen::Vector3d footStabilizer::getOmega() const
{
    return _omega;
}

Eigen::Vector3d footStabilizer::getCP() const
{
    return _cp;
}

Eigen::Vector3d footStabilizer::getInstantaneousCP() const
{
    return _cp_instantaneous;
}

Eigen::Vector3d footStabilizer::getKd() const
{
    return _kd;
}

Eigen::Vector3d footStabilizer::getKp() const
{
    return _kp;
}

