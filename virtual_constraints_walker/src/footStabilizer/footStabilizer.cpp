#include<footStabilizer/footStabilizer.h>

footStabilizer::footStabilizer(double dt) :
    _dt(dt),
    _kp(1.0, 1.0, 1.0),
    _kd(0.1, 0.1, 0.1)
{ 
    _logger = XBot::MatLogger::getLogger("/tmp/foot_stabilizer_log");
    _omega.setZero();
    _pos.setZero();
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


void footStabilizer::update(const Eigen::Vector3d& com_pos, const Eigen::Vector3d& com_velocity)
{
    _cp_previous = _cp;
    _cp = computeCapturePoint(com_pos, com_velocity);
    
    Eigen::Vector3d cp_dot = (_cp - _cp_previous)/_dt;
    
    
    _kp << 0.001, 0.001, 0.001; /* 0.01, 0.01, 0.01 */
    _kd << 0.007, 0.007, 0.007; /* 0.01, 0.01, 0.01 */
    
    _pos = Eigen::Matrix3d(_kp.asDiagonal()) * (_cp - _cp_ref) + Eigen::Matrix3d(_kd.asDiagonal()) * cp_dot;
    
    std::cout << "cp error: " << (_cp - _cp_ref).transpose() << std::endl;
}


Eigen::Vector3d footStabilizer::getPos() const
{
    return _pos;
}
Eigen::Vector3d footStabilizer::getOmega() const
{
    return _omega;
}

Eigen::Vector3d footStabilizer::getCP() const
{
    return _cp;
}