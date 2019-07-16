#ifndef __FOOT_STABILIZER_H__
#define __FOOT_STABILIZER_H__

#include <iostream>
#include <XBotInterface/MatLogger.hpp>

# define grav 9.80665

class footStabilizer
{
public:
    
    footStabilizer(double dt);
    ~footStabilizer() {_logger->flush();};
    
    Eigen::Vector3d getPos() const;
    Eigen::Vector3d getOmega() const;
    Eigen::Vector3d getCP() const;
    
    void setCPRef(Eigen::Vector3d cp_ref) {_cp_ref = cp_ref;};
    void update(const Eigen::Vector3d& com_velocity,
                const Eigen::Vector3d& com_pose);
    
    
    Eigen::Vector3d computeCapturePoint(const Eigen::Vector3d& com_pos, const Eigen::Vector3d& com_velocity);
    
private:
    

    Eigen::Vector3d _omega, _pos;
    
    Eigen::Vector3d _kp, _kd;
    double _dt;
    
    Eigen::Vector3d _cp, _cp_previous, _cp_ref;
    
    XBot::MatLogger::Ptr _logger;
};



#endif