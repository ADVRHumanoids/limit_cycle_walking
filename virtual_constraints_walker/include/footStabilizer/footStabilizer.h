#ifndef __FOOT_STABILIZER_H__
#define __FOOT_STABILIZER_H__

#include <iostream>
#include <XBotInterface/MatLogger.hpp>

class footStabilizer
{
public:
    footStabilizer();
    ~footStabilizer() {_logger->flush();};
    
    void update(const Eigen::VectorXd& com_velocity,
                const Eigen::VectorXd& com_pose);
    
    
    
private:
    
    
    
    XBot::MatLogger::Ptr _logger;
};



Eigen::VectorXd computeCapturePoint(const Eigen::VectorXd& com_velocity,
                                                     const Eigen::VectorXd& com_pose)
{
    Eigen::VectorXd cp(3); cp.setZero(3);

    Eigen::VectorXd com_velocity_ = com_velocity;

    double g = 9.81;

    cp[0] = com_pose[0] + com_velocity_[0]*sqrt(com_pose[2]/g);
    cp[1] = com_pose[1] + com_velocity_[1]*sqrt(com_pose[2]/g);
    cp[2] = 0.0;

    return cp;
}