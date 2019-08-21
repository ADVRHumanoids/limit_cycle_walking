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
    
    Eigen::Vector3d getTheta() const;
    Eigen::Vector3d getOmega() const;
    
    Eigen::Vector3d getCP() const;
    Eigen::Vector3d getInstantaneousCP() const;
    
    Eigen::Vector3d getKp() const;
    Eigen::Vector3d getKd() const;
    
    void setKp(Eigen::Vector3d kp) {_kp = kp;  /* std::cout << "K_p gain set to: " << _kp.transpose() << std::endl; */  };
    void setKd(Eigen::Vector3d kd) {_kd = kd; /* std::cout << "K_d gain set to: " << _kd.transpose() << std::endl; */ };
    void setCPRef(Eigen::Vector3d cp_ref) {_cp_ref = cp_ref; /* std::cout << "CP reference set to: " << _cp_ref.transpose() << std::endl; */};
    void setInstantaneousCPRef(Eigen::Vector3d cp_instantaneous_ref) {_cp_instantaneous_ref = cp_instantaneous_ref;};
    
    Eigen::Vector3d controller(const Eigen::Vector3d cp, const Eigen::Vector3d cp_dot, const Eigen::Vector3d cp_ref);
    
    /* update stabilizer using CP */
    void update(const Eigen::Vector3d& com_pose,
                const Eigen::Vector3d& com_velocity);
    
    /* update stabilizer using instantaneous CP */
    void update(const double height_com, 
                const Eigen::Vector3d& com_velocity);
    
    
    
private:
    
    Eigen::Vector3d computeCapturePoint(const Eigen::Vector3d& com_pos, const Eigen::Vector3d& com_velocity);
    Eigen::Vector3d computeInstantaneousCapturePoint(double height, const Eigen::Vector3d& com_velocity);
    
    Eigen::Vector3d _omega, _theta;
    
    Eigen::Vector3d _kp, _kd;
    double _dt;
    
    Eigen::Vector3d _cp, _cp_previous, _cp_ref;
    Eigen::Vector3d _cp_instantaneous, _cp_instantaneous_previous, _cp_instantaneous_ref;
    
    XBot::MatLogger::Ptr _logger;
};



#endif