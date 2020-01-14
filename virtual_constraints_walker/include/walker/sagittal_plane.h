#ifndef SAGITTAL_PLANE_H
#define SAGITTAL_PLANE_H

#include <XBotLogger/Logger.hpp>

class SagittalPlane {
public:

    typedef std::shared_ptr< SagittalPlane > Ptr;

    SagittalPlane(double dt);

    void update(double q, double height_com);

    double getDeltaCom(){return _delta_com;}
    
    Eigen::Affine3d getFootStart() const;
    Eigen::Affine3d getFootEnd() const;
    
    Eigen::Vector3d getComStart() const;
    Eigen::Vector3d getComEnd() const;

private:
    
    
    double computeCom(double q,
                     double height);

    bool computeStep(double q_min,
                     double q_max,
                     double theta,
                     double step_duration,
                     double height_com,
                     double& steep_coeff,
                     Eigen::Vector3d inital_com,
                     Eigen::Vector3d& final_com,
                     Eigen::Affine3d initial_swing_foot,
                     Eigen::Affine3d initial_stance_foot,
                     Eigen::Affine3d& final_swing_foot,
                     Eigen::Affine3d& final_stance_foot,
                     Eigen::Affine3d& final_waist);

    double _delta_com;
    
//    Eigen::Affine3d _delta_foot;
    
    Eigen::Affine3d _foot_start;
    Eigen::Affine3d _foot_end;
    
    Eigen::Vector3d _com_start;
    Eigen::Vector3d _com_end;

    double _q;





};

#endif // SAGITTAL_PLANE_H
