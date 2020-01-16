#ifndef SAGITTAL_PLANE_H
#define SAGITTAL_PLANE_H

#include <XBotLogger/Logger.hpp>

class SagittalPlane {
public:

    typedef std::shared_ptr< SagittalPlane > Ptr;

    SagittalPlane(double dt);

    void update(double q, double height_com);

    double getDeltaCom(){return _delta_com;}
    
    Eigen::Affine3d getFootGoal() const;
    Eigen::Vector3d getComGoal() const;

private:
    
    
    double computeCom(double q,
                     double height);

    bool computeStep(double q_min,
                     double q_max,
                     double theta,
                     double height_com,
                     Eigen::Vector3d inital_com,
                     Eigen::Affine3d initial_stance_foot,
                     Eigen::Vector3d& final_com,
                     Eigen::Affine3d& final_swing_foot);

    double _delta_com;
    
//    Eigen::Affine3d _delta_foot;
    
//    Eigen::Affine3d _foot_start;
    Eigen::Affine3d _foot_goal;
    
//    Eigen::Vector3d _com_start;
    Eigen::Vector3d _com_goal;

    double _q;





};

#endif // SAGITTAL_PLANE_H
