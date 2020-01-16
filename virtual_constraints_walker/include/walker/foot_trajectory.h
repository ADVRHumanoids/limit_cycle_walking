#ifndef FOOT_TRAJECTORY_H
#define FOOT_TRAJECTORY_H

#include <eigen3/Eigen/Dense>

namespace mdof {

    Eigen::Affine3d computeTrajectory(Eigen::Affine3d T_i,
                                      Eigen::Affine3d T_f,
                                      double clearance,
                                      double start_time, double end_time,
                                      double time
                                      );


//    Eigen::Affine3d compute_swing_trajectory(const Eigen::Vector3d& start,
//                                             const Eigen::Vector3d& end,
//                                             double clearance,
//                                             double t_start,
//                                             double t_end,
//                                             double time
//                                             );




    double computeSwingTrajectoryNormalizedPlane(double dx0, double ddx0,
                                                     double dxf, double ddxf,
                                                     double tau,
                                                     double* __dx, double* __ddx
                                                     );

    double computeSwingTrajectoryNormalizedClearing(double tau);

    double timeWarp(double tau, double beta);

    void FifthOrderPlanning(double x0, double dx0, double ddx0,  /* initial position */
                            double xf, double dxf, double ddxf,  /* final position */
                            double start_time, double end_time,
                            double time, double& x, double& dx, double& ddx
                            );

//    void lSpline(Eigen::VectorXd times, Eigen::VectorXd y, double dt, Eigen::VectorXd &times_vec, Eigen::VectorXd &Y);

};

#endif // FOOT_TRAJECTORY_H
