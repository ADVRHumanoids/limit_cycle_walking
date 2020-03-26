#include "walker/foot_trajectory.h"

namespace Eigen
{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}



Eigen::Affine3d mdof::computeTrajectory(Eigen::Affine3d T_i, Eigen::Affine3d T_f,
                                                double clearance,
                                                double start_time, double end_time,
                                                double time
                                                )
    {


    //position

    if (start_time - end_time == 0)
    {
        end_time = start_time + 1e-4;
    }

    Eigen::Vector3d ret;

    double dx0 = 0;
    double ddx0 = 0;

    double dxf = 0;
    double ddxf = 0;

    double dx;
    double ddx;

    double beta = 1; //2
    double tau = std::min(std::max((time - start_time)/(end_time - start_time), 0.0), 1.0);
    double alpha = computeSwingTrajectoryNormalizedPlane(dx0, ddx0, dxf, ddxf, timeWarp(tau, beta), &dx, &ddx);

    ret.head<2>() = ( 1 - alpha ) * T_i.translation().head<2>() + alpha * T_f.translation().head<2>();
    ret.z() = T_i.translation().z() + computeSwingTrajectoryNormalizedClearing( timeWarp(tau, beta) ) * clearance;

    /* rotation */

    Eigen::Quaterniond q_start(T_i.linear());
    Eigen::Quaterniond q_end(T_f.linear());

    double tau2, dtau2, ddtau2;
    FifthOrderPlanning( 0, 0, 0, 1, 0, 0, start_time, end_time, time, tau2, dtau2, ddtau2);
    Eigen::Affine3d interpolated;
    interpolated.setIdentity();
    interpolated.linear() = q_start.slerp(tau, q_end).toRotationMatrix();
    interpolated.translation() = ret;
//     interpolated.translation() = (1 - tau)*start.translation() + tau*end.translation();

    return interpolated;

    }

//Eigen::Affine3d foot_trajectory::compute_swing_trajectory(const Eigen::Vector3d& start,
//                                                                 const Eigen::Vector3d& end,
//                                                                 double clearance,
//                                                                 double t_start,
//                                                                 double t_end,
//                                                                 double time)
//{
//    Eigen::Vector3d ret;

//    double dx0 = 0;
//    double ddx0 = 0;

//    double dxf = 0;
//    double ddxf = 0;

//    double dx;
//    double ddx;

//    double beta = 1; //2
//    double tau = std::min(std::max((time - t_start)/(t_end - t_start), 0.0), 1.0);
//    double alpha = compute_swing_trajectory_normalized_plane(dx0, ddx0, dxf, ddxf, time_warp(tau, beta), &dx, &ddx);

//    ret.head<2>() = (1-alpha)*start.head<2>() + alpha*end.head<2>();
//    ret.z() = start.z() + compute_swing_trajectory_normalized_clearing(end.z()/clearance, time_warp(tau, beta))*clearance;

//    return ret;
//}

double mdof::computeSwingTrajectoryNormalizedPlane(double dx0, double ddx0,
                                                              double dxf, double ddxf,
                                                              double tau,
                                                              double* __dx, double* __ddx)
{

    double x, dx, ddx;
    FifthOrderPlanning(0, dx0, ddx0, 1, dxf, ddxf, 0, 1, tau, x, dx, ddx);

    if(__dx) *__dx = dx;
    if(__ddx) *__ddx = ddx;

    return x;
}

double mdof::computeSwingTrajectoryNormalizedClearing(double tau)
{
    double x = std::pow(tau, 3)*std::pow(1-tau, 3);
    double x_max = 1./64.;
    x = x/x_max;

    return x;
}

double mdof::timeWarp(double tau, double beta)
{
    return 1.0 - std::pow(1.0 - tau, beta);
}

void mdof::FifthOrderPlanning(double x0, double dx0, double ddx0,  //initial position
                              double xf, double dxf, double ddxf,  //final position
                              double start_time, double end_time,
                              double time, double& x, double& dx, double& ddx
                              )
{
    Eigen::Matrix6d A;
    A << 1.0000,         0,         0,         0,         0,         0,
              0,    1.0000,         0,         0,         0,         0,
              0,         0,    0.5000,         0,         0,         0,
       -10.0000,   -6.0000,   -1.5000,   10.0000,   -4.0000,    0.5000,
        15.0000,    8.0000,    1.5000,  -15.0000,    7.0000,   -1.0000,
        -6.0000,   -3.0000,   -0.5000,    6.0000,   -3.0000,    0.5000;

    double alpha = (end_time-start_time);
    alpha = std::max(1e-6, alpha);
    double tau = (time - start_time)/alpha; // d/dt = d/d(alpha*tau)
    tau = std::max(0.0, tau);
    tau = std::min(tau, 1.0);

    Eigen::Vector6d b;
    b << x0, dx0*alpha, ddx0*std::pow(alpha,2.0), xf, dxf*alpha, ddxf*std::pow(alpha,2.0);

    Eigen::Vector6d coeffs = A*b;

    Eigen::Vector6d t_v, dt_v, ddt_v;

    for(int i = 0; i < 6; i++)
    {
        t_v(i) = std::pow(tau, i);
        dt_v(i) = i > 0 ? std::pow(tau, i-1)*i : 0;
        ddt_v(i) = i > 1 ? std::pow(tau, i-2)*i*(i-1) : 0;

    }

    x = coeffs.dot(t_v);
    dx = coeffs.dot(dt_v)/alpha;
    ddx = coeffs.dot(ddt_v)/(alpha*alpha);
}


//void foot_trajectory::lSpline(Eigen::VectorXd times, Eigen::VectorXd y, double dt, Eigen::VectorXd &times_vec, Eigen::VectorXd &Y)
//{

//    int n = times.size();
//    double N = 0;


//    Eigen::VectorXi N_chunks(n-1);

//    for(int i=0; i<n-1; i++)
//    {
//        N = N + (times.coeff(i+1)-times.coeff(i))/dt;
//        N_chunks(i) = round((times.coeff(i+1)-times.coeff(i))/dt);   //round((times.coeff(i+1)-times.coeff(i))/dt);
//    }

////     std::cout << "N: " << N << std::endl;
////     std::cout << "N_progress: " << N_chunks.transpose() << std::endl;

//    Y.resize(N+1,1);
//    Y.setZero();

//    times_vec.setLinSpaced(N, dt, dt*N);

////     std::cout << "times_vec_init" << times_vec(0) << std::endl;
////     std::cout << "times_vec: " << times_vec.transpose() << std::endl;

//    int idx = 0;
//    for(int i=0; i<n-1; i++)
//    {
//        Eigen::VectorXd temp_vec(N_chunks(i)+1);
//        temp_vec.setLinSpaced(N_chunks(i)+1, y.coeff(i), y.coeff(i+1));

////         std::cout << "temp_vec.size(): " << temp_vec.size()-1 << std::endl;

//        Y.segment(idx, temp_vec.size()-1) = temp_vec.segment(1,temp_vec.size()-1);
//        idx += (temp_vec.size()-1);

////         std::cout << "idx: " << idx << std::endl;
//    }

////     std::cout << "Y: " << Y.transpose() << std::endl;
//    for (int i = 0; i < times_vec.size(); i++)
//    {
//        _logger->add("comy_traj_planned", times_vec(i));
//        _logger->add("times_traj_planned", Y(i));
//    }
//}
