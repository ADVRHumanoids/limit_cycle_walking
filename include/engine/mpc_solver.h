#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include <XBotLogger/Logger.hpp>
#include <OpenMpC/solver/UnconstrainedMpc.h>

#include <engine/lateral_plane.h>

class LateralPlane::MpcSolver {
public:

    MpcSolver(double h,
              double Ts,
              int size_window,
              Eigen::MatrixXd Q,
              Eigen::MatrixXd R);


    /* set new parameters for MpC solver */
//    bool setOptions(Options new_opt);

    bool update();

    Eigen::MatrixXd getKfb() {return _K_fb;}
    Eigen::MatrixXd getKprev() {return _K_prev;}

    OpenMpC::dynamics::LtiDynamics::Ptr getIntegrator() {return _integrator;}

private:

    Eigen::MatrixXd _K_fb;
    Eigen::MatrixXd _K_prev;
    Eigen::Matrix<double, 1,3> _C_zmp;
    OpenMpC::dynamics::LtiDynamics::Ptr _integrator;
    OpenMpC::UnconstrainedMpc::Ptr _lqr;

    const double _h;
    const double _Ts;
    int _size_window;
    const Eigen::MatrixXd _Q;
    const Eigen::MatrixXd _R;

};

#endif // MPC_SOLVER_H
