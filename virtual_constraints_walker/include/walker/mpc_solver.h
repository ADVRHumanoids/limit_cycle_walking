#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include <XBotLogger/Logger.hpp>
#include <OpenMpC/solver/UnconstrainedMpc.h>

#include <walker/lateral_plane.h>

class LateralPlane::MpcSolver {
public:

    struct Options {

        /* height of inverted pendulum (m) */
        double h;

        /*window resolution (sec) */
        double Ts;

        /*window length for MpC (sec) */
        double T;

        /* gains MpC */
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;

        /* integration step */
        double dt;

    };

    MpcSolver(double h,
              double Ts,
              double T,
              double dt,
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
    const double _T;
    const double _dt;
    const Eigen::MatrixXd _Q;
    const Eigen::MatrixXd _R;

};


LateralPlane::MpcSolver::MpcSolver(double h,
                                   double Ts,
                                   double T,
                                   double dt,
                                   Eigen::MatrixXd Q,
                                   Eigen::MatrixXd R) :
    _h(h),
    _Ts(Ts),
    _T(T),
    _dt(dt),
    _Q(Q),
    _R(R)
{
    /* inverted pendulum */
    double w = sqrt(9.8/_h);
    _integrator = OpenMpC::dynamics::LtiDynamics::Integrator(3,1);

    _C_zmp << 1 ,0, -1.0/std::pow(w, 2.0);
    _integrator->addOutput("zmp", _C_zmp);

    int N = round(_T/_dt);
    _lqr = boost::make_shared<OpenMpC::UnconstrainedMpc>(_integrator, _Ts, N); //+1

    _lqr->addInputTask(_R);
    _lqr->addOutputTask("zmp", _Q);

    _lqr->compute();

    _K_fb = _lqr->getStateFeedbackGain();
    _K_prev = _lqr->getOutputFeedforwardGainPreview("zmp");
}

bool LateralPlane::MpcSolver::update()
{
    _lqr->compute();

    _K_fb = _lqr->getStateFeedbackGain();
    _K_prev = _lqr->getOutputFeedforwardGainPreview("zmp");
}



#endif // MPC_SOLVER_H


