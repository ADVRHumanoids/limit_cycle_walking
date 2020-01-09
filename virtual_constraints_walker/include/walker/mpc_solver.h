#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include <XBotLogger/Logger.hpp>
#include <OpenMpC/solver/UnconstrainedMpc.h>

#include <walker/lateral_plane.h>

class LateralPlane::MpcSolver {
public:

    struct Options {

        Options() :
            h(1.0),
            Ts(0.01),
            T(1),
            dt(0.01),
            Q(1,1),
            R(1,1)
        {
            Q << 1000000;
            R << 1;
        };
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

    MpcSolver(Options opt = Options());

    /* set new parameters for MpC solver */
    bool setOptions(Options new_opt);

    bool update();

    Eigen::MatrixXd getKfb() {return _K_fb;}
    Eigen::MatrixXd getKprev() {return _K_prev;}

private:

    Eigen::MatrixXd _K_fb;
    Eigen::MatrixXd _K_prev;
    Options _opt;
    Eigen::Matrix<double, 1,3> _C_zmp;
    OpenMpC::dynamics::LtiDynamics::Ptr _integrator;
    OpenMpC::UnconstrainedMpc::Ptr _lqr;

};


LateralPlane::MpcSolver::MpcSolver(Options opt) :
    _opt(opt)
{
    /* inverted pendulum */
    double w = sqrt(9.8/_opt.h);
    _integrator = OpenMpC::dynamics::LtiDynamics::Integrator(3,1);

    _C_zmp << 1 ,0, -1.0/std::pow(w, 2.0);
    _integrator->addOutput("zmp", _C_zmp);

    int N = round(_opt.T/_opt.dt);
    _lqr = boost::make_shared<OpenMpC::UnconstrainedMpc>(_integrator, _opt.Ts, N); //+1

    _lqr->addInputTask(_opt.R);
    _lqr->addOutputTask("zmp", _opt.Q);

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


