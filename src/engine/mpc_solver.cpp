#include <engine/mpc_solver.h>

LateralPlane::MpcSolver::MpcSolver(double h,
                                   double Ts,
                                   int size_window,
                                   Eigen::MatrixXd Q,
                                   Eigen::MatrixXd R) :
    _h(h),
    _Ts(Ts),
    _size_window(size_window),
    _Q(Q),
    _R(R)
{
    std::cout << "constructing mpc solver... " << std::endl;

    /* inverted pendulum */
    double w = sqrt(9.8/_h);
    _integrator = OpenMpC::dynamics::LtiDynamics::Integrator(3,1);

    _C_zmp << 1 ,0, -1.0/std::pow(w, 2.0);
    _integrator->addOutput("zmp", _C_zmp);

    _lqr = boost::make_shared<OpenMpC::UnconstrainedMpc>(_integrator, _Ts, size_window);

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

    return true;
}






