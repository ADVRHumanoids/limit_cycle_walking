#include <walker/lateral_plane.h>

LateralPlane::LateralPlane()
{
    MpcSolver::Options opt;

    _mpc_solver = std::make_shared<MpcSolver>(opt);

    _q_sns_prev = 0;
    _q_sns = 0;

    _alpha_old = 0;
    _alpha = 0;

    _zmp = 0;

    _switch_side_zmp = 1;

    _size_window = 0;
    _size_step = 0;

    _n_step_future = 0;


}

void LateralPlane::update(double q_sns,
                          double q_max,
                          double q_min,
                          double zmp_val,
                          double duration_preview_window, /* should be constant, equal to size of _K_prev form MPC = duration_preview_window/dt */
                          double duration_step,
                          double dt)
{
    _q_sns_prev = _q_sns;
    _q_sns = q_sns;

    /* compute alpha, phase variable normalized [0, 1] between q_max and q_min */
    _alpha_old = (_q_sns_prev - q_min)/(q_max - q_min);
    _alpha = (_q_sns - q_min)/(q_max - q_min);

    /* if alpha < 0 make it zero. This can happen if q_sns become */
    if (_alpha < 0)
    {
        _alpha = 0;
    }

    /* zmp to track */
    if (_alpha <= 1)
    {
        _zmp = zmp_val;
    }

    /* (double) number of step in the preview window */
    _n_step_future = duration_preview_window / duration_step;

    /* size of the window */
    _size_window = static_cast<int>( duration_preview_window / dt );

    /* size of the step */
    _size_step = static_cast<int>( duration_step / dt );

    /* size of current step (depending on alpha) */
    int size_current = static_cast<int>((1 - _alpha) * _size_step);

    /* current position in preview window (expressed as a time) */
    double time = _alpha * fabs( duration_step );

    /* fill first segment (zmp value in a single step) of zmp_window with current zmp */
    _zmp_window.segment(0, size_current) << zmp_val;
    std::cout << _zmp_window << std::endl;

    int size_remaining = _size_window - size_current;
    /* fill all the other segment (zmp value of all the step after the current) */
    for (int n = 1; n < _n_step_future; n++)
    {
        _zmp_window.segment()
    }

















}

Eigen::VectorXd LateralPlane::computePreviewWindow() {

    /**
      * double dt, double q, double q_max, double q_min,
                          pose,
                          duration_preview_window,
                          duration_step
                          t_impact
      *
      * */
    Eigen::VectorXd preview_window;
    return preview_window;
}

Eigen::Vector3d LateralPlane::updateCom() {

    /* enters current_time, state, start_walk*/
    updatePreviewWindow();

    if (_current_state == State::STARTING)
    {
// //             this is for the beginning, to move the com before the step
        if (_internal_time < _start_walk)
        {
            _zmp_starting.head(_zmp_starting.size()-1) = _zmp_starting.tail(_zmp_starting.size()-1);
            _logger->add("zmp_start", _zmp_starting);
            _zmp_window_y = _zmp_starting;
        }
    }

    _u = _mpc_solver->_K_fb * _com_y + _mpc_solver->_K_prev * _zmp_window_y;
    _mpc_solver->_integrator->integrate(_com_y, _u, dt, _com_y);

    return _com_y;


}
