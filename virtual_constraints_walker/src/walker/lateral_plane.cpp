#include <walker/lateral_plane.h>

LateralPlane::LateralPlane(double dt, Options opt) :
    _q_sns(0),
    _q_sns_prev(0),
    _alpha_old(0),
    _alpha(0),
    _zmp_val(0),
    _switch_side_zmp(1),
    _size_window(0),
    _size_step(0),
    _dt(dt),
    _opt(opt)
{
    _mpc_solver = std::make_shared<MpcSolver>(_opt.h, _opt.Ts, _opt.T, _opt.Q, _opt.R);
}

void LateralPlane::computePreviewWindow(double q_sns,
                                        double q_max,
                                        double q_min,
                                        double zmp_val,
                                        double duration_preview_window, /* should be constant, equal to size of _K_prev form MPC = duration_preview_window/dt */
                                        double duration_step,
                                        double middle_zmp,
                                        double offset)
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
        _zmp_val = zmp_val;
    }

    /* size of the window */
    _size_window = static_cast<int>( duration_preview_window / _dt );

    /* size of the step */
    _size_step = static_cast<int>( duration_step / _dt );

    /* size of current step (depending on alpha) */
    int size_current = static_cast<int>((1 - _alpha) * _size_step);

    /* current position in preview window (expressed as a time) */
    double time = _alpha * fabs( duration_step );

    /* fill first segment (zmp value in a single step) of zmp_window with current zmp */
    _zmp_window.head(size_current) << zmp_val;
    std::cout << "first_segment_zmp: " << _zmp_window << std::endl;

    int size_remaining = _size_window - size_current;

    /* fill all the other segment (zmp value of all the step after the current) */

    while (size_remaining > 0)
    {
        if (_size_step >= size_remaining)
        {
            addSegment(_zmp_window, _size_step, _zmp_val);
            size_remaining = size_remaining - _size_step;
            /* switch _zmp side symmetric to middle_zmp */
            _zmp_val = 2 * middle_zmp - _zmp_val;
            /* add offset if needed */
            _zmp_val = _zmp_val + ( ( (_zmp_val - middle_zmp) > 0) - ( (_zmp_val - middle_zmp) < 0) ) * offset ;
        }
        else
        {
            addSegment(_zmp_window, size_remaining, _zmp_val);
            size_remaining = 0;
            /* switch _zmp side symmetric to middle_zmp */
            _zmp_val = 2 * middle_zmp - _zmp_val;
            /* add offset if needed */
            _zmp_val = _zmp_val + ( ( (_zmp_val - middle_zmp) > 0) - ( (_zmp_val - middle_zmp) < 0) ) * offset ;
        }
    }

    std::cout << "full_zmp: "<< _zmp_window << std::endl;
}

void LateralPlane::addSegment(Eigen::VectorXd& vector, long size, double value)
{
    Eigen::VectorXd segment(size);
    segment = value * segment.setOnes();


    long initial_size = vector.size();
    Eigen::VectorXd new_vector(initial_size + size);
    new_vector.setZero();

    new_vector.head(initial_size) << vector;
    new_vector.tail(size) << segment;
}

void LateralPlane::update(double q_sns,
                          double q_max,
                          double q_min,
                          double zmp_val,
                          double duration_preview_window, /* should be constant, equal to size of _K_prev form MPC = duration_preview_window/dt */
                          double duration_step,
                          double middle_zmp = 0.,
                          double offset = 0.)
{

    /* enters current_time, state, start_walk*/


//    if (_internal_time < _delay_start)
//    {

//        computePreviewWindow(/* .... */);
//        _zmp_starting.head(_zmp_starting.size()-1) = _zmp_starting.tail(_zmp_starting.size()-1);
//        _zmp_window_y = _zmp_starting;
//    }

    computePreviewWindow(q_sns,q_max, q_min,zmp_val, duration_preview_window, duration_step, middle_zmp, offset);
    _u = _mpc_solver->getKfb() * _delta_com + _mpc_solver->getKprev() * _zmp_window;
    _mpc_solver->getIntegrator()->integrate(_delta_com, _u, _dt, _delta_com);

//    return _delta_com;


}
