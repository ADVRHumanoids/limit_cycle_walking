#include <engine/lateral_plane.h>

LateralPlane::LateralPlane(double dt, Options opt) :
    _q_sns(0),
    _q_sns_prev(0),
    _alpha_old(0),
    _alpha(0),
    _zmp_val(0),
    _switch_side_zmp(1),
    _size_step(0),
    _delay_start(0),
    _dt(dt),
    _opt(opt)

{
    _u.setZero();
    /* size window is fixed now: horizon_lenght (t) / dt) */
    _size_window = round(_opt.horizon_duration/_dt);
    _delta_com.setZero();
    _mpc_solver = std::make_shared<MpcSolver>(_opt.h, _opt.Ts, _size_window, _opt.Q, _opt.R);
    _zmp_window.resize(_size_window);
}

void LateralPlane::computePreviewWindow(double q_sns,
                                        double q_min,
                                        double q_max,
                                        double zmp_val_current,
                                        double zmp_val_next,
                                        double duration_step,
                                        double middle_zmp,
                                        double offset)
{
    _q_sns_prev = q_sns;

    /* compute alpha, phase variable normalized [0, 1] between q_max and q_min */
    _alpha_old = (_q_sns_prev - q_min)/(q_max - q_min);
    _alpha = (q_sns - q_min)/(q_max - q_min);

    /* if alpha < 0 make it zero. This can happen if q_sns become */
    if (_alpha < 0)
    {
        _alpha = 0;
    }

    /* zmp to track */
    if (_alpha <= 1)
    {
        _zmp_val = zmp_val_current;
    }

    /* size of the step */
    _size_step = static_cast<int>( duration_step / _dt );

    /* size of current step (depending on alpha) */
    int size_current_step = static_cast<int>((1 - _alpha) * _size_step);

    /* current position in preview window (expressed as a time) */
//    double time = _alpha * fabs( duration_step );

    /* fill first segment (zmp value in a single step) of zmp_window with current zmp */
    /* if size_current_step is bigger than full size window, take size window */
    Eigen::VectorXd first_segment(size_current_step > _size_window ? _size_window : size_current_step);
    _zmp_window.head(first_segment.size()) << _zmp_val * first_segment.setOnes();
//    std::cout << "first_segment_zmp: " << _zmp_window.transpose() << std::endl;

    int size_remaining = _size_window - first_segment.size();

    /* fill all the other segment (zmp value of all the step after the current) with next zmp (and then switch symmetrically around the middle)*/
    _zmp_val = zmp_val_next;

    while (size_remaining > 0)
    {
        if (_size_step <= size_remaining)
        {
            Eigen::VectorXd segment(_size_step);
            _zmp_window.segment(_size_window - size_remaining, _size_step) << _zmp_val * segment.setOnes();
            size_remaining = size_remaining - _size_step;
            /* switch _zmp side symmetric to middle_zmp */
            _zmp_val = 2 * middle_zmp - _zmp_val;
            /* add offset if needed */
            _zmp_val = _zmp_val + ( ( (_zmp_val - middle_zmp) > 0) - ( (_zmp_val - middle_zmp) < 0) ) * offset ;

        }
        else
        {
            Eigen::VectorXd segment(size_remaining);
            _zmp_window.tail(size_remaining) << _zmp_val * segment.setOnes();
            size_remaining = 0;

        }
    }

//    std::cout << "zmp_window: " << _zmp_window.transpose() << std::endl;
}

void LateralPlane::update(double q_sns,
                          double q_max,
                          double q_min,
                          double zmp_val_current,
                          double zmp_val_next,
                          double duration_step,
                          double middle_zmp,
                          double offset)
{
    computePreviewWindow(q_sns, q_max, q_min, zmp_val_current, zmp_val_next, duration_step, middle_zmp, offset);

    /* com value is actually a delta_com */
    _u = _mpc_solver->getKfb() * _delta_com + _mpc_solver->getKprev() * _zmp_window;
    _mpc_solver->getIntegrator()->integrate(_delta_com, _u, _dt, _delta_com);
}
