#include <engine/lateral_plane.h>

LateralPlane::LateralPlane(double dt, Options opt) :
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
                                        double step_duration,
                                        double zmp_middle,
                                        double offset)
{
    /* TODO how to get prev */
//    _q_sns_prev = q_sns;

    /* compute alpha, phase variable normalized [0, 1] between q_max and q_min */
//    _alpha_old = (_q_sns_prev - q_min)/(q_max - q_min);
    _alpha = (q_sns - q_min)/(q_max - q_min);

    /* if alpha < 0 make it zero. This can happen if q_sns become */
    if (_alpha < 0)
    {
        std::cout << "Lateral plane. WARNING: alpha < 0 " << std::endl;
        _alpha = 0;
    }

    if (_alpha > 1)
    {
        std::cout << "Lateral plane. WARNING: alpha > 1 " << std::endl;
        _alpha = 1;
    }


    /* zmp to track */
    if (_alpha <= 1)
    {
        _zmp_val = zmp_val_current;
    }

    /* size of the step */
    _size_step = static_cast<int>( step_duration / _dt );

    /* size of current step (depending on alpha) */
    int size_current_step = static_cast<int>((1 - _alpha) * _size_step);

    /* current position in preview window (expressed as a time) */
//    double time = _alpha * fabs( step_duration );

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
            /* switch _zmp side symmetric to zmp_middle */
            _zmp_val = 2 * zmp_middle - _zmp_val;
            /* add offset if needed */
            _zmp_val = _zmp_val + ( ( (_zmp_val - zmp_middle) > 0) - ( (_zmp_val - zmp_middle) < 0) ) * offset ;

        }
        else
        {
            Eigen::VectorXd segment(size_remaining);
            _zmp_window.tail(size_remaining) << _zmp_val * segment.setOnes();
            size_remaining = 0;

        }
    }
}

void LateralPlane::update(double q_sns,
                          double q_min,
                          double q_max,
                          double zmp_val_current,
                          double zmp_val_next,
                          double step_duration,
                          double zmp_middle,
                          double offset)
{
    computePreviewWindow(q_sns, q_min, q_max, zmp_val_current, zmp_val_next, step_duration, zmp_middle, offset);

    /* com value is actually a delta_com */
    _u = _mpc_solver->getKfb() * _delta_com + _mpc_solver->getKprev() * _zmp_window;
    _mpc_solver->getIntegrator()->integrate(_delta_com, _u, _dt, _delta_com);
}

void LateralPlane::log(std::string name, XBot::MatLogger::Ptr logger)
{
    logger->add(name + "_delta_com", _delta_com);
    logger->add(name + "_alpha_old", _alpha_old);
    logger->add(name + "_alpha", _alpha);
    logger->add(name + "_zmp_val", _zmp_val);
    logger->add(name + "_switch_side_zmp", _switch_side_zmp);
    logger->add(name + "_size_window", _size_window);
    logger->add(name + "_size_step", _size_step);
    logger->add(name + "_dt", _dt);
    logger->add(name + "_u", _u);
    logger->add(name + "_zmp_window", _zmp_window);
    logger->add(name + "_delay_start", _delay_start);
    logger->add(name + "_zmp_tracked", _mpc_solver->getC()*_delta_com);

    _mpc_solver->log("mpc", logger);
}
