#include <engine/lateral_plane.h>

LateralPlane::LateralPlane(double dt, Options opt) :
    _alpha_old(0),
    _alpha(0),
    _switch_side_zmp(1),
    _size_current(0),
    _size_next(0),
    _dt(dt),
    _opt(opt)

{
    _zmp_val_current.resize(1);
    _zmp_val_current.setZero();

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
                                        Eigen::VectorXd zmp_val_current,
                                        Eigen::VectorXd zmp_val_next,
                                        double duration_current,
                                        double duration_next,
                                        double zmp_middle,
                                        double offset)
{
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

    _zmp_val_current = zmp_val_current;
    /* size of current stance (depending on alpha) */
    _size_current = static_cast<int>( duration_current / _dt ); // const
    /* size of the next stance */
    _size_next = static_cast<int>( duration_next / _dt ); // const

    /* if size_current_step is bigger than full size window, take size window */
    Eigen::VectorXd current_segment;
    int current_size = (_size_current > _size_window ? _size_window : _size_current);

    if (_zmp_val_current.size() == 1)
    {
        current_segment = current_segment.Constant(_size_current, _zmp_val_current[0]);
    }
    else if (_zmp_val_current.size() == 2)
    {
        current_segment = current_segment.LinSpaced(_size_current, _zmp_val_current[0], _zmp_val_current[1]);
    }
    else
    {
        /* TODO */
        throw;
    }

    _receding_size = static_cast<int>((1 - _alpha) * current_size);// changes at each step, receding

    /* fill current step with receding current_segment values */
    _zmp_window.head(_receding_size) << current_segment.tail(_receding_size);


    /* fill all the other segment (zmp value of all the step after the current) with next zmp (and then switch symmetrically around the middle)*/
    _zmp_val_next = zmp_val_next;

    Eigen::VectorXd chunk(_size_next + _size_current);
    if (_zmp_val_next.size() == 1)
    {
        chunk.head(_size_next) << chunk.Constant(_size_next, _zmp_val_next[0]);
    }
    else if (_zmp_val_next.size() == 2)
    {
        chunk.head(_size_next) << chunk.LinSpaced(_size_next, _zmp_val_next[0], _zmp_val_next[1]);
    }
    else
    {
        /* TODO */
        throw;
    }
    if (_zmp_val_current.size() == 1)
    {
        /* flip around middle zmp (zmp_current) for third chunk BEFORE filling third chunk */
        _zmp_val_current[0] = 2 * zmp_middle - _zmp_val_current[0];
        chunk.tail(_size_current) << chunk.Constant(_size_current, _zmp_val_current[0]);
    }
    else if (_zmp_val_current.size() == 2)
    {
        /* flip both vertices of zmp (zmp_current) for third chunk BEFORE filling third chunk */
        _zmp_val_current[0] = 2 * zmp_middle - _zmp_val_current[0];
        _zmp_val_current[1] = 2 * zmp_middle - _zmp_val_current[0];
        chunk.tail(_size_current) << chunk.LinSpaced(_size_current, _zmp_val_current[0], _zmp_val_current[1]);
    }
    else
    {
        /* TODO */
        throw;
    }

    long int size_remaining = _size_window - _receding_size;
    while (size_remaining > 0)
    {
        if (_size_next + _size_current < size_remaining)
        {
            _zmp_window.segment(_size_window - size_remaining, _size_next + _size_current) = chunk;
            chunk = 2 * Eigen::VectorXd::Constant(chunk.size(), zmp_middle) - chunk;
            size_remaining = size_remaining - (_size_next + _size_current);
        }
        else
        {
            _zmp_window.tail(size_remaining) = chunk.head(size_remaining);
            size_remaining = 0;
        }
    }



}

void LateralPlane::update(double q_sns,
                          double q_min,
                          double q_max,
                          Eigen::VectorXd zmp_val_current,
                          Eigen::VectorXd zmp_val_next,
                          double duration_current,
                          double duration_next,
                          double zmp_middle,
                          double offset)
{
    computePreviewWindow(q_sns, q_min, q_max, zmp_val_current, zmp_val_next, duration_current, duration_next, zmp_middle, offset);

    /* com value is actually a delta_com */
    _u = _mpc_solver->getKfb() * _delta_com + _mpc_solver->getKprev() * _zmp_window;
    _mpc_solver->getIntegrator()->integrate(_delta_com, _u, _dt, _delta_com);
}


void LateralPlane::log(std::string name, XBot::MatLogger::Ptr logger)
{
    logger->add(name + "_delta_com", _delta_com);
    logger->add(name + "_alpha_old", _alpha_old);
    logger->add(name + "_alpha", _alpha);
//    logger->add(name + "_zmp_val", _zmp_val);
    logger->add(name + "_switch_side_zmp", _switch_side_zmp);
    logger->add(name + "_size_window", _size_window);
    logger->add(name + "_size_current", _size_current);
    logger->add(name + "_size_next", _size_next);
    logger->add(name + "_dt", _dt);
    logger->add(name + "_u", _u);
    logger->add(name + "_zmp_window", _zmp_window);
    logger->add(name + "_zmp_tracked", _mpc_solver->getC()*_delta_com);

    _mpc_solver->log("mpc", logger);
}
