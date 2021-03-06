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
                                        std::vector<Eigen::MatrixXd> zmp_val,
                                        Eigen::VectorXd duration,
                                        double zmp_middle,
                                        double offset)
{

    /* compute alpha, phase variable normalized [0, 1] between q_max and q_min */
//    _alpha_old = (_q_sns_prev - q_min)/(q_max - q_min);
    _alpha = std::min(std::max((q_sns - q_min)/(q_max - q_min), 0.0), 1.0);


    /* fill zmp with given values */
    /* assumes that vector and array have same number of rows */

    double last_value;
    last_value = zmp_val.back().coeffRef(0, zmp_val.back().cols() -1);

    std::vector<Eigen::VectorXd> chunks;
    std::vector<int> sizes;
    int count = 0;
    /* generate all the chunk given the zmp values */
    for(auto val : zmp_val)
    {
        sizes.push_back(static_cast<int>( duration[count] / _dt ));
        chunks.push_back(makeChunk(val, sizes[count]));
        count++;
   }
   /* _size of zmp for a period (one step) is zmp_window + period */
   _zmp.resize(_zmp_window.size() + chunks[0].size());

   int pos = 0;

   /* fill zmp with chunks */
   for (auto chunk : chunks)
   {
       _zmp.segment(pos, chunk.size()) << chunk;
       pos = pos + chunk.size();
   }

   /* if the current chunks fill the zmp... */
   if (pos > _zmp_window.size() + sizes[0])
   {
       /* okay */
   }
   else /* otherwise...*/
   {
       int size_remaining = _zmp.size() - pos;

       while (size_remaining > 0)
       {

               last_value = 2 * zmp_middle - last_value;

               if (sizes.back() < size_remaining)
               {
                   _zmp.segment(_zmp.size() - size_remaining, sizes.back()) = makeChunk((Eigen::MatrixXd(1,1) << last_value).finished(), sizes.back() );
                   size_remaining = size_remaining - sizes.back();
               }
               else
               {
                   _zmp.tail(size_remaining) = makeChunk((Eigen::MatrixXd(1,1) << last_value).finished(), size_remaining);
                   size_remaining = 0;
               }
       }
   }
   _zmp_window << _zmp.segment(_alpha * chunks[0].size(), _zmp_window.size());
}

Eigen::VectorXd LateralPlane::makeChunk(Eigen::MatrixXd values, int size_current)
{
    Eigen::VectorXd segment(size_current);

    if (values.cols() == 1)
    {
        segment = segment.Constant(size_current, values(0,0));
    }
    else if (values.cols() == 2)
    {
        Eigen::VectorXd segment_support;
        segment.head(size_current/2) << segment_support.Constant(size_current/2, values(0,0));
        segment.tail(size_current/2) << segment_support.Constant(size_current/2, values(0,1));
//        segment = segment.LinSpaced(size_current, values(0,0), values(0,1));
    }
    else if (values.cols() == 3)
    {
        int segment_lenght = static_cast<int>(size_current/3);

        Eigen::VectorXd segment_support;
        segment.segment(0, segment_lenght) << segment_support.Constant(segment_lenght, values(0,0));
        segment.segment(segment_lenght, segment_lenght) << segment_support.Constant(segment_lenght, values(0,1));
        segment.segment(2 *segment_lenght, segment_lenght) << segment_support.Constant(segment_lenght, values(0,2));
    }
    else if (values.cols() == 4)
    {
        int segment_lenght = static_cast<int>(size_current/4);

        Eigen::VectorXd segment_support;
        segment.segment(0, segment_lenght) << segment_support.Constant(segment_lenght, values(0,0));
        segment.segment(segment_lenght, segment_lenght) << segment_support.Constant(segment_lenght, values(0,1));
        segment.segment(2 *segment_lenght, segment_lenght) << segment_support.Constant(segment_lenght, values(0,2));
        segment.segment(3 *segment_lenght, segment_lenght) << segment_support.Constant(segment_lenght, values(0,3));
    }
    else
    {
        /* TODO */
        throw;
    }

    return segment;
}

void LateralPlane::update(double q_sns,
                          double q_min,
                          double q_max,
                          std::vector<Eigen::MatrixXd> zmp_val_current,
                          Eigen::VectorXd duration,
                          double zmp_middle,
                          double offset)
{

    if (q_min == 0 && q_max == 0)
    {
        _zmp_window.setZero();
    }
    else
    {
        computePreviewWindow(q_sns, q_min, q_max, zmp_val_current, duration, zmp_middle, offset);
    }
    /* com value is actually a delta_com */
    _u = _mpc_solver->getKfb() * _delta_com + _mpc_solver->getKprev() * _zmp_window;
    _mpc_solver->getIntegrator()->integrate(_delta_com, _u, _dt, _delta_com);
}


void LateralPlane::log(std::string name, XBot::MatLogger2::Ptr logger)
{
    logger->add(name + "_delta_com", _delta_com);
    logger->add(name + "_alpha_old", _alpha_old);
    logger->add(name + "_alpha", _alpha);
    logger->add(name + "_zmp_val", _zmp_window[0]);
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
