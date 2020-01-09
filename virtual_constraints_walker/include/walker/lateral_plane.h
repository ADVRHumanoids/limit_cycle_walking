#ifndef LATERAL_PLANE_H
#define LATERAL_PLANE_H

#include <XBotLogger/Logger.hpp>

class LateralPlane {
private:

    struct Options
    {

    };

    class MpcSolver;

    std::shared_ptr< MpcSolver > _mpc_solver;

    Eigen::VectorXd computePreviewWindow();

    Eigen::Vector3d getDeltaComLat(){return _delta_com_lat;};

    Eigen::Vector3d _delta_com_lat;
    double _q_sns, _q_sns_prev;

    double _alpha_old, _alpha;

    double _zmp;

    bool _switch_side_zmp;

    Eigen::VectorXd max_;

    int _size_window, _size_step;




public:

    /* ask for:

     q_sensed
     q_max
     q_min
     state (empty, starting, stopping ..)
     dt
     t_impact (time) OR foot_pos_impact (space)

    */

    LateralPlane();

    void update(double q_sns,
                double q_max,
                double q_min,
                double zmp_val,
                double duration_preview_window,
                double duration_step,
                double dt);

};

#include <walker/mpc_solver.h>

#endif // LATERAL_PLANE_H
