#ifndef LATERAL_PLANE_H
#define LATERAL_PLANE_H

#include <XBotLogger/Logger.hpp>

class LateralPlane {

public:

    typedef std::shared_ptr< LateralPlane > Ptr;

    /* ask for:

     q_sensed
     q_max
     q_min
     state (empty, starting, stopping ..)
     dt
     t_impact (time) OR foot_pos_impact (space)

    */

    struct MpcOptions
    {
        MpcOptions()
        {
            Q << 1000000;
            R << 1;
        }

        /* height linear inverted pendulum */
        double h = 1.0;
        /* preview window resolution */
        double Ts = 0.01;
        /* horizon length */
        double T = 1.0;
        /* gains for mpc */
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
    };

    LateralPlane(double dt, MpcOptions opt = MpcOptions());

    void update(double q_sns,
                double q_max,
                double q_min,
                double zmp_val_current,
                double zmp_val_next,
                double duration_preview_window, /* should be constant, equal to size of _K_prev form MPC = duration_preview_window/dt */
                double duration_step,
                double middle_zmp  = 0., /* should be constant */
                double offset = 0.);

    /* get position of */
    double getDeltaCom(){return _delta_com(0);}

private:

    class MpcSolver;

    std::shared_ptr< MpcSolver > _mpc_solver;


    void addSegment(Eigen::VectorXd& vector, long size, double value);

    /* compute preview window */
    void computePreviewWindow(double q_sns,
                              double q_max,
                              double q_min,
                              double zmp_val_current,
                              double zmp_val_next,
                              double duration_preview_window,
                              double duration_step,
                              double middle_zmp,
                              double offset);

    Eigen::Vector3d _delta_com;

    double _q_sns, _q_sns_prev;

    double _alpha_old, _alpha;

    double _zmp_val;

    bool _switch_side_zmp;

    int _size_window, _size_step;

    double _delay_start;

    double _dt;

    Eigen::Matrix<double,1,1> _u;

    Eigen::VectorXd _zmp_window;

    MpcOptions _opt;


};

#include <walker/mpc_solver.h>

#endif // LATERAL_PLANE_H
