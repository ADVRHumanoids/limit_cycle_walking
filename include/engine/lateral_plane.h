#ifndef LATERAL_PLANE_H
#define LATERAL_PLANE_H

#include <XBotLogger/Logger.hpp>

class LateralPlane {

    /**
      * now duration_preview_window is CONSTANT. Cannot be changed online
     **/

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

    struct Options
    {
        Options() :
            h(1.0),
            horizon_duration(5),
            Ts(0.01),
            Q(1,1),
            R(1,1)
        {
            Q << 1000000;
            R << 1;
        }

        /* height linear inverted pendulum */
        double h;
        /* duration window */
        double horizon_duration;
        /* preview window resolution */
        double Ts;
        /* gains for mpc */
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
    };

    LateralPlane(double dt, Options opt = Options());

    void update(double q_sns,
                double q_min,
                double q_max,
                std::vector<Eigen::MatrixXd> zmp_val,
                Eigen::VectorXd duration,
                double zmp_middle  = 0., /* should be constant */
                double offset = 0.);

    /* get position of */
    double getDeltaCom(){return _delta_com(0);}

    Eigen::VectorXd getPreviewWindow(){return _zmp_window;}

    void log(std::string name, XBot::MatLogger::Ptr logger);

private:

    class MpcSolver;

    Eigen::VectorXd makeChunk(Eigen::MatrixXd values, int size_current);

    /* compute preview window */
    void computePreviewWindow(double q_sns,
                              double q_min,
                              double q_max,
                              std::vector<Eigen::MatrixXd> zmp_val,
                              Eigen::VectorXd duration,
                              double zmp_middle,
                              double offset);

    Eigen::Vector3d _delta_com;

    double _alpha_old, _alpha;

    Eigen::VectorXd _zmp_val_current, _zmp_val_next;
    Eigen::VectorXd _zmp;


    bool _switch_side_zmp;

    /* size of window, current phase and next phase and receding current phase*/
    int _size_window;
    int _size_current, _size_next, _receding_size;

    double _dt;

    Eigen::Matrix<double,1,1> _u;

    Eigen::VectorXd _zmp_window;

    std::shared_ptr< MpcSolver > _mpc_solver;

    Options _opt;


};

#include <engine/mpc_solver.h>

#endif // LATERAL_PLANE_H
