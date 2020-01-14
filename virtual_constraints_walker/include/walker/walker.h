#ifndef WALKER_H
#define WALKER_H

#include <string>
#include <cartesian_interface/CartesianInterface.h>
#include <robot/robot_state.h>

#include <walker/lateral_plane.h>
#include <walker/sagittal_plane.h>

class Walker {
public:

    typedef std::shared_ptr<Walker> Ptr;

    struct Options
    {
        Options();

        /* initial lean, used for tuning */
        double lean_forward = 0;
        double zmp_offset = 0;
        double crouch_height = 0;
        bool initial_swing_leg = 0;
        double horizon_length = 5;
        double mpc_Q = 1000000;
        double mpc_R = 1;
        double double_stance_duration;

    };

    Walker(double dt, Options opt = Options());

    enum class Side { Left = 0, Right = 1, Double = -1 };  /*Side that is SWINGING*/     /*think a way to put here the values of step_y*/

    bool initialize(double time, const mdof::RobotState& state);

    bool compute(double time,
                const mdof::RobotState * state,
                mdof::RobotState * ref);

    friend std::ostream& operator<<(std::ostream& os, Side s);

private:
    /* set homing configuration for robot */
    void idle();

    void walk(double time,
              const mdof::RobotState& state,
              mdof::RobotState& ref);

    bool homing(double time,
                const mdof::RobotState& state,
                mdof::RobotState& ref);

    bool reset();

    /* dt inside walker */
    double _dt;

    /* reset q1 after each impact */
    bool _reset_condition;

    /*time between instant that received START command is received and instant where robot starts walking. Required for initial lateral shift of the CoM*/
    double _delay_start;

    /* fist step side */
    Side _current_side, _other_side;

    /* swinging leg */
    bool _current_swing_leg;

    /* phase variable (and its max and min) */
    double _q, _q_max, _q_min;

    /* duration of step */
    double _step_duration;

    /* clearing of step */
    double _step_clearing;

    /* steepness of the phase variable --> (_q1_max - _q1_min)/_step_duration */
    double _steep_q;

    /* current height */
    double _height;

    /* com trajectory */
    Eigen::Vector3d _com_x, _com_y;

    /* current zmp */
    Eigen::Vector2d _current_zmp;

    LateralPlane::Ptr _lat;
    SagittalPlane::Ptr _sag;

    bool _ready_flag;

    const Options _opt;

};


#endif // WALKER_H
