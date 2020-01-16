#ifndef WALKER_H
#define WALKER_H

#include <string>
#include <cartesian_interface/CartesianInterface.h>
#include <robot/step_state.h>

#include <walker/lateral_plane.h>
#include <walker/sagittal_plane.h>

class Walker {
public:

    typedef std::shared_ptr<Walker> Ptr;

    struct Options
    {
        Options();

        /* initial lean, used for tuning */
//        double lean_forward = 0;
        double zmp_offset = 0;
//        double crouch_height = 0;
//        bool initial_swing_leg = 0;
        double horizon_length = 5;
        double mpc_Q = 1000000;
        double mpc_R = 1;
        double double_stance_duration;

    };

    Walker(double dt, Options opt = Options());

    enum class Side { Left = 0, Right = 1, Double = -1 };  /*Side that is SWINGING*/     /*think a way to put here the values of step_y*/

    bool initialize(double time,
                    const mdof::StepState * state);

    bool compute(double time,
                 const mdof::StepState * state,
                 Eigen::Vector3d& delta_com,
                 Eigen::Affine3d& foot_goal);

    friend std::ostream& operator<<(std::ostream& os, Side s);

private:
    /* set homing configuration for robot */
    void idle();

    bool reset();

    /* dt inside walker */
    double _dt;

    /* reset q1 after each impact */
//    bool _reset_condition;

    /*time between instant that received START command is received and instant where robot starts walking. Required for initial lateral shift of the CoM*/
//    double _delay_start;

    LateralPlane::Ptr _lat;
    SagittalPlane::Ptr _sag;

//    bool _ready_flag;

    const Options _opt;

};


#endif // WALKER_H
