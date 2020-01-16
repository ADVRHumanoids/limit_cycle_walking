#include <walker/walker.h>

Walker::Walker(double dt, Options opt) :
    _dt(dt),
    _opt(opt)
{
}

bool Walker::initialize(double time,
                        const mdof::StepState * state)
{
    /* before setting stuff, I need the homing */
//    homing(time, state, ref);

    /* required parameters here:
     * zmp indent
     * initial_swing_leg
     * duration preview window for MPC
     * Q for MPC
     * R for MPC
     */

//    _reset_condition = 0;

    /* com to sole (height of linear inverted pendulum for MPC) */
//    double height_lip = fabs(state->getCom().z() - state->getFoot()[state->getSwingLeg()].translation().z());

//    _delay_start = 1.5;

//    double step_lenght = fabs(4* fabs(state->getAnkleCom()[state->getSwingLeg()].translation().z()) * tan(state->getQMax()));

    /* steepness should be zero ? */
    /* zmp should be ? */
//    std::cout << "Initial angle: " << state->q << " rad)" << std::endl;
//    std::cout << "Step length: " <<  step_lenght << " m (Max angle of inclination: " << state->getQMax() << " rad)" <<  std::endl;
//    std::cout << "Step duration: " << state->step_duration << " s" << std::endl;
//    std::cout << "Double stance: " <<  _opt.double_stance_duration << " s" << std::endl;
//    std::cout << "Steepness: " << _steep_q <<  std::endl;
//    std::cout << "ZMP width correction: " << - state.get_indent_zmp() << " --> ZMP Right: " << _current_zmp << " and ZMP Left: " << _initial_zmp_y_right << std::endl;

    LateralPlane::MpcOptions _mpc_opt;

    _mpc_opt.Q << _opt.mpc_Q;
    _mpc_opt.R << _opt.mpc_R;
    _mpc_opt.T = _opt.horizon_length;
    _mpc_opt.h = state->height_robot;
    /* preview window resolution set as the control dt */
    _mpc_opt.Ts = _dt;

    _lat = std::make_shared<LateralPlane>(_dt, _mpc_opt);
    _sag = std::make_shared<SagittalPlane>(_dt);

    /* TODO check sanity */
    return true;

}

bool Walker::compute(double time,
                     const mdof::StepState * state,
                      Eigen::Vector3d& delta_com,
                      Eigen::Affine3d& foot_goal)
{   
    double q = state->q;
    double q_min = state->q_min;
    double q_max = state->q_max;
    double step_duration = state->step_duration;

    /* for now it is not needed, but it will be if I put here the trajectory generation of step, which now is computed using cartesio */
    double step_clearing = state->step_clearance;
    double zmp_current = state->zmp_val_current;
    double zmp_next = state->zmp_val_next;
    double height_robot = state->height_robot;


    /* compute stepping motion */
    /* STILL TODO the update of sag */
    _sag->update(q, height_robot);

    /* here can be put also middle_zmp and offset_zmp */
    _lat->update(q, q_max, q_min, zmp_current, zmp_next, _opt.horizon_length, step_duration);

    delta_com(0) = _sag->getDeltaCom();
    delta_com(1) = _lat->getDeltaCom();
    delta_com(2) = 0;

    foot_goal = _sag->getFootGoal();

    return true;

}

