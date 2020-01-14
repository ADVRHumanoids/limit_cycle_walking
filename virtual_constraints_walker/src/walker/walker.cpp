#include <walker/walker.h>

Walker::Walker(double dt, Options opt) :
    _opt(opt),
    _dt(dt),
    _reset_condition(false),
    _delay_start(0),
    _current_side(Side::Double), _other_side(Side::Double),
    _current_swing_leg(0),
    _q(0),
    _q_max(0),
    _q_min(0),
    _step_duration(0),
    _step_clearing(0),
    _steep_q(0),
    _height(0)

{
    _com_x.setZero();
    _com_y.setZero();
    _current_zmp.setZero();
}

bool Walker::initialize(double time, const mdof::RobotState& state)
{
    /* before setting stuff, I need the homing */
    homing();

    /* required parameters here:
     * zmp indent
     * crouch height
     * initial_swing_leg
     * duration preview window for MPC
     * Q for MPC
     * R for MPC
     *
     */
    _reset_condition = 0;
    _q_max = state.getQMin();
    _q_min = state.getQMax();

    _steep_q = state.getSteepQ(); /* should be 0! */

    _step_duration = state.getStepDuration();
    _step_clearing = state.getStepClearing();

    /* com to sole (height of linear inverted pendulum for MPC) */
    double height_lip = fabs(state.getCom().z() - state.getFoot()[_current_swing_leg].translation().z());

    _delay_start = 1.5;

    double step_lenght = fabs(4* fabs(state.getAnkleCom()[_current_swing_leg].translation().z()) * tan(state.getQMax()));

    Eigen::Affine3d l_foot = state.getFoot()[0];
    Eigen::Affine3d r_foot = state.getFoot()[1];


    std::cout << "Lean forward: " << _opt.lean_forward << " m (Initial angle: " << _initial_q1 << " rad)" << std::endl;
    std::cout << "Step length: " <<  step_lenght << " m (Max angle of inclination: " << _q_max << " rad)" <<  std::endl;
    std::cout << "Step duration: " << _step_duration << " s" << std::endl;
    std::cout << "Double stance: " <<  _opt.double_stance_duration << " s" << std::endl;
//    std::cout << "Steepness: " << _steep_q <<  std::endl;
//    std::cout << "ZMP width correction: " << - state.get_indent_zmp() << " --> ZMP Right: " << _current_zmp << " and ZMP Left: " << _initial_zmp_y_right << std::endl;

    LateralPlane::MpcOptions _mpc_opt;

    _mpc_opt.Q << _opt.mpc_Q;
    _mpc_opt.R << _opt.mpc_R;
    _mpc_opt.T = _opt.horizon_length;
    _mpc_opt.h = height_lip;
    /* preview window resolution set as the control dt */
    _mpc_opt.Ts = _dt;

    _lat = std::make_shared<LateralPlane>(_dt, _mpc_opt);
    _sag = std::make_shared<SagittalPlane>(_dt);


}

bool Walker::homing(double time,
                    const mdof::RobotState * state,
                    mdof::RobotState * ref)
{
    /* com homing  */

    Eigen::Vector3d com_goal = state->getCom();
    /* center the CoM w.r.t. the feet */
    com_goal(0) = state->getFoot()[_current_swing_leg].translation()(0) + _opt.lean_forward;
    com_goal(1) = (state->getFoot()[0].translation()(1) + (state->getFoot()[1].translation()(1)))/2; //TODO put in
    com_goal(2) = _opt.crouch_height;

    ref->setComStart(state->getCom());
    ref->setComGoal(com_goal);

    Eigen::Affine3d l_foot = state->getFoot()[0];
    Eigen::Affine3d r_foot = state->getFoot()[1];

    Eigen::Quaterniond _orientation_goal;



    /* unrequired? */
    _ready_flag = 1;
}

bool Walker::compute(double time,
                     const mdof::RobotState * state,
                     mdof::RobotState * ref)
{

    /*
     * I should probably put here the mechanics to move com before step at the beginning

     */
    if (time < state->getStartTime())
    {
        /* do nothing */
    }

    if (time > state->getStartTime() && time < state->getStartTime() + _delay_start)
    {
        /* this fakes a _q in the beginning, so that the com moves before the step */
        _q = state->getSteepQ() * time;
    }

    _lat->update(state->getQ(), state->getQMax(), state->getQMin(), zmp_val, duration_preview_window, duration_step);
    _sag->update(delta_com_lat, delta_step);

    ref.com_traj[0] = state.com_sag_start + delta_com_sag;
    ref.com_traj[1] = state.com_lat_start + delta_com_lat;

    ref.step_traj = compute_traj(state.foot_start, state.foot_goal);

}


