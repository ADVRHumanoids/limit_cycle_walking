#include <walker/walker.h>

Walker::Walker(double dt) :
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

    homing();

    _reset_condition = 0;
    _q_max = state.q_max;
    _q_min = state.q_min;

    _steep_q = state.steep_q;

    _step_duration = state.step_duration;
    _step_clearing = state.step_clearing;

    _height = state.ankle_T_com(/* ...TODO ... */);

    _delay_start = 1.5;

    double step_lenght = fabs(4* fabs(state.ankle_T_com(state.foot_contact).z()) * tan(state.q_max));

    std::cout << "Lean forward: " << /*.... TODO ... */ << " m (Initial angle: " << _initial_q1 << " rad)" << std::endl;
    std::cout << "Step length: " << /* ... TODO ....*/ << " m (Max angle of inclination: " << _q1_max << " rad)" <<  std::endl;
    std::cout << "Step duration: " << _step_duration << " s" << std::endl;
    std::cout << "Double stance: " <<  _initial_param.get_double_stance() << " s" << std::endl;
    std::cout << "Steepness: " << _steep_q <<  std::endl;
    std::cout << "ZMP width correction: " << - state.get_indent_zmp() << " --> ZMP Right: " << _current_zmp << " and ZMP Left: " << _initial_zmp_y_right << std::endl;


}
bool Walker::idle()
{
    /* do nothing */
}

bool Walker::homing(double time,
                    const mdof::RobotState& state,
                    mdof::RobotState& ref)
{
    Eigen::Vector3d com_ref = state.com_pos;
    /* center the CoM w.r.t. the feet */
    com_ref(0) = state.foot_pos() + state.get_lean_forward();
    com_ref(1) = (_current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1) + _current_pose_ROS.get_sole(robot_interface::Side::Right).coeff(1))/2; //TODO put in
    com_ref(2) = _initial_param.get_crouch();

    /*TODO PUT DEFAULT POSITION*/
    _poly_com.set_com_initial_position(straight_com);
    std::cout << straight_com.transpose() << std::endl;
    return straight_com;
}

bool Walker::compute(double time,
                     const mdof::RobotState& state,
                     mdof::RobotState& ref)
{

    sagittal_plane.update(state, ref);
    lateral_plane();
//    if (_current_phase == Phase::Land)
//    {
//        /* manage landing */
//    }


    //    state.initial_stuff
    //    lateral_plane.update(delta_com)
    //    sagittal_plane.update(delta_com, delta_step)

    //    ref.com_traj[0] = state.com_sag_start + delta_com_sag;
    //    ref.com_traj[1] = state.com_lat_start + delta_com_lat;

    //    ref.step_traj = compute_traj(state.foot_start, state.foot_goal)

}
