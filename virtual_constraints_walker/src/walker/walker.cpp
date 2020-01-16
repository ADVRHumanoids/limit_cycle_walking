#include <walker/walker.h>

Walker::Walker(double dt, Options opt) :
    _dt(dt),
    _reset_condition(false),
    _delay_start(0),
    _opt(opt)
{
}

bool Walker::initialize(double time,
                        const mdof::RobotState * state)
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

    _reset_condition = 0;

    /* com to sole (height of linear inverted pendulum for MPC) */
    double height_lip = fabs(state->getCom().z() - state->getFoot()[state->getSwingLeg()].translation().z());

    _delay_start = 1.5;

    double step_lenght = fabs(4* fabs(state->getAnkleCom()[state->getSwingLeg()].translation().z()) * tan(state->getQMax()));

    /* steepness should be zero ? */
    /* zmp should be ? */
    std::cout << "Initial angle: " << state->getQ() << " rad)" << std::endl;
    std::cout << "Step length: " <<  step_lenght << " m (Max angle of inclination: " << state->getQMax() << " rad)" <<  std::endl;
    std::cout << "Step duration: " << state->getStepDuration() << " s" << std::endl;
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

bool Walker::compute(double time,
                     const mdof::RobotState * state,
                     mdof::RobotState * ref)
{   
    /* get values from state*/
    double q = state->getQ();
    double q_min = state->getQMin();
    double q_max = state->getQMax();
    bool current_swing_leg = state->getSwingLeg();
    double t_start_walk = state->getStartWalkTime();
    double step_duration = state->getStepDuration();
    double step_clearing = state->getStepClearing();
    Eigen::Vector3d com_pos_start = state->getCom();
    std::array< Eigen::Affine3d, 2 > foot_pos_start = state->getFootStart();
//    std::array< Eigen::Affine3d, 2 > foot_pos_goal = state->getFootGoal();
    double t_start = state->getTStart();
//    double t_end = state->getTEnd();
    double current_zmp = state->getZmp();
    double theta = state->getTheta();
    Eigen::Affine3d waist_pos_start = state->getWaist();

    Eigen::Rotation2Dd rot2(theta);

    if (time < t_start_walk)
    {
        /* do nothing */
    }

    if (time > t_start_walk && time < t_start_walk + _delay_start)
    {
        /* this fakes a _q in the beginning, so that the com moves before the step */
        q = state->getSteepQ() * time;
    }

    double height_robot = state->getAnkleCom()[1 - current_swing_leg].translation().z();

    /* compute stepping motion */
    _sag->update(q, height_robot);
    _lat->update(q, q_max, q_min, current_zmp, _opt.horizon_length, step_duration);

    /* compute com trajectory */
    Eigen::Vector3d com_ref;

    com_ref(0) = com_pos_start(0) + _sag->getDeltaCom();
    com_ref(1) = com_pos_start(1) + _lat->getDeltaCom();
    com_ref(2) = com_pos_start(2);

    /* rotate com_ref if needed */
    com_ref.head(2) = rot2.toRotationMatrix() * com_ref.head(2);


    /*compute feet trajectories */
    std::array<Eigen::Affine3d, 2> feet_ref;

    /* set foot_goal and t_goal */
    /* THIS IS NECESSARY ONLY WHEN NEW STEP IS ISSUED */
    std::array< Eigen::Affine3d, 2 > foot_pos_goal = state->getFootStart();
    foot_pos_goal[current_swing_leg] = _sag->getFootGoal();
    ref->setFootGoal(foot_pos_goal);
    t_start = state->getTStart();/* which is also t_impact */
    ref->setTEnd(t_start + state->getStepDuration()); /* t_end */

    feet_ref[1 - current_swing_leg] = foot_pos_start[1 - current_swing_leg];

    /* here the feet trajectory should be a function of q, not t */
    feet_ref[current_swing_leg] = mdof::computeTrajectory(foot_pos_start[current_swing_leg],
                                                          ref->getFootGoal()[current_swing_leg],
                                                          step_clearing,
                                                          t_start,
                                                          ref->getTEnd(),
                                                          time);

    /* appearing to be useless:
     * robot state t_end
     * robot state foot_goal
     */
    /* compute waist trajectory */
    Eigen::Affine3d waist_ref = waist_pos_start;

    waist_ref.linear() = (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())).toRotationMatrix();

    /* set trajectories */
    ref->setCom(com_ref);
    ref->setFoot(feet_ref);
    ref->setWaist(waist_ref);
}

