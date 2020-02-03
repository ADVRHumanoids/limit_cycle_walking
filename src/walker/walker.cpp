#include <walker/walker.h>
#include <stdexcept>

Walker::Walker(double dt, std::shared_ptr<Param> par) :
    _current_state(State::Idle),
    _current_event(Event::Empty),
    _step_counter(0),
    _cycle_counter(0),
    _steep_q(0),
    _t_impact(0),
    _theta(0),
    _time(0),
    _new_event_time(0),
    _started(false),
    _t_start_walk(0),
    _q(0),
    _q_min(0),
    _q_max(0),
    _q_fake(0),
    _dt(dt),
    _current_swing_leg(0),
    _delay_start(0),
    _terrain_height(0),
    _step_t_start(0),
    _step_t_end(0),
    _step_duration(0),
    _step_clearance(0),
    _zmp_middle(0),
    _disable_step(false),
    _update_step(false),
    _param(par)
{
    _com_pos_start.setZero();
    _com_pos_goal.setZero();

    _foot_pos_start[0].matrix().setZero();
    _foot_pos_start[1].matrix().setZero();

    _foot_pos_goal[0].matrix().setZero();
    _foot_pos_goal[1].matrix().setZero();
    _waist_pos_start.matrix().setZero();
    _waist_pos_goal.matrix().setZero();
}

bool Walker::init(const mdof::RobotState &state)
{
    _terrain_height = state.world_T_foot[_current_swing_leg].translation()(2);
    _current_swing_leg = _param->getFirstSideStep();
    _zmp_middle = (state.world_T_foot[0].translation()(1) + state.world_T_foot[1].translation()(1)) /2;
    _delay_start = 1.5;
    _com_pos_start = state.world_T_com;
    _com_pos_goal = state.world_T_com;
    _step_duration = _param->getStepDuration();
    /* TODO _q depends on current _q_max, so I need to update _q_max before _q */
    _q_max_previous = 0;
    _q_max = _param->getMaxInclination();
    _q = computeQ(_current_swing_leg, _theta, state.world_T_com, _com_pos_start, state.ankle_T_com);
    /* TODO this should be _q as starting, but since I am using a small hack for starting, I'm beginning from 0
     * so that after the first part (up to _start + _delay) is at the right value
     */
    _q_min = _q;

    _foot_pos_start[0] = state.world_T_foot[0];
    _foot_pos_start[1] = state.world_T_foot[1];

    _foot_pos_goal = _foot_pos_start;

    /* initialize the Engine of Walker */
    Engine::Options eng_opt;
    eng_opt.mpc_Q = _param->getMpcQ();
    eng_opt.mpc_R = _param->getMpcR();
    eng_opt.zmp_offset = _param->getZmpOffset();
    eng_opt.horizon_duration = _param->getHorizonDuration();
    eng_opt.double_stance_duration = _param->getDoubleStanceDuration();

    _engine = std::make_shared<Engine>(_dt, eng_opt);


    /* TODO: is this right? I don't think so */
    _height_com = fabs(state.world_T_com[2] - state.world_T_foot[_current_swing_leg].translation()[2]);

    _distance_ankle_com = state.ankle_T_com[1 - _current_swing_leg].translation().z();

    _zmp_val_current = state.world_T_foot[_current_swing_leg].translation()[1];
    _zmp_val_current = _zmp_val_current + ( ( (_zmp_val_current - _zmp_middle) > 0) - ( (_zmp_val_current - _zmp_middle) < 0) ) * _param->getZmpOffset();

    _zmp_val_next = state.world_T_foot[1 - _current_swing_leg].translation()[1];
    _zmp_val_next = _zmp_val_next + ( ( (_zmp_val_next - _zmp_middle) > 0) - ( (_zmp_val_next - _zmp_middle) < 0) ) * _param->getZmpOffset();

    updateStep();

    _engine->initialize(_step);
    /* TODO sanity check? */
    return true;
}

bool Walker::homing(const mdof::RobotState &state,
                    mdof::RobotState &ref)
{
    /* com homing  */
    Eigen::Vector3d com = state.world_T_com;

    /* center the com w.r.t. the feet */
    com(0) = state.world_T_foot[_current_swing_leg].translation()(0) + _param->getLeanForward();
    com(1) = (state.world_T_foot[0].translation()(1) + (state.world_T_foot[1].translation()(1)))/2;
    com(2) = state.world_T_com[2] + _param->getInitialLowering();

    std::array<Eigen::Affine3d, 2> feet = {state.world_T_foot[0], state.world_T_foot[1]};


    Eigen::Quaterniond _orientation_goal;
    _orientation_goal.setIdentity();

    feet[0].linear() = _orientation_goal.normalized().toRotationMatrix();
    feet[1].linear() = _orientation_goal.normalized().toRotationMatrix();

    /* set com and feet in ref */
    ref.world_T_com = com;
    ref.world_T_foot = feet;

     /* TODO sanity check? */
    return true;
}

bool Walker::start()
{
    _previous_event = _current_event;
    _current_event = Event::Start;
     /* TODO sanity check? */
    return true;
}

bool Walker::stop()
{
    _previous_event = _current_event;
    _current_event = Event::Stop;
     /* TODO sanity check? */
    return true;
}

bool Walker::setQMax(std::vector<double> q_max)
{
    for (auto i : q_max)
    {
        _q_buffer.push_back(i);
    }
    /* TODO sanity check? */
    return true;
}

bool Walker::setTheta(std::vector<double> theta)
{
    /* sanity check? */
    for (auto i : theta)
    {
        _theta_buffer.push_back(i);
    }
    /* TODO sanity check? */
    return true;
}

bool Walker::updateQMax(double time)
{
    /* update q_max */
    /* TODO update only if it started walking */
    if (_started == 1 && time < _t_start_walk + _delay_start)
    {
        return false;
    }

    if (_q_buffer.empty())
    {
//        _q_max_previous = _q_max;
        _q_max = _param->getMaxInclination();
    }
    else
    {
//        _q_max_previous = _q_max;
        _q_max = _q_buffer.front();
        _q_buffer.pop_front();
    }
    return true;
}

bool Walker::updateStep()
{
    _step.q = _q;
    _step.q_fake = _q_fake;
    _step.q_min = _q_min;
    _step.q_max = _q_max;
    _step.height_com = _height_com;
    _step.distance_ankle_com = _distance_ankle_com;
    _step.step_duration = _step_duration;
    _step.step_clearance = _step_clearance;
    _step.zmp_val_current = _zmp_val_current;
    _step.zmp_val_next = _zmp_val_next;
    _step.disable_step = _disable_step;
    _step.t_min = _t_min;
    _step.t_max = _t_max;

    return true;
}

bool Walker::update(double time,
                         const mdof::RobotState &state,
                         mdof::RobotState &ref)
{

    /* update _q_max */
//    updateQMax(time);

    /* update _q given the robot state */
    _q = computeQ(_current_swing_leg, _theta, state.world_T_com, _com_pos_start, state.ankle_T_com);

    /*update step duration. Right now, it is constant, the first one given in the param */
    _step_duration = _param->getStepDuration();

    /*compute new q_steep for qFake */
    _steep_q = (_q_max - _q_min)/ _step_duration;

    /* update _q_fake: this start computing at _t_start_walk + _delay_start*/
    computeQFake(time, _q, _q_min, _q_max, _steep_q, _started, _t_start_walk + _delay_start, _q_fake);

    /* update zmp values */
    _zmp_val_current = state.world_T_foot[_current_swing_leg].translation()(1);
    _zmp_val_current = _zmp_val_current + ( ( (_zmp_val_current - _zmp_middle) > 0) - ( (_zmp_val_current - _zmp_middle) < 0) ) * _param->getZmpOffset();

    _zmp_val_next  = state.world_T_foot[1 - _current_swing_leg].translation()(1);
    _zmp_val_next = _zmp_val_next + ( ( (_zmp_val_next - _zmp_middle) > 0) - ( (_zmp_val_next - _zmp_middle) < 0) ) * _param->getZmpOffset();


    /* t_impact gets updated every time an impact occurs */
    _step_t_start = _t_impact;

    /* which is also t_impact */
     _step_t_end = _step_t_start + _step_duration;


    _disable_step = false;

    if (!_started == 1)
    {
        /* if time is never step */
        _step_t_start = time + 1;
        /* if walking is not started, zmp stays in the middle */
        _zmp_val_current = _zmp_middle;
        _zmp_val_next = _zmp_middle;
    }

    if (_started == 1 && time >= _t_start_walk && time < _t_start_walk + _delay_start)
    {
        _disable_step = true;
        /* TODO HACK to fake the starting lateral swing, since not q nor q_fake are actually moving in this period
         * step_start and step_end goes into step, I'm using them to move the window
         */
        _t_min = _t_start_walk;
        _t_max = _t_start_walk + _delay_start;
        /* delay -first- step to let the com swing laterally */
        _step_t_start = _t_start_walk + _delay_start;
        /* if walking is started, zmp current is still in the middle, but next zmp is the first step */
        _zmp_val_current = _zmp_middle;
        /* TODO could be wrong? */
        _zmp_val_next = state.world_T_foot[_current_swing_leg].translation()(1);
    }

    if (_current_state == State::LastStep)
    {
        /* for the ending motion of the last step */
        _zmp_val_next = _zmp_middle;
    }

    /* -- until here everything is updated -- */

    /* this takes:
     * time
     * q
     * q_min
     * q_max
     * terrain_heigth
     */
    /* this changes:
     * q_min
     * com_start
     * q_fake
     */
    landingHandler(time, state);

    /* this changes:
     * q_max
     * q_min
     * q_steep
     */
    step_machine(time);



    Eigen::Rotation2Dd rot2(_theta);
    /* TODO are these constant? */
    _distance_ankle_com = state.ankle_T_com[1 - _current_swing_leg].translation().z();
    _height_com = fabs(state.world_T_com[2] - state.world_T_foot[_current_swing_leg].translation()[2]);

    /* TODO do something intelligent with this */
    /* FILL STEP STATE */
    updateStep();

    Eigen::Vector3d delta_com;
    Eigen::Vector3d delta_foot_tot;
    Eigen::Vector3d delta_com_tot;

    /* compute com and foot displacement */
    _engine->computeCom(time, _step, delta_com);

    if (_update_step)
    {
        if (_current_state != State::LastStep)
        {
            updateQMax(time);
        }

        Eigen::Vector3d delta_com_tot;
        Eigen::Vector3d delta_foot_tot;

        _engine->computeStep(time, _step, delta_foot_tot, delta_com_tot);

        delta_com_tot.head(2) = rot2.toRotationMatrix() * delta_com_tot.head(2);
        _com_pos_goal = _com_pos_start + delta_com_tot;

        delta_foot_tot.head(2) = rot2.toRotationMatrix() * delta_foot_tot.head(2);
        _foot_pos_goal[_current_swing_leg].translation() = _foot_pos_start[_current_swing_leg].translation() + delta_foot_tot;
        _foot_pos_goal[_current_swing_leg].linear() = (Eigen::AngleAxisd(_theta, Eigen::Vector3d::UnitZ())).toRotationMatrix();

        /* burn update_step */
        _update_step = false;
    }

    /* compute com trajectory and rotate if needed */
    delta_com.head(2) = rot2.toRotationMatrix() * delta_com.head(2);

    Eigen::Vector3d com_ref;
    com_ref = _com_pos_start + delta_com;

//    com_ref(0) = _com_pos_start(0) + delta_com(0);
//    com_ref(1) = delta_com(1);
//    com_ref(2) = _com_pos_start(2) + delta_com(2);

    /*compute feet trajectories and rotate if needed */   
    std::array<Eigen::Affine3d, 2> feet_ref;
    feet_ref[1 - _current_swing_leg] = _foot_pos_start[1 - _current_swing_leg];

    /* here the feet trajectory should be a function of q, not t */
    feet_ref[_current_swing_leg] = mdof::computeTrajectory(_foot_pos_start[_current_swing_leg],
                                                           _foot_pos_goal[_current_swing_leg],
                                                           _step_clearance,
                                                           _step_t_start,
                                                           _step_t_end,
                                                           time);

    /* rotate waist */
    Eigen::Affine3d waist_ref = _waist_pos_start;
    waist_ref.linear() = (Eigen::AngleAxisd(_theta, Eigen::Vector3d::UnitZ())).toRotationMatrix();

    /* set RobotState ref*/
    ref.world_T_com = com_ref;
    ref.world_T_foot = feet_ref;
    ref.world_T_waist = waist_ref;

    /*TODO sanity check? */
    return true;
}

std::shared_ptr<Walker::Param> Walker::getDefaultParam()
{
    std::shared_ptr<Param> par;
    par = std::make_shared<Param>();
    return par;
}

bool Walker::impactDetector(double time,
                            double q, /* TODO fake or not? */
                            double q_min,
                            double q_max,
                            double swing_leg_heigth,
                            double terrain_heigth)
{
    bool flag_q(false);
    bool flag_step(false);

    /* first condition of impact: q reaches q_max */
    if (q_min <= q_max)
    {
        flag_q = (q >= q_max);
    }
    else if (q_min > q_max)
    {
        flag_q = (q <= q_max);
    }

    /* second condition of impact: sole impacts ground (a certain treshold is reached) */
    flag_step = fabs(fabs(swing_leg_heigth) - fabs(terrain_heigth)) <= 1e-3;

    if (flag_step && flag_q)
    {
        _t_impact = time;
        return true;
    }

    return false;
}


bool Walker::landingHandler(double time,
                            const mdof::RobotState &state)
{
    /* terrain_height
     * current_swing_leg
     * q_fake
     * q
     * q_min
     * q_max
     * com_start
     * foot_start
     * _q_buffer
     */

    /* TODO what if in IDLE? THINK ABOUT THIS */
    if (impactDetector(time, _q, _q_min, _q_max, state.world_T_foot[_current_swing_leg].translation()(2), _terrain_height))
    {
        _previous_event = _current_event;
        _current_event = Event::Impact;

        // ---------------------------
        _q_min = - _q; /* HACK ALERT */
        _q_max_previous = _q_max;
        // ---------------------------

        /* reset q_fake */
        _q_fake = 0;

        std::string side;
        (_current_swing_leg) ? side = "LEFT" : side = "RIGHT";
        std::cout << "Impact! Current side: " << side << std::endl;


        /* TODO update com start: WITH REAL OR WITH PLANNED? this is basically a way to get rid of the com jump in the last step */
//        _com_pos_start = state.world_T_com;
        _com_pos_start = _com_pos_goal;

        /* this is probably necessary! */
        _foot_pos_start = state.world_T_foot;

        _current_swing_leg = 1 - _current_swing_leg;

        (_current_swing_leg) ? side = "LEFT" : side = "RIGHT";
        std::cout << "State changed. Current side: " << side << std::endl;

        return 1;
    }
    else
    {
        return 0; // if impact detector does not detect an impact
    }
}

bool Walker::computeQFake(double time,
                          double q,
                          double q_min,
                          double q_max,
                          double steep_q,
                          bool started,
                          double start_walk,
                          double& q_fake)
{
    if (started == 1 && time >= start_walk)
    {
        bool cond_q(false);

        if (q_min <= q_max)
        {
            cond_q = (q >= q_max);
        }
        else if (_q_min > q_max)
        {
            cond_q = (q <= q_max);
        }
        if (cond_q)
        {
            /* do nothing */
        }
        else
        {
            q_fake = q_fake + steep_q*(_dt); // basically q = a*t
        }
    }
    /* TODO sanity check? */
    return true;
}

double Walker::computeQ(bool current_swing_leg,
                             double theta,
                             Eigen::Vector3d world_T_com,
                             Eigen::Vector3d world_T_com_start,
                             std::array<Eigen::Affine3d, 2> ankle_T_com)
{
    /* takes:
     * ankle_T_com
     * world_T_com
     * theta
     * offset q, which is q_max TODO
     * current_stance_leg
     */
    /* TODO remember the stuff about offset ! */

    Eigen::Vector3d dist_com;
    double q;
    double offset_q;
    /* 2D rot matrix, theta in radian */
    Eigen::Rotation2Dd rot2(theta);


    /* distance between current com and starting com (updated at each step) */
    dist_com = world_T_com - world_T_com_start;

    /* rotate back com */
    dist_com.head(2) = rot2.toRotationMatrix() * dist_com.head(2);

    /* TODO depending on the length of the step, remove offset */
    offset_q = _q_max_previous;

    /* compute q, inclination angle of the robot */
    q = atan( dist_com(0) / fabs(ankle_T_com[1 - current_swing_leg].translation()(2)) ) - offset_q;

    return q;
}

//bool Walker::resetter(const mdof::RobotState * state,
//                           mdof::RobotState * ref)
//{
////     generate_starting_zmp();

//    Eigen::Vector3d new_com(state->getCom());
//    new_com(0) = state->getCom()(0) - state->getAnkleCom()[_current_swing_leg].coeff(2) * tan(_initial_param.get_max_inclination());
//    ref->setCom(new_com);
//}

bool Walker::step_machine(double time)
{
    /* q_max
     * q_min
     * started
     * step_counter
     * state
     * steep_q
     */

    std::cout << "Entering step machine with event: '" << _current_event << "' during state: '" << _current_state << "'" << std::endl;

    if (_previous_event != _current_event)
    {
        _new_event_time = _time;
    }

    switch (_current_event)
    {
    case Event::Impact :
        switch (_current_state)
        {
        case State::Idle :
            throw std::runtime_error(std::string("Something is wrong. Impact during IDLE"));

        case State::Walking :
            _step_counter++;
            _update_step = true;
            break;

        case State::Starting :
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::Walking;
            _update_step = true;
            break;

        case State::Stopping :
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::LastStep;
            _update_step = true;

            /* TODO set q_max to zero (is this enough to make it half step?)*/
            _q_max_previous = _q_max;
            _q_max = 0;
            break;

        case State::LastStep :
            _step_counter++;

            /* TODO 'started' flag set to False */
            _started = 0;

            _previous_state = _current_state;
            _current_state = State::Idle;
            _steep_q = 0;
            _q_max = _param->getMaxInclination();
//            TODO resetter();
            break;
        }
        break;

    case Event::Start :
    {
        switch (_current_state)
        {

        case State::Idle :
            _previous_state = _current_state;
            _current_state = State::Starting;

            /* put it somewhere else */
//            _q_max_previous = _q_max;
//            _q_max = _param->getMaxInclination();
            _step_counter++;
            _cycle_counter++;

            /* 'started' flag set to True */
            _started = 1;

            _t_start_walk = time;

            _update_step = 1;
            break;

        default : /*std::cout << "Ignored starting event. Already WALKING" << std::endl; */
            break;
        }
        break;
    }
    case Event::Stop :
    {
        switch (_current_state)
        {
        case State::Idle :
            /* std::cout << "Ignored stopping event. Already in IDLE" << std::endl; */
            break;

        case State::Walking :
        case State::Starting :
            _previous_state = _current_state;
            _current_state = State::Stopping; // TODO replan as soon as I get the message?
            break;

        case State::Stopping :
            /* std::cout << "Ignored stopping event. Already STOPPING" << std::endl; */
            break;

        case State::LastStep :
            /* std::cout << "Ignored stopping event. Already LASTSTEP" << std::endl; */
            break;

        }
        break;
    }
    case Event::Empty :

        switch (_current_state)
        {
        case State::Idle :
            break;
        default :
            break;
        }

        break;

    }

    /* burn EVENT */
    _previous_event = _current_event;
    _current_event = Event::Empty;


    return true;
}

void Walker::log(std::string name, XBot::MatLogger::Ptr logger)
{
    logger->add(name + "_step_counter", _step_counter);
    logger->add(name + "_cycle_counter", _cycle_counter);
    logger->add(name + "_steep_q", _steep_q);
    logger->add(name + "_t_impact", _t_impact);
    logger->add(name + "_theta", _theta);
    logger->add(name + "_t_impact", _t_impact);
    logger->add(name + "_time", _time);
    logger->add(name + "_new_event_time", _new_event_time);
    logger->add(name + "_started_flag", _started);
    logger->add(name + "_t_start_walk", _t_start_walk);
    logger->add(name + "_q", _q);
    logger->add(name + "_q_min", _q_min);
    logger->add(name + "_q_max", _q_max);
    logger->add(name + "_q_max_previous", _q_max_previous);
    logger->add(name + "_q_fake", _q_fake);
    logger->add(name + "_dt", _dt);
    logger->add(name + "_current_swing_leg", _current_swing_leg);
    logger->add(name + "_delay_start", _delay_start);
    logger->add(name + "_terrain_height", _terrain_height);
    logger->add(name + "_com_pos_start", _com_pos_start);
    logger->add(name + "_com_pos_goal", _com_pos_goal);
    logger->add(name + "_l_foot_pos_start", _foot_pos_start[0].translation());
    logger->add(name + "_l_foot_pos_goal", _foot_pos_goal[0].translation());
    logger->add(name + "_r_foot_pos_start", _foot_pos_start[1].translation());
    logger->add(name + "_r_foot_pos_goal", _foot_pos_goal[1].translation());
    logger->add(name + "_waist_pos_start", _waist_pos_start.translation());
    logger->add(name + "_waist_pos_goal", _waist_pos_goal.translation());
    logger->add(name + "_step_t_start", _step_t_start);
    logger->add(name + "_step_t_end", _step_t_end);
    logger->add(name + "_step_duration", _step_duration);
    logger->add(name + "_step_clearance", _step_clearance);
    logger->add(name + "_middle_zmp", _zmp_middle);
    logger->add(name + "_zmp_val_current", _zmp_val_current);
    logger->add(name + "_zmp_val_next", _zmp_val_next);
    logger->add(name + "_distance_ankle_com", _distance_ankle_com);
    logger->add(name + "_height_com", _height_com);

    _step.log("step", logger);
    _param->log("param", logger);
    _engine->log("engine", logger);

}
