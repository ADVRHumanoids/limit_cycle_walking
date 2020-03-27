#include <walker/walker.h>
#include <stdexcept>

Walker::Walker(double dt, std::shared_ptr<Param> par) :
    _current_state(State::Idle),
    _current_event(Event::Empty),
    _step_counter(0),
    _cycle_counter(0),
    _steep_q_sag(0),
    _steep_q_lat(0),
    _t_impact(0),
    _theta(0),
    _time(0),
    _new_event_time(0),
    _t_start_walk(0),
    _q(0),
    _q_sag(0),
    _q_sag_min(0),
    _q_sag_max(0),
    _q_lat(0),
    _q_lat_min(0),
    _q_lat_max(0),
    _dt(dt),
    _current_swing_leg(0),
    _delay_start(0),
    _terrain_height(0),
    _step_t_start(0),
    _step_t_end(0),
    _step_clearance(0),
    _zmp_middle(0),
    _update_step(false),
    _param(par)
{

    _durations.resize(1);
    _com_pos_start.setZero();
    _com_pos_goal.setZero();

    _delta_com.setZero();
    _delta_com_rot.setZero();

    _foot_pos_start[0].matrix().setZero();
    _foot_pos_start[1].matrix().setZero();

    _foot_pos_goal[0].matrix().setZero();
    _foot_pos_goal[1].matrix().setZero();

    _waist_pos_start.matrix().setZero();
    _waist_pos_goal.matrix().setZero();
}

bool Walker::init(const mdof::RobotState &state)
{
    std::cout << "Initialization of Walker... " << std::endl;
    _current_stance = Stance::Double;
    _terrain_height = state.world_T_foot[_current_swing_leg].translation()(2);
    _current_swing_leg = _param->getFirstSideStep();
    _zmp_middle = (state.world_T_foot[0].translation()(1) + state.world_T_foot[1].translation()(1)) /2;
    _com_pos_start = state.world_T_com;
    _com_pos_goal = state.world_T_com;
    /* initialize duration for zmp window */
    _durations <<_param->getStepDuration();
    /* TODO _q depends on current _q_max, so I need to update _q_max before _q */
    _q_sag_max_previous = 0;
    _q_sag_max = _param->getMaxInclination();
    _q_sag = computeQSag(_current_swing_leg, _theta, state.world_T_com, _com_pos_start, state.ankle_T_com);
    /* TODO this should be _q as starting, but since I am using a small hack for starting, I'm beginning from 0
     * so that after the first part (up to _start + _delay) is at the right value
     */
    _q_sag_min = _q_sag;  /*TODO here make changes!!!! */

    _q_lat = _q_sag;
    _q_lat_min = _q_sag_min;
    _q_lat_max = _q_sag_max;

    _foot_pos_start = state.world_T_foot;
    _foot_pos_goal = _foot_pos_start;


    /* durations */
    _ss_duration = _param->getStepDuration();
    _ds_duration = 0; //_param->getDoubleStanceDuration();

    /* zmp vals */
    _zmp_val_initial_left = state.world_T_foot[0].translation()[1];
    _zmp_val_initial_right = state.world_T_foot[1].translation()[1];

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

    /* set zmp */
    _zmp_vals.push_back((Eigen::VectorXd(1,1) << _zmp_middle).finished());


    updateStep();

    _engine->initialize(_step);

    std::cout << "Ready." << std::endl;
    /* TODO sanity check? */
    return true;
}

bool Walker::homing(const mdof::RobotState &state,
                    mdof::RobotState &ref)
{
    /* intialize ref with state*/
    ref = state;

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

bool Walker::setPhi(std::vector<double> phi)
{
    /* sanity check? */
    for (auto i : phi)
    {
        _phi_buffer.push_back(i);
    }
    /* TODO sanity check? */
    return true;
}

bool Walker::updateQMax(double time)
{
    /* update q_max */
    if (_q_buffer.empty())
    {
        std::cout << "empty buffer, _q_sag_max is not updated: " << _q_sag_max << std::endl;
        return false;
    }
    else
    {
        _q_sag_max = _q_buffer.front();
        _q_buffer.pop_front();
        return true;
    }

}

bool Walker::updateTheta(double time)
{
    if (_theta_buffer.empty())
    {
        _theta = 0;
    }
    else
    {
        _theta = _theta_buffer.front();
        _theta_buffer.pop_front();
    }
    return true;
}

bool Walker::updatePhi(double time)
{
    if (_phi_buffer.empty())
    {
        _phi = 0;
    }
    else
    {
        _phi = _phi_buffer.front();
        _phi_buffer.pop_front();
    }
    return true;
}

bool Walker::updateStep()
{
    _step.q_sag = _q;
    _step.q_lat = _q_lat;
    _step.q_sag_min = _q_sag_min;
    _step.q_sag_max = _q_sag_max;
    _step.q_lat_min = _q_lat_min;
    _step.q_lat_max = _q_lat_max;
    _step.height_com = _height_com;
    _step.distance_ankle_com = _distance_ankle_com;
    _step.step_clearance = _step_clearance;
    _step.zmp_vals = _zmp_vals;
    _step.durations = _durations;
    _step.zmp_middle = _zmp_middle;

    return true;
}

bool Walker::update(double time,
                    const mdof::RobotState &state,
                    mdof::RobotState &ref)
{
    /* update _q_sag */
    Eigen::Vector3d com_lat(0, _delta_com[1], 0);
    Eigen::Vector3d com = state.world_T_com - com_lat;

    _q_sag = computeQSag(_current_swing_leg, _theta, com, _com_pos_start, state.ankle_T_com);

    sagHandler(time, state);

    /* if impact, go to double stance */
    if (_current_event == Event::SagReached)
    {
        _current_stance = Stance::Double;
        std::cout << "State changed. Current stance: DOUBLE" << std::endl;
        /* change _current_swing_leg */
        _current_swing_leg = 1 - _current_swing_leg;
    }

    /* if in idle and receive a q_max, start TODO with that q_max!*/
    if (_current_state == State::Idle)
    {
        if (!_q_buffer.empty())
        {
          _current_event = Event::Start;
        }
    }

    /* if qDetector lat triggers, get from buffer if it's not empty and issue step */
    if (qDetector(_q_lat, _q_lat_min, _q_lat_max))
    {
        _current_event = Event::LatReached;
        _q_lat = 0; /* RESET Q_LAT */

        if (!_q_buffer.empty())
        {
            updateQMax(time);
            updateTheta(time);
            _update_step = 1;
            _execute_step = 1;
        }
        else
        {
            _current_event = Event::Stop;
        }

        _current_stance = Stance::Single;
        std::string side;
        (_current_swing_leg) ? side = "LEFT" : side = "RIGHT";
        std::cout << "State changed. Current stance: " << side << std::endl;
    }

    step_machine(time);

    /* ------------------------------- update durations ------------------------------------------*/
    if (_current_state != State::Idle)
    {
        /* if SS */
        if (_current_stance == Stance::Single)
        {
            _durations.resize(2);
            _durations << _ss_duration, _ds_duration;
            if (_current_state == State::LastStep)
            {
                _durations.resize(3);
                _durations << _ss_duration, _ds_duration, _ss_duration;
            }
        }
        /* IF DS */
        else
        {
            if (_current_state == State::Starting)
            {
                _durations.resize(3);
                /* regardless of ds duration, I need some time to start walking, hence 2 * ss_duration */
                _durations << 2 *_ss_duration, _ss_duration, _ds_duration;
            }
            else if (_current_state == State::LastStep)
            {
                _durations.resize(3);
                _durations << _ds_duration, _ss_duration, _ss_duration;
            }
            else
            {
                _durations.resize(2);
                _durations << _ds_duration, _ss_duration;
            }
        }
    }
    else
    {
        _durations.resize(1);
        _durations << _param->getStepDuration();
    }
    /* ---------------------------------- update q ----------------------------------------------*/
    if (_current_state != State::Idle)
    {
        if (_current_stance == Stance::Single)
        {
            _steep_q_sag = (_q_sag_max - _q_sag_min) / _durations[0];
            updateQ(time, _q_sag, _q_sag_min, _q_sag_max, _steep_q_sag, _q);
            _q_lat = _q_sag;
            _q_lat_min = _q_sag_min;
            _q_lat_max = _q_sag_max;
        }
        else
        {
            if (_current_state == State::Starting || _current_state == State::Walking || _current_state == State::LastStep || _current_state == State::Stopping)
            {
                _steep_q_lat = (_q_lat_max - _q_lat_min)/_durations[0];
                _q_lat = _q_lat + _steep_q_lat * _dt;
            }
        }
    }
    /* TODO are these constant? */
    _distance_ankle_com = state.ankle_T_com[1 - _current_swing_leg].translation().z();
    _height_com = fabs(state.world_T_com[2] - state.world_T_foot[_current_swing_leg].translation()[2]);


    /* --------------------------------- update zmp -----------------------------------------------*/
    if (_current_state != State::Idle)
    {
        double zmp_current;
        double zmp_next;

        if (_current_swing_leg == 0)
        {
            zmp_current = _zmp_val_initial_right;
            zmp_next = _zmp_val_initial_left;
        }
        else
        {
            zmp_current = _zmp_val_initial_left;
            zmp_next = _zmp_val_initial_right;
        }

//        zmp_current = state.world_T_foot[1 - _current_swing_leg].translation()[1];
//        zmp_next = state.world_T_foot[_current_swing_leg].translation()[1];

        if (_current_stance == Stance::Single)
        {
            _zmp_vals.clear();
            _zmp_vals.push_back((Eigen::MatrixXd(1,1) << zmp_current).finished());
            _zmp_vals.push_back((Eigen::MatrixXd(1,2) << zmp_current, zmp_next).finished());
        }
        else
        {
            _zmp_vals.clear();
            if (_current_state == State::Starting)
            {
                _zmp_vals.push_back((Eigen::MatrixXd(1,3) << _zmp_middle, _zmp_middle, zmp_current).finished());
                _zmp_vals.push_back((Eigen::MatrixXd(1,1) << zmp_current).finished());
                _zmp_vals.push_back((Eigen::MatrixXd(1,2) << zmp_current, zmp_next).finished());
            }
            else
            {
                _zmp_vals.push_back((Eigen::MatrixXd(1,2) << zmp_next, zmp_current).finished());
                _zmp_vals.push_back((Eigen::MatrixXd(1,1) << zmp_current).finished());
            }
        }


        if (_current_state == State::LastStep)
        {
            /* for the ending motion of the last step */
            /* TODO put here for zeroing zmp afterlast step */
            if (_current_stance == Stance::Single)
            {
                _zmp_vals.clear();
                _zmp_vals.push_back((Eigen::MatrixXd(1,1) << zmp_current).finished());
                _zmp_vals.push_back((Eigen::MatrixXd(1,1) << _zmp_middle).finished());
                _zmp_vals.push_back((Eigen::MatrixXd(1,1) << _zmp_middle).finished());
            }
            else
            {
                _zmp_vals.clear();
                _zmp_vals.push_back((Eigen::MatrixXd(1,2) << zmp_next, zmp_current).finished());
                _zmp_vals.push_back((Eigen::MatrixXd(1,1) <<zmp_current).finished());
                _zmp_vals.push_back((Eigen::MatrixXd(1,1) << _zmp_middle).finished());
            }
        }
    }
    else
    {
        _zmp_vals.clear();
        _zmp_vals.push_back((Eigen::MatrixXd(1,1) << _zmp_middle).finished());
    }

    /* -- until here everything is updated -- */
    updateStep();

    /* compute com and foot displacement */
    _engine->computeCom(time, _step, _delta_com);

    /*------------------------------- update step --------------------------------------------*/
    if (_update_step)
    {
        Eigen::Rotation2Dd rot2(_theta);

        Eigen::Vector3d delta_com_tot;
        Eigen::Vector3d delta_foot_tot;

        _engine->computeStep(time, _step, delta_foot_tot, delta_com_tot);

        delta_com_tot.head(2) = rot2.toRotationMatrix() * delta_com_tot.head(2);
        _com_pos_goal = _com_pos_start + delta_com_tot;

        delta_foot_tot.head(2) = rot2.toRotationMatrix() * delta_foot_tot.head(2);
        _foot_pos_goal[_current_swing_leg].translation() = _foot_pos_start[_current_swing_leg].translation() + delta_foot_tot;
        _foot_pos_goal[_current_swing_leg].linear() = (Eigen::AngleAxisd(_phi, Eigen::Vector3d::UnitZ())).toRotationMatrix();

//        _zmp_middle = (_foot_pos_goal[_current_swing_leg].translation()[1] + _foot_pos_goal[1 - _current_swing_leg].translation()[1]) /2;
        /* t_impact gets updated every time an impact occurs */
        /* delay -first- step to let the com swing laterally */
        _update_step = false;
    }
    /*------------------------------ execute step ---------------------------------------------*/
    if (_execute_step)
    {
        _step_t_start = time;
        _step_t_end = _step_t_start + _durations[0];

        /* burn execute_step */
        _execute_step = 0;
    }

    /*------------------------------- computing ---------------------------------------------------*/
    Eigen::Rotation2Dd rot2(_theta);
    /* compute com trajectory and rotate if needed */

    Eigen::Vector2d delta_com_x(_delta_com[0], 0);
    delta_com_x = rot2.toRotationMatrix() * delta_com_x.head(2);


    _delta_com_rot << delta_com_x[0], delta_com_x[1] + _delta_com[1], _delta_com[2];


//    delta_com_rot.head(2) = rot2.toRotationMatrix() * _delta_com.head(2);
//    delta_com_rot[2] = _delta_com[2];



    Eigen::Vector3d com_ref;
    com_ref = _com_pos_start + _delta_com_rot;

    /*compute feet trajectories and rotate if needed */
    std::array<Eigen::Affine3d, 2> feet_ref;
    feet_ref[1 - _current_swing_leg] = _foot_pos_start[1 - _current_swing_leg];

    /* TODO here the feet trajectory should be a function of q, not t */
    feet_ref[_current_swing_leg] = mdof::computeTrajectory(_foot_pos_start[_current_swing_leg],
                                                           _foot_pos_goal[_current_swing_leg],
                                                           _step_clearance,
                                                           _step_t_start,
                                                           _step_t_end,
                                                           time);

    /* rotate waist */
    Eigen::Affine3d waist_ref = _waist_pos_start;
    waist_ref.linear() = (Eigen::AngleAxisd(_phi, Eigen::Vector3d::UnitZ())).toRotationMatrix();

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

bool Walker::qDetector(double q, /* TODO fake or not? */
                       double q_min,
                       double q_max)
{
    bool flag_q(false);

    /* q reaches q_max */
    if (q_min <= q_max)
    {
        flag_q = (q >= q_max);
    }
    else if (q_min > q_max)
    {
        flag_q = (q <= q_max);
    }

    return flag_q;
}

bool Walker::impactDetector(double swing_leg_heigth,
                            double terrain_heigth)
{
    bool flag_step(false);
    /* sole impacts ground (a certain treshold is reached) */
    flag_step = fabs(fabs(swing_leg_heigth) - fabs(terrain_heigth)) <= 1e-3;

    return flag_step;

}

bool Walker::sagHandler(double time,
                            const mdof::RobotState &state)
{
    /* TODO what if in IDLE? THINK ABOUT THIS */
    if (qDetector( _q_sag, _q_sag_min, _q_sag_max) && impactDetector(state.world_T_foot[_current_swing_leg].translation()(2), _terrain_height))
    {

        _t_impact = time;

        _previous_event = _current_event;
        _current_event = Event::SagReached;

        // ---------------------------
        _q_sag_min = - _q_sag; /* HACK ALERT */
        _q_sag_max_previous = _q_sag_max;
        // ---------------------------

        /* reset q_fake */
        _q = 0;
        /* TODO is it ok? */
        _q_lat = - _q_sag;
        _q_lat_min = - _q_sag;

        std::string side;
        (_current_swing_leg) ? side = "LEFT" : side = "RIGHT";
        std::cout << "Impact! during " << side << " foot swing."<< std::endl;

        /* TODO update com start: WITH REAL OR WITH PLANNED? this is basically a way to get rid of the com jump in the last step */
        //        _com_pos_start = state.world_T_com;
        _com_pos_start = _com_pos_goal;

        /* this is probably necessary! */
        _foot_pos_start = state.world_T_foot;
        return 1;
    }
    else
    {
        return 0; // if impact detector does not detect an impact
    }
}

bool Walker::updateQ(double time,
                     double q_sag,
                     double q_sag_min,
                     double q_sag_max,
                     double steep_q,
                     double& q)
{
        bool cond_q(false);

        if (q_sag_min <= q_sag_max)
        {
            cond_q = (q_sag >= q_sag_max);
        }
        else if (_q_sag_min > q_sag_max)
        {
            cond_q = (q_sag <= q_sag_max);
        }
        if (cond_q)
        {
            /* do nothing */
        }
        else
        {
            q = q + steep_q*(_dt); // basically q = a*t
        }
    /* TODO sanity check? */
    return true;
}

double Walker::computeQSag(bool current_swing_leg,
                           double theta,
                           Eigen::Vector3d world_T_com,
                           Eigen::Vector3d world_T_com_start,
                           std::array<Eigen::Affine3d, 2> ankle_T_com)
{
    Eigen::Vector3d dist_com;
    double q;
    double offset_q;
    /* 2D rot matrix, theta in radian */
    Eigen::Rotation2Dd rot2(-theta);

    /* distance between current com and starting com (updated at each step) */
    dist_com = world_T_com - world_T_com_start;

    /* rotate back com */
    dist_com.head(2) = rot2.toRotationMatrix() * dist_com.head(2);

    /* TODO depending on the length of the step, remove offset */
    offset_q = _q_sag_max_previous;

    /* compute q, inclination angle of the robot */
    q = atan( dist_com(0) / fabs(ankle_T_com[1 - current_swing_leg].translation()(2)) ) - offset_q;

    return q;
}

bool Walker::updateZmp(mdof::RobotState state)
{
}

bool Walker::step_machine(double time)
{
    /* q_max
     * q_min
     * started
     * step_counter
     * state
     * steep_q
     */

    //    std::cout << "Entering step machine with event: '" << _current_event << "' during state: '" << _current_state << "'" << std::endl;

    if (_previous_event != _current_event)
    {
        _new_event_time = _time;
    }

    switch (_current_event)
    {
    case Event::SagReached :
        switch (_current_state)
        {
        case State::Idle :
            throw std::runtime_error(std::string("Something is wrong. Impact during IDLE"));

        case State::Walking :
            if (_q_buffer.empty())
            {
                std::vector<double> new_q;
                new_q.push_back(_param->getMaxInclination());
                setQMax(new_q);
            }
            _step_counter++;
            break;

        case State::Starting :
            if (_q_buffer.empty())
            {
                std::vector<double> new_q;
                new_q.push_back(_param->getMaxInclination());
                setQMax(new_q);
            }
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::Walking;
            break;

        case State::Stopping :
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::LastStep;

            _q_sag_max_previous = _q_sag_max;
            if (_q_buffer.empty())
            {
                std::vector<double> new_q;
                new_q.push_back(0);
                setQMax(new_q);
            }
            break;

        case State::LastStep :
            _step_counter++;

            _previous_state = _current_state;
            _current_state = State::Idle;
            _steep_q_sag = 0;
            _q_sag_max = _param->getMaxInclination();
            break;
        }
        break;
    case Event::LatReached :
    {
        switch (_current_state)
        {
        default:
            break;
        }
        break;
    }
    case Event::Start :
    {
        switch (_current_state)
        {

        case State::Idle :
            if (_q_buffer.empty())
            {
                std::vector<double> new_q;
                new_q.push_back(_param->getMaxInclination());
                setQMax(new_q);
            }
            _previous_state = _current_state;
            _current_state = State::Starting;

            _step_counter++;
            _cycle_counter++;

            _t_start_walk = time;
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
    logger->add(name + "_steep_q_sag", _steep_q_sag);
    logger->add(name + "_steep_q_lat", _steep_q_lat);
    logger->add(name + "_t_impact", _t_impact);
    logger->add(name + "_theta", _theta);
    logger->add(name + "_phi", _phi);
    logger->add(name + "_time", _time);
    logger->add(name + "_new_event_time", _new_event_time);
    logger->add(name + "_t_start_walk", _t_start_walk);
    logger->add(name + "_q", _q);
    logger->add(name + "_q_sag", _q_sag);
    logger->add(name + "_q_sag_min", _q_sag_min);
    logger->add(name + "_q_sag_max", _q_sag_max);
    logger->add(name + "_q_sag_max_previous", _q_sag_max_previous);
    logger->add(name + "_q_lat", _q_lat);
    logger->add(name + "_q_lat_min", _q_lat_min);
    logger->add(name + "_q_lat_max", _q_lat_max);
    logger->add(name + "_dt", _dt);
    logger->add(name + "_current_swing_leg", _current_swing_leg);
    logger->add(name + "_delay_start", _delay_start);
    logger->add(name + "_terrain_height", _terrain_height);
    logger->add(name + "_com_pos_start", _com_pos_start);
    logger->add(name + "_com_pos_goal", _com_pos_goal);
    logger->add(name + "_delta_com", _delta_com);
    logger->add(name + "_delta_com_rot", _delta_com_rot);
    logger->add(name + "_l_foot_pos_start", _foot_pos_start[0].translation());
    logger->add(name + "_l_foot_pos_goal", _foot_pos_goal[0].translation());
    logger->add(name + "_r_foot_pos_start", _foot_pos_start[1].translation());
    logger->add(name + "_r_foot_pos_goal", _foot_pos_goal[1].translation());
    logger->add(name + "_waist_pos_start", _waist_pos_start.translation());
    logger->add(name + "_waist_pos_goal", _waist_pos_goal.translation());
    logger->add(name + "_step_t_start", _step_t_start);
    logger->add(name + "_step_t_end", _step_t_end);
    logger->add(name + "_step_clearance", _step_clearance);
    logger->add(name + "_middle_zmp", _zmp_middle);
    logger->add(name + "_distance_ankle_com", _distance_ankle_com);
    logger->add(name + "_height_com", _height_com);

    _step.log("step", logger);
    _param->log("param", logger);
    _engine->log("engine", logger);

}




