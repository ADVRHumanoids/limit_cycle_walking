#include <step_machine/step_machine.h>
#include <stdexcept>

StepMachine::StepMachine(double dt, Param * par) :
    _current_state(State::Idle),
    _current_event(Event::Empty), /* TODO */
    _step_counter(0),
    _cycle_counter(0),
    _steep_q(0),
    _t_impact(0),
    _theta(0),
    _param(par),
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
    _middle_zmp(0)
{
}

bool StepMachine::initialize(const mdof::RobotState *state)
{
    _terrain_height = state->getFeet()[_current_swing_leg].translation()(2);
    _current_swing_leg = _param->getFirstSide();

    _middle_zmp = (state->getFeet()[0].translation()(1) + state->getFeet()[1].translation()(1)) /2;

//    _walker->homing(time, state, ref);



    return true;
}

bool StepMachine::homing(double time,
                    const mdof::RobotState * state,
                    mdof::RobotState * ref)
{
    /* com homing  */
    Eigen::Vector3d com = state->getCom();


    /* center the com w.r.t. the feet */
    com(0) = state->getFeet()[_current_swing_leg].translation()(0) + _param->getLeanForward();
    com(1) = (state->getFeet()[0].translation()(1) + (state->getFeet()[1].translation()(1)))/2;
    com(2) = _param->getInitialLowering();


    std::array<Eigen::Affine3d, 2> feet = {state->getFeet()[0], state->getFeet()[1]};


    Eigen::Quaterniond _orientation_goal;
    _orientation_goal.setIdentity();

    feet[0].linear() = _orientation_goal.normalized().toRotationMatrix();
    feet[1].linear() = _orientation_goal.normalized().toRotationMatrix();

    /* set com and feet in ref */
    ref->setCom(com);
    ref->setFeet(feet);

}

bool StepMachine::start()
{
    _previous_event = _current_event;
    _current_event = Event::Start;

    return true;
}

bool StepMachine::stop()
{
    _previous_event = _current_event;
    _current_event = Event::Stop;

    return true;
}

bool StepMachine::setQMax(std::vector<double> q_max)
{
    /* sanity check? */
    for (auto i : q_max)
    {
        _q_buffer.push_back(i);
    }

    return true;
}

bool StepMachine::setTheta(std::vector<double> theta)
{
    /* sanity check? */
    for (auto i : theta)
    {
        _theta_buffer.push_back(i);
    }

    return true;
}

bool StepMachine::updateStep()
{
    if (_q_buffer.empty())
    {
        _q_max = _param->getMaxInclination();
    }
    else
    {
        _q_max = _q_buffer.front();
        _q_buffer.pop_front();
    }

    return true;
}

bool StepMachine::update(double time,
                         const mdof::RobotState * state,
                         mdof::RobotState * ref)
{

    /* update given the robot state */


    computeQ(_current_swing_leg, _theta, state->getCom(), _com_pos_start, state->getAnkleCom());

    computeQFake(time, _q, _q_min, _q_max, _steep_q, _started, _t_start_walk, _q_fake);

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
    core(time);

    /*compute new q_steep for qFake */
    _steep_q = _q_max - _q_min;


    double zmp_val_current = state->getFeet()[_current_swing_leg].translation()(1);
    double zmp_val_next  = state->getFeet()[1 - _current_swing_leg].translation()(1);

    /* t_impact gets updated every time an impact occurs */
    double _step_t_start = _t_impact;

    if (time < _t_start_walk)
    {
        /* never step */
        _step_t_start = time + 1;
        /* if walking is not started, zmp stays in the middle */
        zmp_val_current = _middle_zmp;
        zmp_val_next = _middle_zmp;
    }

    if (time > _t_start_walk && time < _t_start_walk + _delay_start)
    {
        /* delay -first- step to let the com swing laterally */
        _step_t_start = _t_start_walk + _delay_start;
        /* if walking is started, zmp current is still in the middle, but next zmp is the first step */
        zmp_val_current = _middle_zmp;
        /* TODO could be wrong? */
        zmp_val_next = state->getFeet()[_current_swing_leg].translation()(1);
    }




    Eigen::Rotation2Dd rot2(_theta);
    double height_robot = state->getAnkleCom()[1 - _current_swing_leg].translation().z();

    /* FILL STEP STATE */
    mdof::StepState * step_state = nullptr;
    step_state->q = _q;
    step_state->q_min = _q_min;
    step_state->q_min = _q_max;
    step_state->height_robot = height_robot;
    step_state->step_duration = _step_duration;
    step_state->step_clearance = _step_clearance;
    step_state->zmp_val_current = zmp_val_current;
    step_state->zmp_val_next = zmp_val_next;


    Eigen::Vector3d delta_com;
    Eigen::Affine3d swing_foot_pos_goal;
    _walker->compute(time, step_state, delta_com, swing_foot_pos_goal);



    /* compute com trajectory and rotate if needed*/
    Eigen::Vector3d com_ref;
    com_ref = _com_pos_start + delta_com;

    com_ref.head(2) = rot2.toRotationMatrix() * com_ref.head(2);

    /*compute feet trajectories */
    std::array<Eigen::Affine3d, 2> feet_ref;

    /* set foot_goal and t_goal */
    /* THIS IS NECESSARY ONLY WHEN NEW STEP IS ISSUED */
    std::array< Eigen::Affine3d, 2 > feet_pos_goal = _foot_pos_start;
    feet_pos_goal[_current_swing_leg] = swing_foot_pos_goal;

   /* which is also t_impact */
    _step_t_end = _step_t_start + _step_duration;

    feet_ref[1 - _current_swing_leg] = _foot_pos_start[1 - _current_swing_leg];



    /* here the feet trajectory should be a function of q, not t */
    feet_ref[_current_swing_leg] = mdof::computeTrajectory(_foot_pos_start[_current_swing_leg],
                                                           feet_pos_goal[_current_swing_leg],
                                                           _step_clearance,
                                                           _step_t_start,
                                                           _step_t_end,
                                                           time);

    /* compute waist trajectory */
    Eigen::Affine3d waist_ref = _waist_pos_start;
    waist_ref.linear() = (Eigen::AngleAxisd(_theta, Eigen::Vector3d::UnitZ())).toRotationMatrix();


    /* set RobotState ref*/
    ref->setCom(com_ref);
    ref->setFeet(feet_ref);
    ref->setWaist(waist_ref);

    /*TODO sanity check? */
    return true;

}

StepMachine::Param *StepMachine::getDefaultParam()
{
    Param * par = nullptr;
    *par = Param();
    return par;
}

bool StepMachine::impactDetector(double time,
                                 double q, /* fake or not? */
                                 double q_min,
                                 double q_max,
                                 double swing_leg_heigth,
                                 double terrain_heigth)
{
    //     std::cout << "_q1_max: " << _q1_max << std::endl;
    //     std::cout << "sense_q1: " << sense_q1() << std::endl;
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


bool StepMachine::landingHandler(double time,
                                 const mdof::RobotState * state)
{


    /* terrain_height
     * current_swing_leg
     * q_fake
     * q
     * q_min
     * q_max
     * com_start
     * _q_buffer
     */

    if (impactDetector(time, _q, _q_min, _q_max, state->getFeet()[_current_swing_leg].translation()(2), _terrain_height))
    {
        _previous_event = _current_event;
        _current_event = Event::Impact;

        // ---------------------------
        _q_min = _q; /* HACK ALERT */
        // ---------------------------

        /* reset q_fake */
        _q_fake = 0;

        std::string side;
        (_current_swing_leg) ? side = "LEFT" : side = "RIGHT";
        std::cout << "Impact! Current side: " << side << std::endl;

        _com_pos_start = state->getCom();

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

bool StepMachine::computeQFake(double time,
                        double q,
                        double q_min,
                        double q_max,
                        double steep_q,
                        bool started,
                        double start_walk,
                        double& q_fake)
{

    if (started == 1 && time >= +start_walk)
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

double StepMachine::computeQ(bool current_swing_leg,
                             double theta,
                             Eigen::Vector3d world_T_com,
                             Eigen::Vector3d world_T_com_start,
                             std::array<Eigen::Affine3d, 2> ankle_T_com)
{
    /* takes:
     * ankle_T_com
     * world_T_com
     * theta
     * offset q
     * current_stance_leg
     */

    /* TODO remember the stuff about offset ! */

    Eigen::Vector3d dist_com;
    double q;
    double offset_q;
    /* 2D rot matrix, theta in radian */
    Eigen::Rotation2Dd rot2(theta);


    /* distance between current com and starting com (updated at each step)*/
    dist_com = world_T_com - world_T_com_start;

    /* rotate back com */
    dist_com.head(2) = rot2.toRotationMatrix() * dist_com.head(2);

    /* depending on the length of the step, remove offset */
    offset_q = 0;

    /* compute q, inclination angle of the robot */
    q = ( dist_com(0) / fabs(ankle_T_com[1 - current_swing_leg].translation()(2)) ) - offset_q;

    return q;
}


//bool StepMachine::resetter(const mdof::RobotState * state,
//                           mdof::RobotState * ref)
//{
////     generate_starting_zmp();

//    Eigen::Vector3d new_com(state->getCom());
//    new_com(0) = state->getCom()(0) - state->getAnkleCom()[_current_swing_leg].coeff(2) * tan(_initial_param.get_max_inclination());
//    ref->setCom(new_com);
//}

bool StepMachine::core(double time)
{
    /* q_max
     * q_min
     * started
     * step_counter
     * state
     * steep_q
     */

    /* uses q_fake */
    std::cout << "Entering step machine with event: " << _current_event << " during state: " << _current_state << std::endl;

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
            updateStep();

            break;

        case State::Starting :
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::Walking;

            /* starting with the half step */
            updateStep();

            break;

        case State::Stopping :
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::LastStep;

            /* set q_max to zero (is this enough to make it half step?)*/
            _q_max = 0;
            break;

        case State::LastStep :
            _step_counter++;

            /* 'started' flag set to False */
            _started = 0;

            _q_min = _q_fake;

            _previous_state = _current_state;
            _current_state = State::Idle;
            _steep_q = 0;
            resetter();
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
            _q_max = _param->getMaxInclination();
            _step_counter++;
            _cycle_counter++;

            /* 'started' flag set to True */
            _started = 1;

            /* this is neeeded to allow the robot to swing before stepping. 'start_time' is the time when the stepping begins, not when the overall walking begin */
            _t_start_walk = time + _delay_start;

            /* this plan a step at time 'time + delay' so to allow the robot to swing before stepping */
            updateStep();
            break;

        default : /*std::cout << "Ignored starting event. Already WALKING" << std::endl; */
            break;
        }
        break;
    }
    case Event::Stop :
    {
        //             _started = 0;
        switch (_current_state)
        {
        case State::Idle :
            /* std::cout << "Ignored stopping event. Already in IDLE" << std::endl; */
            break;

        case State::Walking :
        case State::Starting :
            //                    _end_walk = time; /*probably not used*/
            _previous_state = _current_state;
            _current_state = State::Stopping; //replan as soon as I get the message?
            updateStep();
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
            updateStep();
            //                    computeStep(/* time */);
            break;
        default :
            break;
        }

        break;

    }
}
