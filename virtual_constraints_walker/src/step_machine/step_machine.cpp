#include <step_machine/step_machine.h>
#include <stdexcept>

StepMachine::StepMachine(double dt, Param * par) :
    _current_state(State::Idle),
    _current_event(Event::Empty), /* TODO */
    _step_counter(0),
    _cycle_counter(0),
    _steep_q(0),
    _time_impact(0),
    _theta(0),
    _param(par),
    _time(0),
    _new_event_time(0),
    _started(false),
    _start_walk(0),
    _q(0),
    _q_min(0),
    _q_max(0),
    _q_fake(0),
    _dt(dt),
    _current_swing_leg(0)
{

}

bool StepMachine::initialize(const mdof::RobotState *state)
{
    _terrain_height = state->getFoot()[_current_swing_leg].translation()(2);
    _current_swing_leg = _param->getFirstSide();

//    _walker->homing(time, state, ref);



    return true;
}

bool StepMachine::homing(double time,
                    const mdof::RobotState * state,
                    mdof::RobotState * ref)
{
    /* com homing  */

    Eigen::Vector3d com = state->getCom();

    ref->setComStart(com);

    /* center the CoM w.r.t. the feet */
    com(0) = state->getFoot()[state->getSwingLeg()].translation()(0) + _param->getLeanForward();
    com(1) = (state->getFoot()[0].translation()(1) + (state->getFoot()[1].translation()(1)))/2;
    com(2) = _param->getInitialLowering();

    ref->setComGoal(com);

    std::array<Eigen::Affine3d, 2> feet = {state->getFoot()[0], state->getFoot()[1]};

    ref->setFootStart(feet);

    Eigen::Quaterniond _orientation_goal;
    _orientation_goal.setIdentity();

    feet[0].linear() = _orientation_goal.normalized().toRotationMatrix();
    feet[1].linear() = _orientation_goal.normalized().toRotationMatrix();

    ref->setFootGoal(feet);
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

bool StepMachine::updateStep(mdof::RobotState * ref)
{
    if (_q_buffer.empty())
    {
        ref->setQMax(_param->getMaxInclination());
    }
    else
    {
        ref->setQMax(_q_buffer.front());
        _q_buffer.pop_front();
    }

    return true;
}

bool StepMachine::update(double time,
                         const mdof::RobotState * state,
                         mdof::RobotState * ref)
{
    /* this changes 'ref':
     * q_min
     * com_start
     */
    landingHandler(time, _terrain_height, state, ref);

    qFake(time, state->getQ(), state->getQMin(), state->getQMax(), state->getSteepQ(), started, state->getStartWalkTime(), &q_fake);

    /* this changes 'ref':
     * q_max
     * q_min
     * q_steep
     */
    core(time, q_fake, ref);

    /*compute new q_steep for qFake */
    ref->setSteepQ(ref->getQMax()-ref->getQMin());

    /* this changes 'ref':
     * com_ref
     * foot_ref
     * waist_ref
     */
    _walker->compute(time, state, ref);

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
        _time_impact = time;
        return true;
    }

    return false;
}


bool StepMachine::landingHandler(double time,
                                 double terrain_heigth,
                                 const mdof::RobotState * state,
                                 mdof::RobotState * ref)
{
    bool current_swing_leg = state->getSwingLeg();

    if (impactDetector(time, state->getQ(), state->getQMin(), state->getQMax(), state->getFoot()[current_swing_leg].translation()(2), terrain_heigth))
    {
        _previous_event = _current_event;
        _current_event = Event::Impact;



        //        _initial_pose = _current_pose_ROS; /* probably not needed */
        // ----------------------------------
        ref->setQMin(state->getQ()); /* HACK ALERT */
        // -------------------------------
        //        _current_side = robot_interface::Side::Double;
        std::string side;
        if (current_swing_leg == 1)
        {
            side = "LEFT";
        }
        else
        {
            side = "RIGHT";
        }
        std::cout << "Impact! Current side: " << side << std::endl;

        ref->setComStart(state->getCom());

        current_swing_leg = 1 - current_swing_leg;
        ref->setSwingLeg(current_swing_leg);
        // ----------------------------------------------------
        //

        //        if (last_side == robot_interface::Side::Left)
        //        {
        //            _current_side = robot_interface::Side::Right;
        //            _other_side = robot_interface::Side::Left;
        //        }
        //        else if (last_side == robot_interface::Side::Right)
        //        {
        //            _current_side = robot_interface::Side::Left;
        //            _other_side = robot_interface::Side::Right;
        //        }
        //        else ROS_INFO("wrong side");

        //                 _q1_min = sense_q1(); /*HACK*/

        if (ref->getSwingLeg() == 1)
        {
            side = "LEFT";
        }
        else
        {
            side = "RIGHT";
        }
        std::cout << "State changed. Current side: " << side << std::endl;
        return 1;
    }
    else
    {
        return 0; // if impact detector does not detect an impact
    }
}

bool StepMachine::qFake(double time,
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


//bool StepMachine::resetter(const mdof::RobotState * state,
//                           mdof::RobotState * ref)
//{
////     generate_starting_zmp();

//    Eigen::Vector3d new_com(state->getCom());
//    new_com(0) = state->getCom()(0) - state->getAnkleCom()[_current_swing_leg].coeff(2) * tan(_initial_param.get_max_inclination());
//    ref->setCom(new_com);
//}

bool StepMachine::core(double time,
                       double q_fake,
                       mdof::RobotState * ref)
{
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
            updateStep(ref);

            break;

        case State::Starting :
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::Walking;

            /* starting with the half step */
            updateStep(ref);

            break;

        case State::Stopping :
            _step_counter++;
            _previous_state = _current_state;
            _current_state = State::LastStep;

            /* set q_max to zero (is this enough to make it half step?)*/
            ref->setQMax(0);
            break;

        case State::LastStep :
            _step_counter++;

            /* 'started' flag set to False */
            _started = 0;

            ref->setQMin(q_fake);

            _previous_state = _current_state;
            _current_state = State::Idle;
            ref->setSteepQ(0);
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
            ref->setQMax(_param->getMaxInclination());
            _step_counter++;
            _cycle_counter++;

            /* 'started' flag set to True */
            _started = 1;

            /* this is neeeded to allow the robot to swing before stepping. 'start_time' is the time when the stepping begins, not when the overall walking begin */
            ref->setStartTime(time + _delay_start);

            /* this plan a step at time 'time + delay' so to allow the robot to swing before stepping */
            updateStep(ref);
            planner(time + _delay_start); //_t_before_first_step
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
            updateStep(ref);
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
            updateStep(ref);
            //                    computeStep(/* time */);
            break;
        default :
            break;
        }

        break;

    }
}
