#include <step_machine/step_machine.h>
#include <stdexcept>

StepMachine::StepMachine(mdof::RobotState * state) :
    _current_state(State::Idle),
    _current_event(Event::Empty),
    _step(), /* TODO */
    _state(state),
    _time(0)
{

}

bool StepMachine::setEvent(Event event)
{
    _previous_event = _current_event;
    _current_event = event;

    return true;
}

bool StepMachine::update(const mdof::RobotState * state,
                         mdof::RobotState * ref)
{

    impactHandler(mdof::StepState& step);

    qHandler();
    core();



}

bool StepMachine::impactDetector()
{
//     std::cout << "_q1_max: " << _q1_max << std::endl;
//     std::cout << "sense_q1: " << sense_q1() << std::endl;

    if (_q1_min <= _q1_max)
    {
        _cond_q = (sense_q1() >= _q1_max);
    }
    else if (_q1_min > _q1_max)
    {
        _cond_q = (sense_q1() <= _q1_max);
    }

    _cond_step = fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-3;

    if (_cond_step && _cond_q)
    {
        _time_fake_impact = _internal_time;
        return true;
    }

    return false;
}


bool StepMachine::landingHandler()
{
    if (impactDetector())
    {
        _previous_event = _current_event;
        _current_event = Event::Impact;
//                 _current_pose_ROS.sense();

        Walker::Side last_side = _current_side;
//                 std::cout << "Last side: " << last_side << std::endl;
        _initial_pose = _current_pose_ROS;
    // ----------------------------------
        _q1_min = -sense_q1(); /*HACK*/
    // -------------------------------
        _current_side = robot_interface::Side::Double;

        std::cout << "Impact! Current side: " << _current_side << std::endl;

//                 _current_pose_ROS.get_sole(_current_side);
        // ----------------------------------------------------
//                 _poly_com.set_com_initial_position(_initial_pose.get_com());

        _current_world_to_com = _current_pose_ROS.get_world_to_com();
        // ----------------------------------------------------
//

        if (last_side == robot_interface::Side::Left)
        {
            _current_side = robot_interface::Side::Right;
            _other_side = robot_interface::Side::Left;
        }
        else if (last_side == robot_interface::Side::Right)
        {
            _current_side = robot_interface::Side::Left;
            _other_side = robot_interface::Side::Right;
        }
        else ROS_INFO("wrong side");

//                 _q1_min = sense_q1(); /*HACK*/
        std::cout << "State changed. Current side: " << _current_side << std::endl;

        return 1;
    }
    else
    {
        return 0; // if impact detector does not detect an impact
    }
}

double StepMachine::qFake()
{

    if (_started == 1 && _time >= +_start_walk)
    {
        bool cond_q = false;

        if (_q_min <= _q_max)
        {
            cond_q = (_q >= _q_max);
        }
        else if (_q_min > _q_max)
        {
            cond_q = (_q <= _q_max);
        }
        if (cond_q)
        {
            /* do nothing */
        }
        else
        {
            _q_temp = _q_temp + _steep_q*(_dt); // basically q = a*t
        }
    }

}

bool StepMachine::core()
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
                    _walker->walk(/* ... */);
                    break;

                case State::Starting :
                    _step_counter++;
                    _previous_state = _current_state;
                    _current_state = State::Walking;
                    
                    _walker->start(/* ... */);
                    break;

                case State::Stopping :
                    _step_counter++;
                    _previous_state = _current_state;
                    _current_state = State::LastStep;
                    _q1_max = 0;
                    
                    _walker->stop();
                    break;

                case State::LastStep :
                    _step_counter++;
                    _started = 0;
                    _q1_start = _q1_temp;
                    _previous_state = _current_state;
                    _current_state = State::Idle;
                    
                    _walker->lastStep();
                    _steep_coeff = 0;
                    resetter();
                    break;
            }
            break;

        case Event::START :
        {
            switch (_current_state)
            {

                case State::Idle :
                    _previous_state = _current_state;
                    _current_state = State::Starting;
                    _q1_max = _initial_param.get_max_inclination();
                    _step_counter++;
                    _cycle_counter++;
                    _started = 1;
                    _start_walk = _internal_time + _delay_start;
                    _steep_coeff = (_q1_max - _q1_min)/_step_duration;
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
                    _end_walk = time;
                    _previous_state = _current_state;
                    _current_state = State::Stopping;
//                     planner(time); //replan as soon as I get the message?
                    break;

                case State::Stopping :
                    /* std::cout << "Ignored stopping event. Already STOPPING" << std::endl; */
                    break;
            }
            break;
        }
        case Event::Empty :

            switch (_current_state)
            {
                case State::Idle :
                    planner(time);
                    break;
                default :
                    break;
            }

            break;

        default :
            throw std::runtime_error(std::string("Event not recognized"));
    }
}
