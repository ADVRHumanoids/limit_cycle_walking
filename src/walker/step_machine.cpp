//#include <walker/step_machine.h>

//State stepMachine::update()
//{
//    //    std::cout << "Entering step machine with event: '" << _current_event << "' during state: '" << _current_state << "'" << std::endl;

//    if (_previous_event != _current_event)
//    {
//        _new_event_time = _time;
//    }

//    switch (_current_event)
//    {
//    case Event::SagReached :
//        switch (_current_state)
//        {
//        case State::Idle :
//            throw std::runtime_error(std::string("Something is wrong. Impact during IDLE"));

//        case State::Walking :
//            if (_q_buffer.empty())
//            {
//                std::vector<double> new_q;
//                new_q.push_back(_param->getMaxInclination());
//                setQMax(new_q);
//            }
//            _step_counter++;
//            break;

//        case State::Starting :
//            if (_q_buffer.empty())
//            {
//                std::vector<double> new_q;
//                new_q.push_back(_param->getMaxInclination());
//                setQMax(new_q);
//            }
//            _step_counter++;
//            _previous_state = _current_state;
//            _current_state = State::Walking;
//            break;

//        case State::Stopping :
//            _step_counter++;
//            _previous_state = _current_state;
//            _current_state = State::LastStep;

//            if (_q_buffer.empty())
//            {
//                std::vector<double> new_q;
//                new_q.push_back(0);
//                setQMax(new_q);
//            }
//            break;

//        case State::LastStep :
//            _step_counter++;

//            _previous_state = _current_state;
//            _current_state = State::Idle;
//            break;
//        }
//        break;
//    case Event::LatReached :
//    {
//        switch (_current_state)
//        {
//        default:
//            break;
//        }
//        break;
//    }
//    case Event::Start :
//    {
//        switch (_current_state)
//        {

//        case State::Idle :
//            if (_q_buffer.empty())
//            {
//                std::vector<double> new_q;
//                new_q.push_back(_param->getMaxInclination());
//                setQMax(new_q);
//            }
//            _previous_state = _current_state;
//            _current_state = State::Starting;

//            _cycle_counter++;
//            break;

//        default : /*std::cout << "Ignored starting event. Already WALKING" << std::endl; */
//            break;
//        }
//        break;
//    }
//    case Event::Stop :
//    {
//        switch (_current_state)
//        {
//        case State::Idle :
//            /* std::cout << "Ignored stopping event. Already in IDLE" << std::endl; */
//            break;

//        case State::Walking :
//        case State::Starting :
//            _previous_state = _current_state;
//            _current_state = State::Stopping; // TODO replan as soon as I get the message?
//            break;

//        case State::Stopping :
//            /* std::cout << "Ignored stopping event. Already STOPPING" << std::endl; */

//            break;

//        case State::LastStep :
//            /* std::cout << "Ignored stopping event. Already LASTSTEP" << std::endl; */
//            break;

//        }
//        break;
//    }
//    case Event::Empty :

//        switch (_current_state)
//        {
//        case State::Idle :
//            break;
//        default :
//            break;
//        }

//        break;

//    }

//    /* burn EVENT */0
//    _previous_event = _current_event;
//    _current_event = Event::Empty;


//    return true;
//}
