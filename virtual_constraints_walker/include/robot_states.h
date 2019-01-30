#ifndef RobotStates_H
#define RobotStates_H

#include <iostream>

class robot_states
{
public:
    
    enum class Event { impact, start, stop };
    enum class State { init, idle, startWalk, walking, endWalk };
    
    
    friend std::ostream& operator<<(std::ostream& os, Event s)
    {
        switch (s)
        {
            case Event::impact :  return os << "impact";
            case Event::start :  return os << "start";
            case Event::stop :  return os << "stop";
            default : return os << "wrong event";
        }
    };
    
    
    friend std::ostream& operator<<(std::ostream& os, State s)
    {
        switch (s)
        {
            case State::idle :  return os << "idle";
            case State::init :  return os << "init";
            case State::startWalk :  return os << "startWalk";
            case State::endWalk :  return os << "endWalk";
            case State::walking : return os << "walking";
            default : return os << "wrong state";
        }
    };
};

#endif