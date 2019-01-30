#ifndef MapSteps_H
#define MapSteps_H

#include <robot_states.h>
#include <iostream>

class mapSteps : public robot_states
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
    
    
    
    mapSteps(int maxNumSteps, double startTimeWalk);
    
    void ExternalEvent(Event eType);
    void internalEvent(State newState) {};
    
    const State getCurrentState() const {return _currentState;};
    


    
    
private:
    
    void transit(Event _triggeredEvent);

    
    State _currentState;
    State _newState;
    
    int _maxNumSteps;
    double _startTimeWalk;
    int _transitionFlag;
    int _stepCounter;
    
    bool _impactFlag, _startFlag, _stopFlag;
        
};




#endif