#include <mapSteps.h>


mapSteps::mapSteps(int maxNumSteps, double startTimeWalk) 
{
    _maxNumSteps = maxNumSteps;
    _startTimeWalk = startTimeWalk;
    
    _currentState = State::init;
};


mapSteps::State mapSteps::transitionMap()
{

    
};
mapSteps::State mapSteps::transit(mapSteps::State _currentState, Event _triggeredEvent) 
{
    if (_triggeredEvent == Event::impact)
    {
        if (_currentState == State::init)
        {
            _currentState = State::idle;
        }
        
    }
        
};

mapSteps::EventListener() {};
mapSteps::changeState() {};


    
