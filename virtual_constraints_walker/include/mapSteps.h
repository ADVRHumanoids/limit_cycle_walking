#ifndef MapSteps_H
#define MapSteps_H

#include <virtualConstraintsNode.h>

class mapSteps
{
public:
    mapSteps(int maxNumSteps, double startTimeWalk) {};
    EventListener() {};
    changeState() {};
    
    const State getCurrentState() const {return _currentState;};
    
protected:
    
    void generateMap() {};
    enum class State { init, idle, startWalk, walking, endWalk };
    State _currentState;
    
    int _maxNumSteps;
    double _startTimeWalk;
    
    
};




#endif