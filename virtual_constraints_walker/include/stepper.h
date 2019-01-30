#ifndef Stepper_H
#define Stepper_H

#include <virtualConstraintsNode.h>

class stepper : public robot_states, public virtualConstraintsNode, public mapSteps
{
public:
    
    stepper() {};
    Eigen::Vector3d getStep() {};

private:  
    
    State checkState() {};
    void invokeAction() {};
//     void checkSide() {};
    Eigen::Vector3d calcStep() {};
    
    void ST_init() {};
    Eigen::Vector3d ST_idle() {};
    Eigen::Vector3d ST_startWalk() {};
    Eigen::Vector3d ST_walking() {};
    Eigen::Vector3d ST_endWalk() {};

};


#endif