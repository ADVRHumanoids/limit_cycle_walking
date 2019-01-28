#ifndef Stepper_H
#define Stepper_H

#include <virtualConstraintsNode.h>

class stepper : public virtualConstraintsNode
{
public:
    
    stepper() {};
    Eigen::Vector3d getStep() {};
    
private:  
    
    void checkState() {};
    void checkSide() {};
    Eigen::Vector3d calcStep() {};
    

};


#endif