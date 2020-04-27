#ifndef SAGITTAL_PLANE_H
#define SAGITTAL_PLANE_H

#include <matlogger2/matlogger2.h>

/**
 * this walker is conceived having in mind a 'normal walk':
 * while walking I'm not thinking at each step, but only at a general direction.
 * Hence, I'm treating the robot as a point (represented by the com), and I'm giving a direction to this point.
 * Footsteps are planned accordingly.
 * (Sometimes it can be the opposite: I may be more interested in a specific stepping motion while not thinking about
 * the center of mass motion. This makes me think that there are three parameters:
 * 1. general idea of motion
 * 2. com motion
 * 3. step motion
 * Can I assume that the general idea of motion coincides with the sagittal com, since our 'principal component of motion'
 * is in the sagittal plane?
**/

class SagittalPlane {
public:

    typedef std::shared_ptr< SagittalPlane > Ptr;

    SagittalPlane(double dt);

    bool update(double q,
                 double distance_com_ankle);

    double getDeltaCom(){return _delta_com;}
    double getDeltaFoot() {return _delta_foot;}

    void log(std::string name, XBot::MatLogger2::Ptr logger);

private:

    double _q;

    /* displacement of com */
    double _delta_com;
    
    /* displacement of foot */
    double _delta_foot;

    /* dt */
    double _dt;






};

#endif // SAGITTAL_PLANE_H
