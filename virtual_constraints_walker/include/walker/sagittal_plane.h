#ifndef SAGITTAL_PLANE_H
#define SAGITTAL_PLANE_H

#include <XBotLogger/Logger.hpp>

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

    void update(double q,
                double q_min,
                double q_max,
                double height_com);

    double getDeltaCom(){return _delta_com;}
    
    double getDeltaComTot() {return _disp_com;}
    double getDeltaFootTot() {return _disp_foot;}

private:
    
    
    bool computeCom(double q,
                    double height_com);

    bool computeStep(double q_min,
                     double q_max,
                     double height_com);

    double _q;

    double _delta_com;
    
    /* total displacement of com */
    double _disp_com;
    
    /* total displacement of foot */
    double _disp_foot;

    /* dt */
    double _dt;






};

#endif // SAGITTAL_PLANE_H
