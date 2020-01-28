#include <engine/sagittal_plane.h>

SagittalPlane::SagittalPlane(double dt) :
    _q(0),
    _delta_com(0),
    _disp_com(0),
    _disp_foot(0),
    _dt(dt)
{

}

void SagittalPlane::update(double q,
                           double q_min,
                           double q_max,
                           double distance_com_ankle)
{
    computeCom(q, distance_com_ankle, _delta_com);
    computeStep(q_min, q_max, distance_com_ankle, _disp_com, _disp_foot);
}

bool SagittalPlane::computeCom(double q,
                                 double distance_com_ankle,
                                 double& delta_com)
{
    /* compute com displacement, which now is linear */
    delta_com = fabs(distance_com_ankle) * tan(q);

    return true;
}

bool SagittalPlane::computeStep(double q_min,
                                double q_max,
                                double distance_com_ankle,
                                double& disp_com,
                                double& disp_foot)
{

    /* total q angle in one step */
    double q_tot = q_max - q_min;

    computeCom(q_tot, distance_com_ankle, disp_com);
    disp_foot = 2 * disp_com;

    return true;
    }
