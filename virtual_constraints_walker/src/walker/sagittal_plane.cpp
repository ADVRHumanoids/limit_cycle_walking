#include <walker/sagittal_plane.h>

void SagittalPlane::update(double q,
                           double q_min,
                           double q_max,
                           double height_com)
{
    computeStep(q_min, q_max, height_com);
    computeCom(q, height_com);

    /* should I compute everything here? */
}

bool SagittalPlane::computeCom(double q,
                  double height_com)
{
    /* compute com displacement, which now is linear */
    _delta_com = fabs(height_com) * tan(q);

    return true;
}

bool SagittalPlane::computeStep(double q_min,
                                double q_max,
                                double height_com)
{

    /* total q angle in one step */
    double q_tot = q_max - q_min;

    _disp_com = computeCom(q_tot, height_com);
    _disp_foot = 2 * _disp_com;

    return true;
}
