#include <engine/sagittal_plane.h>

SagittalPlane::SagittalPlane(double dt) :
    _q(0),
    _delta_com(0),
    _delta_foot(0),
    _dt(dt)
{

}

bool SagittalPlane::update(double q,
                           double distance_com_ankle)
{
    /* compute com displacement, which now is linear */
    _delta_com = fabs(distance_com_ankle) * tan(q);
    _delta_foot = 2 * _delta_com;

    return true;
}

void SagittalPlane::log(std::string name, XBot::MatLogger::Ptr logger)
{
    std::string className("sagittal_plane_");

    logger->add(name + "_q", _q);
    logger->add(name + "_delta_com", _delta_com);
    logger->add(name + "_delta_foot", _delta_foot);
    logger->add(name + "_dt", _dt);
}

