#ifndef STEP_STATE_H
#define STEP_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {

struct StepState {

    StepState() :
        q(0),
        q_fake(0),
        q_min(0),
        q_max(0),
        step_duration(0),
        step_clearance(0),
        zmp_val_current(0),
        zmp_val_next(0),
        height_com(0),
        distance_ankle_com(0)
    {
    }

    double q, q_fake, q_min, q_max;

    double step_duration, step_clearance;

    double zmp_val_current;
    double zmp_val_next;

    double height_com;
    double distance_ankle_com;

    void log(XBot::MatLogger::Ptr logger)
    {
        logger->add("q", q);
        logger->add("q_fake", q_fake);
        logger->add("q_min", q_min);
        logger->add("q_max", q_max);
        logger->add("zmp_current", zmp_val_current);
        logger->add("zmp_next", zmp_val_next);
        logger->add("height_com", height_com);
        logger->add("distance_ankle_com", distance_ankle_com);

    }

};
}
#endif // STEP_STATE_H
