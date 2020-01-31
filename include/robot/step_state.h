#ifndef STEP_STATE_H
#define STEP_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {

struct StepState {

    typedef std::shared_ptr<StepState> Ptr;

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
        distance_ankle_com(0),
        disable_step(false),
        t_min(0),
        t_max(0)
    {
    }

    double q, q_fake, q_min, q_max;
    /* TODO these are needed if you want to use time instead of q to advance with the preview window */
    double t_min, t_max;

    double step_duration, step_clearance;

    double zmp_val_current;
    double zmp_val_next;

    double height_com;
    double distance_ankle_com;

    bool disable_step;

    void log(std::string name, XBot::MatLogger::Ptr logger)
    {
        logger->add(name + "_q", q);
        logger->add(name + "_q_fake", q_fake);
        logger->add(name + "_q_min", q_min);
        logger->add(name + "_q_max", q_max);
        logger->add(name + "_zmp_current", zmp_val_current);
        logger->add(name + "_zmp_next", zmp_val_next);
        logger->add(name + "_height_com", height_com);
        logger->add(name + "_distance_ankle_com", distance_ankle_com);
        logger->add(name + "_t_min", t_min);
        logger->add(name + "_t_max", t_max);
    }

};
}
#endif // STEP_STATE_H
