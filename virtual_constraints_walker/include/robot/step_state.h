#ifndef STEP_STATE_H
#define STEP_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {
struct StepState {

    StepState() :
        step_counter(0),
        cycle_counter(0),
        q_min(0),
        q_max(0),
        steep_q(0),
        theta(0)
    {
        com_start.setZero();
        com_goal.setZero();
        foot_start[0].matrix().setZero();
        foot_start[1].matrix().setZero();

        foot_goal[0].matrix().setZero();
        foot_goal[1].matrix().setZero();
    }

    Eigen::Vector3d com_start;
    Eigen::Vector3d com_goal;

    std::array<Eigen::Affine3d, 2> foot_start;
    std::array<Eigen::Affine3d, 2> foot_goal;

    int step_counter;
    int cycle_counter;

    double q_min, q_max;
    double steep_q;

    double theta;

    void log(XBot::MatLogger::Ptr logger)
    {
        logger->add("com_start", com_start);
        logger->add("com_goal", com_goal);
        logger->add("step_start", foot_start);
        logger->add("step_goal", foot_goal);
        logger->add("step_counter", step_counter);
        logger->add("cycle_counter", cycle_counter);
        logger->add("q_min", q_min);
        logger->add("q_max", q_max);
        logger->add("steep_q", steep_q);
        logger->add("step_direction", theta);
    }

};
}
#endif // STEP_STATE_H
