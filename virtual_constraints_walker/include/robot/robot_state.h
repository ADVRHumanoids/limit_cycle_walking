#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {

struct RobotState
{

    RobotState() :
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

        com_pos.setZero();
        com_vel.setZero();
        zmp.setZero();

        foot_pos[0].matrix().setZero();
        foot_pos[1].matrix().setZero();

        waist_pos.matrix().setZero();
        ankle_T_com[0].matrix().setZero();
        ankle_T_com[1].matrix().setZero();
        foot_contact[0] = 1;
        foot_contact[1] = 1;
    }

    Eigen::Vector3d com_pos, com_vel, zmp;

    std::array<Eigen::Affine3d, 2> foot_pos;

    Eigen::Affine3d waist_pos;

    std::array<Eigen::Affine3d, 2> ankle_T_com;

    std::array<bool, 2> foot_contact;


    Eigen::Vector3d com_start;
    Eigen::Vector3d com_goal;

    std::array<Eigen::Affine3d, 2> foot_start;
    std::array<Eigen::Affine3d, 2> foot_goal;

    double step_duration;
    double step_clearing;

    int step_counter;
    int cycle_counter;

    double initial_q;

    double q_min, q_max;
    double steep_q;

    double theta;

    void log(XBot::MatLogger::Ptr logger)

    {
        logger->add("com_pos", com_pos);
        logger->add("com_vel", com_vel);
        logger->add("zmp", zmp);
        logger->add("lfoot", foot_pos[0].translation());
        logger->add("rfoot", foot_pos[1].translation());
        logger->add("contact", Eigen::Vector2d(foot_contact[0], foot_contact[1]));

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

        logger->add("step_clearing", step_clearing);
        logger->add("step_duration", step_duration);

    }
};

}
#endif // ROBOT_STATE_H
