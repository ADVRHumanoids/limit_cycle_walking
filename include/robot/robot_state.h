﻿#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {

struct RobotState
{

    RobotState()
    {
        world_T_com.setZero();

        com_vel.setZero();

        world_T_foot[0].matrix().setZero();
        world_T_foot[1].matrix().setZero();

        world_T_waist.matrix().setZero();

        ankle_T_com[0].matrix().setZero();
        ankle_T_com[1].matrix().setZero();
    }

    /* feet are organized in arrays, first left second right */

    Eigen::Vector3d com_vel;

    std::array<Eigen::Affine3d, 2> world_T_foot;

    Eigen::Affine3d world_T_waist;

    std::array<Eigen::Affine3d, 2> ankle_T_com;

    Eigen::Vector3d world_T_com;

    void log(std::string name, XBot::MatLogger::Ptr logger)
    {
        logger->add(name + "_com_vel", com_vel);
        logger->add(name + "_lfoot", world_T_foot[0].translation());
        logger->add(name + "_rfoot", world_T_foot[1].translation());
        logger->add(name + "_waist", world_T_waist.translation());
        logger->add(name + "_lankle_to_com", ankle_T_com[0].translation());
        logger->add(name + "_rankle_to_com", ankle_T_com[1].translation());
        logger->add(name + "_com_pos", world_T_com);
    }
};
}
#endif // ROBOT_STATE_H
