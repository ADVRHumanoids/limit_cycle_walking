#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <matlogger2/matlogger2.h>

namespace mdof {

struct RobotState
{

    RobotState()
    {
        world_T_com.setZero();

        world_T_ankle[0].matrix().setZero();
        world_T_ankle[1].matrix().setZero();


        com_vel.setZero();

        world_T_foot[0].matrix().setZero();
        world_T_foot[1].matrix().setZero();

        world_T_waist.matrix().setZero();
        world_T_torso.matrix().setZero();

        ankle_T_com[0].matrix().setZero();
        ankle_T_com[1].matrix().setZero();

        feet_contact[0] = false;
        feet_contact[1] = false;

        impact[0] = false;
        impact[1] = false;

    }

    /* feet are organized in arrays, first left second right */

    Eigen::Vector3d com_vel;

    std::array<Eigen::Affine3d, 2> world_T_foot;

    Eigen::Affine3d world_T_waist, world_T_torso;

    std::array<Eigen::Affine3d, 2> ankle_T_com;

    std::array<Eigen::Affine3d, 2> world_T_ankle;

    Eigen::Vector3d world_T_com;

    std::array<bool, 2> feet_contact;

    std::array<bool, 2> impact;


    void log(std::string name, XBot::MatLogger2::Ptr logger)
    {
        logger->add(name + "_com_vel", com_vel);
        logger->add(name + "_l_foot", world_T_foot[0].translation());
        logger->add(name + "_r_foot", world_T_foot[1].translation());
        logger->add(name + "_waist", world_T_waist.translation());
        logger->add(name + "_l_ankle_to_com", ankle_T_com[0].translation());
        logger->add(name + "_r_ankle_to_com", ankle_T_com[1].translation());
        logger->add(name + "_com_pos", world_T_com);
        logger->add(name + "_left_contact", feet_contact[0]);
        logger->add(name + "_right_contact", feet_contact[1]);
    }
};
}
#endif // ROBOT_STATE_H
