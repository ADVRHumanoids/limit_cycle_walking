﻿#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {

class RobotState
{
public:

    RobotState() :
        step_counter(0),
        cycle_counter(0),
        q_min(0),
        q_max(0),
        steep_q(0),
        theta(0)
    {
        world_T_com_start.setZero();
        world_T_com_goal.setZero();
        world_T_foot_start[0].matrix().setZero();
        world_T_foot_start[1].matrix().setZero();

        world_T_foot_goal[0].matrix().setZero();
        world_T_foot_goal[1].matrix().setZero();

        world_T_com.setZero();
        com_vel.setZero();
        zmp.setZero();

        world_T_foot[0].matrix().setZero();
        world_T_foot[1].matrix().setZero();

        world_T_waist.matrix().setZero();
        ankle_T_com[0].matrix().setZero();
        ankle_T_com[1].matrix().setZero();
        foot_contact[0] = 1;
        foot_contact[1] = 1;
    }

    Eigen::Vector3d getComVel() const;
    void setComVel(const Eigen::Vector3d &value);

    Eigen::Vector3d getZmp() const;
    void setZmp(const Eigen::Vector3d &value);

    std::array<Eigen::Affine3d, 2> getFoot() const;
    void setFoot(const std::array<Eigen::Affine3d, 2> &value);

    void setLFoot(const Eigen::Affine3d &value);
    void setRFoot(const Eigen::Affine3d &value);

    std::array<Eigen::Affine3d, 2> getFootStart() const;
    void setFootStart(const std::array<Eigen::Affine3d, 2> &value);

    std::array<Eigen::Affine3d, 2> getFootGoal() const;
    void setFootGoal(const std::array<Eigen::Affine3d, 2> &value);

    Eigen::Affine3d getWaist() const;
    void setWaist(const Eigen::Affine3d &value);

    std::array<Eigen::Affine3d, 2> getAnkleCom() const;
    void setAnkleCom(const std::array<Eigen::Affine3d, 2> &value);

    Eigen::Vector3d getCom() const;
    void setCom(const Eigen::Vector3d &value);

    Eigen::Vector3d getComStart() const;
    void setComStart(const Eigen::Vector3d &value);

    Eigen::Vector3d getComGoal() const;
    void setComGoal(const Eigen::Vector3d &value);

    std::array<bool, 2> getFootContact() const;
    void setFootContact(const std::array<bool, 2> &value);

    double getStepDuration() const;
    void setStepDuration(double value);

    double getStepClearing() const;
    void setStepClearing(double value);

    int getCycleCounter() const;
    void setCycleCounter(int value);

    double getInitialQ() const;
    void setInitialQ(double value);

    double getQMin() const;
    void setQMin(double value);

    double getQMax() const;
    void setQMax(double value);

    double getSteepQ() const;
    void setSteepQ(double value);


    double getTheta() const;
    void setTheta(double value);

    double getStartTime() const;
    void setStartTime(double value);

    double getQ() const
    {
        /* takes:
         * ankle_T_com
         * world_T_com
         * theta
         * offset
         * current_stance_leg
         */

        /* TODO remember the stuff about offset ! */
        bool current_stance_leg;

        /* if in double stance, take one leg, it's the same */
        if ( foot_contact[0] == 1 && foot_contact[1] == 1 )
        {
            current_stance_leg = 0;
        }
        else
        {
            if (foot_contact[0] == 1)
            {
                current_stance_leg = 0;
            }
            else
            {
                current_stance_leg = 1;
            }
        }

        Eigen::Vector3d dist_com;
        double q;
        double offset_q;
        /* 2D rot matrix, theta in radian */
        Eigen::Rotation2Dd rot2(theta);


        /* distance between current com and starting com (updated at each step)*/
        dist_com = world_T_com - world_T_com_start;

        /* rotate back com */
        dist_com.head(2) = rot2.toRotationMatrix() * dist_com.head(2);

        /* depending on the length of the step, remove offset */
        offset_q = 0;

        /* compute q, inclination angle of the robot */
        q = ( dist_com(0) / fabs(ankle_T_com[current_stance_leg].translation()(2)) ) - offset_q;

        return q;
    }


private:

    /* feet are organized in arrays, first left second right */

    Eigen::Vector3d com_vel, zmp;

    std::array<Eigen::Affine3d, 2> world_T_foot;
    std::array<Eigen::Affine3d, 2> world_T_foot_start;
    std::array<Eigen::Affine3d, 2> world_T_foot_goal;

    Eigen::Affine3d world_T_waist;

    std::array<Eigen::Affine3d, 2> ankle_T_com;

    Eigen::Vector3d world_T_com;
    Eigen::Vector3d world_T_com_start;
    Eigen::Vector3d world_T_com_goal;

    std::array<bool, 2> foot_contact;


    double step_duration;
    double step_clearing;

    int step_counter;
    int cycle_counter;

    double initial_q;

    double q_min, q_max;

    double steep_q;

    /* direction of step */
    double theta;

    double start_walk;

    void log(XBot::MatLogger::Ptr logger)
    {
        logger->add("com_pos", world_T_com);
        logger->add("com_vel", com_vel);
        logger->add("zmp", zmp);
        logger->add("lfoot", world_T_foot[0].translation());
        logger->add("rfoot", world_T_foot[1].translation());
        logger->add("contact", Eigen::Vector2d(foot_contact[0], foot_contact[1]));

        logger->add("com_start", world_T_com_start);
        logger->add("com_goal", world_T_com_goal);
        logger->add("step_start", world_T_foot_start);
        logger->add("step_goal", world_T_foot_goal);
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

Eigen::Vector3d RobotState::getComVel() const
{
    return com_vel;
}

void RobotState::setComVel(const Eigen::Vector3d &value)
{
    com_vel = value;
}

Eigen::Vector3d RobotState::getZmp() const
{
    return zmp;
}

void RobotState::setZmp(const Eigen::Vector3d &value)
{
    zmp = value;
}

double RobotState::getTheta() const
{
    return theta;
}

void RobotState::setTheta(double value)
{
    theta = value;
}

double RobotState::getStartTime() const
{
    return start_time;
}

void RobotState::setStartTime(double value)
{
    start_time = value;
}


double RobotState::getQMax() const
{
    return q_max;
}

void RobotState::setQMax(double value)
{
    q_max = value;
}

double RobotState::getSteepQ() const
{
    return steep_q;
}

void RobotState::setSteepQ(double value)
{
    steep_q = value;
}

double RobotState::getQMin() const
{
    return q_min;
}

void RobotState::setQMin(double value)
{
    q_min = value;
}

double RobotState::getInitialQ() const
{
    return initial_q;
}

void RobotState::setInitialQ(double value)
{
    initial_q = value;
}

int RobotState::getCycleCounter() const
{
    return cycle_counter;
}

void RobotState::setCycleCounter(int value)
{
    cycle_counter = value;
}

double RobotState::getStepDuration() const
{
    return step_duration;
}

void RobotState::setStepDuration(double value)
{
    step_duration = value;
}

double RobotState::getStepClearing() const
{
    return step_clearing;
}

void RobotState::setStepClearing(double value)
{
    step_clearing = value;
}

std::array<bool, 2> RobotState::getFootContact() const
{
    return foot_contact;
}

void RobotState::setFootContact(const std::array<bool, 2> &value)
{
    foot_contact = value;
}

Eigen::Vector3d RobotState::getCom() const
{
    return world_T_com;
}

void RobotState::setCom(const Eigen::Vector3d &value)
{
    world_T_com = value;
}

std::array<Eigen::Affine3d, 2> RobotState::getAnkleCom() const
{
    return ankle_T_com;
}

void RobotState::setAnkleCom(const std::array<Eigen::Affine3d, 2> &value)
{
    ankle_T_com = value;
}

Eigen::Affine3d RobotState::getWaist() const
{
    return world_T_waist;
}

void RobotState::setWaist(const Eigen::Affine3d &value)
{
    world_T_waist = value;
}

std::array<Eigen::Affine3d, 2> RobotState::getFootGoal() const
{
    return world_T_foot_goal;
}

void RobotState::setFootGoal(const std::array<Eigen::Affine3d, 2> &value)
{
    world_T_foot_goal = value;
}

std::array<Eigen::Affine3d, 2> RobotState::getFootStart() const
{
    return world_T_foot_start;
}

void RobotState::setFootStart(const std::array<Eigen::Affine3d, 2> &value)
{
    world_T_foot_start = value;
}

std::array<Eigen::Affine3d, 2> RobotState::getFoot() const
{
    return world_T_foot;
}

void RobotState::setFoot(const std::array<Eigen::Affine3d, 2> &value)
{
    world_T_foot = value;
}

}
#endif // ROBOT_STATE_H
