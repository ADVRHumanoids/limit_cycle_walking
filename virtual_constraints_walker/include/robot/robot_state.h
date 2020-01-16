#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {

class RobotState
{
public:

    RobotState()
//        step_counter(0),
//        cycle_counter(0),
//        q_min(0),
//        q_max(0),
//        steep_q(0),
//        theta(0),
//        zmp(0)
    {
//        world_T_com_start.setZero();
//        world_T_com_goal.setZero();

//        world_T_foot_start[0].matrix().setZero();
//        world_T_foot_start[1].matrix().setZero();

//        world_T_foot_goal[0].matrix().setZero();
//        world_T_foot_goal[1].matrix().setZero();

        world_T_com.setZero();
        com_vel.setZero();

        world_T_foot[0].matrix().setZero();
        world_T_foot[1].matrix().setZero();

        world_T_waist.matrix().setZero();
        ankle_T_com[0].matrix().setZero();
        ankle_T_com[1].matrix().setZero();
//        foot_contact[0] = 1;
//        foot_contact[1] = 1;
    }

    Eigen::Vector3d getComVel() const;
    void setComVel(const Eigen::Vector3d &value);

//    double getZmp() const;
//    void setZmp(const double &value);

    std::array<Eigen::Affine3d, 2> getFeet() const;
    void setFeet(const std::array<Eigen::Affine3d, 2> &value);

//    void setLFoot(const Eigen::Affine3d &value);
//    void setRFoot(const Eigen::Affine3d &value);

//    std::array<Eigen::Affine3d, 2> getFootStart() const;
//    void setFootStart(const std::array<Eigen::Affine3d, 2> &value);

//    std::array<Eigen::Affine3d, 2> getFootGoal() const;
//    void setFootGoal(const std::array<Eigen::Affine3d, 2> &value);

    Eigen::Affine3d getWaist() const;
    void setWaist(const Eigen::Affine3d &value);

    std::array<Eigen::Affine3d, 2> getAnkleCom() const;
    void setAnkleCom(const std::array<Eigen::Affine3d, 2> &value);

    Eigen::Vector3d getCom() const;
    void setCom(const Eigen::Vector3d &value);

//    Eigen::Vector3d getComStart() const;
//    void setComStart(const Eigen::Vector3d &value);

//    Eigen::Vector3d getComGoal() const;
//    void setComGoal(const Eigen::Vector3d &value);

//    std::array<bool, 2> getFootContact() const;
//    void setFootContact(const std::array<bool, 2> &value);

//    double getStepDuration() const;
//    void setStepDuration(double value);

//    double getStepClearing() const;
//    void setStepClearing(double value);

//    int getCycleCounter() const;
//    void setCycleCounter(int value);

//    double getInitialQ() const;
//    void setInitialQ(double value);

//    double getQMin() const;
//    void setQMin(double value);

//    double getQMax() const;
//    void setQMax(double value);

//    double getSteepQ() const;
//    void setSteepQ(double value);


//    double getTheta() const;
//    void setTheta(double value);

//    double getStartWalkTime() const;
//    void setStartWalkTime(double value);


//    double getTStart() const;
//    void setTStart(double value);

//    double getTEnd() const;
//    void setTEnd(double value);

//    bool getSwingLeg() const;
//    void setSwingLeg(bool swingLeg);

private:

    /* feet are organized in arrays, first left second right */

    Eigen::Vector3d com_vel;

//    double zmp;

    std::array<Eigen::Affine3d, 2> world_T_foot;

    Eigen::Affine3d world_T_waist;

    std::array<Eigen::Affine3d, 2> ankle_T_com;

    Eigen::Vector3d world_T_com;

    void log(XBot::MatLogger::Ptr logger)
    {
        logger->add("com_pos", world_T_com);
        logger->add("com_vel", com_vel);
//        logger->add("zmp", zmp);
        logger->add("lfoot", world_T_foot[0].translation());
        logger->add("rfoot", world_T_foot[1].translation());
//        logger->add("contact", Eigen::Vector2d(foot_contact[0], foot_contact[1]));
//        logger->add("com_start", world_T_com_start);
//        logger->add("com_goal", world_T_com_goal);
//        logger->add("step_start", world_T_foot_start);
//        logger->add("step_goal", world_T_foot_goal);
//        logger->add("step_counter", step_counter);
//        logger->add("cycle_counter", cycle_counter);
//        logger->add("q_min", q_min);
//        logger->add("q_max", q_max);
//        logger->add("steep_q", steep_q);
//        logger->add("step_direction", theta);

//        logger->add("step_clearing", step_clearing);
//        logger->add("step_duration", step_duration);

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

//double RobotState::getZmp() const
//{
//    return zmp;
//}

//void RobotState::setZmp(const double &value)
//{
//    zmp = value;
//}

//double RobotState::getTheta() const
//{
//    return theta;
//}

//void RobotState::setTheta(double value)
//{
//    theta = value;
//}

//double RobotState::getStartWalkTime() const
//{
//    return start_walk_time;
//}

//void RobotState::setStartWalkTime(double value)
//{
//    start_walk_time = value;
//}

//double RobotState::getTStart() const
//{
//    return t_start;
//}

//void RobotState::setTStart(double value)
//{
//    t_start = value;
//}

//double RobotState::getTEnd() const
//{
//    return t_end;
//}

//void RobotState::setTEnd(double value)
//{
//    t_end = value;
//}

//bool RobotState::getSwingLeg() const
//{
//    return swing_leg;
//}

//void RobotState::setSwingLeg(bool value)
//{
//    swing_leg = value;
//}


//double RobotState::getQMax() const
//{
//    return q_max;
//}

//void RobotState::setQMax(double value)
//{
//    q_max = value;
//}

//double RobotState::getSteepQ() const
//{
//    return steep_q;
//}

//void RobotState::setSteepQ(double value)
//{
//    steep_q = value;
//}

//double RobotState::getQMin() const
//{
//    return q_min;
//}

//void RobotState::setQMin(double value)
//{
//    q_min = value;
//}

//double RobotState::getInitialQ() const
//{
//    return initial_q;
//}

//void RobotState::setInitialQ(double value)
//{
//    initial_q = value;
//}

//int RobotState::getCycleCounter() const
//{
//    return cycle_counter;
//}

//void RobotState::setCycleCounter(int value)
//{
//    cycle_counter = value;
//}

//double RobotState::getStepDuration() const
//{
//    return step_duration;
//}

//void RobotState::setStepDuration(double value)
//{
//    step_duration = value;
//}

//double RobotState::getStepClearing() const
//{
//    return step_clearing;
//}

//void RobotState::setStepClearing(double value)
//{
//    step_clearing = value;
//}

//std::array<bool, 2> RobotState::getFootContact() const
//{
//    return foot_contact;
//}

//void RobotState::setFootContact(const std::array<bool, 2> &value)
//{
//    foot_contact = value;
//}

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

//std::array<Eigen::Affine3d, 2> RobotState::getFootGoal() const
//{
//    return world_T_foot_goal;
//}

//void RobotState::setFootGoal(const std::array<Eigen::Affine3d, 2> &value)
//{
//    world_T_foot_goal = value;
//}

//std::array<Eigen::Affine3d, 2> RobotState::getFootStart() const
//{
//    return world_T_foot_start;
//}

//void RobotState::setFootStart(const std::array<Eigen::Affine3d, 2> &value)
//{
//    world_T_foot_start = value;
//}

std::array<Eigen::Affine3d, 2> RobotState::getFeet() const
{
    return world_T_foot;
}

void RobotState::setFeet(const std::array<Eigen::Affine3d, 2> &value)
{
    world_T_foot = value;
}

}
#endif // ROBOT_STATE_H
