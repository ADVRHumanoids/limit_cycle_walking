#ifndef STEP_STATE_H
#define STEP_STATE_H

#include <XBotInterface/MatLogger.hpp>

namespace mdof {

struct StepState {

    typedef std::shared_ptr<StepState> Ptr;

    StepState() :
        q_sag(0),
        q_lat(0),
        q_sag_min(0),
        q_sag_max(0),
        q_lat_min(0),
        q_lat_max(0),
        step_clearance(0),
        zmp_middle(0),
        height_com(0),
        distance_ankle_com(0)
    {
        durations.setZero();
    }

    double q_sag, q_sag_min, q_sag_max;
    double q_lat, q_lat_min, q_lat_max;

    double step_clearance;

    std::vector<Eigen::MatrixXd> zmp_vals;
    Eigen::VectorXd durations;

    double zmp_middle;

    double height_com;
    double distance_ankle_com;

    void log(std::string name, XBot::MatLogger::Ptr logger)
    {
        logger->add(name + "_q_sag", q_sag);
        logger->add(name + "_q_lat", q_lat);
        logger->add(name + "_q_sag_min", q_sag_min);
        logger->add(name + "_q_sag_max", q_sag_max);
        logger->add(name + "_q_lat_min", q_lat_min);
        logger->add(name + "_q_lat_max", q_lat_max);
//        logger->add(name + "_zmp_current", zmp_val_current);
//        logger->add(name + "_zmp_next", zmp_val_next);
        logger->add(name + "_height_com", height_com);
        logger->add(name + "_distance_ankle_com", distance_ankle_com);
    }

};
}
#endif // STEP_STATE_H
