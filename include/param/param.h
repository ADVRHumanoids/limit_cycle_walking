#ifndef WALKING_PARAM_H
#define WALKING_PARAM_H

#include <walker/walker.h>
#include <yaml-cpp/node/node.h>
#include <ros/node_handle.h>

class Walker::Param
{
public:

    /**
    * @brief Default (empty) problem
    */
    Param();
    /**
     * @brief Construct from a YAML description
     * @param yaml_node YAML node containing a problem_description node
     */
    Param(YAML::Node yaml_node);

    /**
     * @brief Construct from a ROS Node
     */
    Param(ros::NodeHandle nh);

//    double getInitialLowering() const {return _initial_lowering;}
//    double getStepClearance() const {return _clearance_height;}
//    double getStepDuration() const {return _step_duration;}
//    bool getFirstSide() const {return _first_side;}
//    int getMaxSteps() const {return _max_steps;}
//    double getZmpOffset() const {return _zmp_offset;}
//    double getDurationDoubleStance() const {return _duration_double_stance;}
//    double getStartTime() const {return _start_time;}
//    double getLeanForward() const {return _lean_forward;}
//    bool getRealImpactFlag() const {return _real_impacts;}
//    double getMaxInclination() const {return _max_inclination;}
//    double getMpcQ() const {return _mpc_Q;}
//    double getMpcR() const {return _mpc_R;}
//    bool getDurationPreviewWindow() const {return _duration_preview_window;}

//    void setInitialLowering(double initial_lowering) {_initial_lowering = initial_lowering;}
//    void setStepClearance(double clearance_height) {_clearance_height = clearance_height;}
//    void setStepDuration(double step_duration) {_step_duration = step_duration;}
//    void setFirstSide(Walker::Side first_side) {_first_side = first_side;}
//    void setMaxSteps(int max_steps) {_max_steps = max_steps;}
//    void setDurationDoubleStance(double duration_double_stance) {_duration_double_stance = _duration_double_stance;}
//    void setOffsetZmp(double offset_zmp) {_offset_zmp = offset_zmp;}
//    void setStartTime(double start_time) {_start_time = start_time;}
//    void setLeanForward(double lean_forward) {_lean_forward = lean_forward;}
//    void setThresholdImpactRight(std::vector<double> threshold_impact_right) {_thresholds_impact_right = threshold_impact_right;}
//    void setThresholdImpactLeft(std::vector<double> threshold_impact_left) {_thresholds_impact_left = threshold_impact_left;}
//    void setRealImpactFlag(bool real_impacts) {_real_impacts = real_impacts;}
//    void setMaxInclination(double max_inclination) {_max_inclination = max_inclination;}
//    void setMpcQ(double mpc_Q) {_mpc_Q = mpc_Q;}
//    void setMpcR(double mpc_R) {_mpc_R = mpc_R;}
//    void setDurationPreviewWindow(double duration_preview_window) {_duration_preview_window = duration_preview_window;}

    /* init parameters */
    double initial_lowering;
    bool first_side_step;
    double lean_forward;

    /* walking parameters */
    double clearance_height;
    double step_duration;
    double double_stance_duration;
    double max_inclination;

    /* mpc parameters */
    double mpc_Q;
    double mpc_R;
    double horizon_duration;
    double zmp_offset;

};

#endif // WALKING_PARAM_H
