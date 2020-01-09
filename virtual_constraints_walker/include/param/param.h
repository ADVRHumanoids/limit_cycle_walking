#ifndef WALKING_PARAM_H
#define WALKING_PARAM_H

#include <step_machine/step_machine.h>
#include <yaml-cpp/node/node.h>
#include <ros/node_handle.h>

class StepMachine::Param
{
public:

    Param();
    Param(YAML::Node);
    Param(ros::NodeHandle);

    double getInitialLowering() const {return _initial_lowering;}
    double getStepClearance() const {return _clearance_height;}
    double getStepDuration() const {return _step_duration;}
    Walker::Side getFirstSide() const {return _first_side;}
    int getMaxSteps() const {return _max_steps;}
    double getOffsetZmp() const {return _offset_zmp;}
    double getDurationDoubleStance() const {return _duration_double_stance;}
    double getStartTime() const {return _start_time;}
    double getLeanForward() const {return _lean_forward;}
    std::vector<double> getThresholdImpactRigth() const {return _thresholds_impact_right;}
    std::vector<double> getThresholdImpactLeft() const {return _thresholds_impact_left;}
    bool getRealImpactFlag() const {return _real_impacts;}
    double getMaxInclination() const {return _max_inclination;}
    double getMpcQ() const {return _mpc_Q;}
    double getMpcR() const {return _mpc_R;}
    bool getDurationPreviewWindow() const {return _duration_preview_window;}

    void setInitialLowering(double initial_lowering) {_initial_lowering = initial_lowering;}
    void setStepClearance(double clearance_height) {_clearance_height = clearance_height;}
    void setStepDuration(double step_duration) {_step_duration = step_duration;}
    void setFirstSide(Walker::Side first_side) {_first_side = first_side;}
    void setMaxSteps(int max_steps) {_max_steps = max_steps;}
    void setDurationDoubleStance(double duration_double_stance) {_duration_double_stance = _duration_double_stance;}
    void setOffsetZmp(double offset_zmp) {_offset_zmp = offset_zmp;}
    void setStartTime(double start_time) {_start_time = start_time;}
    void setLeanForward(double lean_forward) {_lean_forward = lean_forward;}
    void setThresholdImpactRight(std::vector<double> threshold_impact_right) {_thresholds_impact_right = threshold_impact_right;}
    void setThresholdImpactLeft(std::vector<double> threshold_impact_left) {_thresholds_impact_left = threshold_impact_left;}
    void setRealImpactFlag(bool real_impacts) {_real_impacts = real_impacts;}
    void setMaxInclination(double max_inclination) {_max_inclination = max_inclination;}
    void setMpcQ(double mpc_Q) {_mpc_Q = mpc_Q;}
    void setMpcR(double mpc_R) {_mpc_R = mpc_R;}
    void setDurationPreviewWindow(double duration_preview_window) {_duration_preview_window = duration_preview_window;}

private:

    double _initial_lowering;
    Side _first_side;
    int _max_steps;
    double _offset_zmp;
    double _start_time;
    double _lean_forward;
    std::vector<double> _thresholds_impact_right;
    std::vector<double> _thresholds_impact_left;
    bool _real_impacts;
    double _mpc_Q;
    double _mpc_R;
    double _duration_preview_window; //sec

    /* parameters step */
    double _clearance_height;
    double _step_duration;
    double _duration_double_stance;
    double _max_inclination;

};


#include <param/param.h>

StepMachine::Param::Param()
{
    /*default parameters*/
    _initial_lowering = -0.12;
    _first_side = Walker::Side::Left;
    _max_steps = 10;
    _offset_zmp = 0;
    _start_time = 1;
    _lean_forward = 0;
    _thresholds_impact_right = {50, 100};
    _thresholds_impact_left = {50, 100};
    _real_impacts = 0;
    _mpc_Q = 1000000;
    _mpc_R = 1;
    _duration_preview_window = 5; //sec

    /* parameters step */
    _clearance_height = 0.1;
    _step_duration = 2;
    _duration_double_stance = 0;
    _max_inclination = 0.1;
}

StepMachine::Param::Param(YAML::Node)
{

}

StepMachine::Param::Param(ros::NodeHandle)
{

}
#endif // WALKING_PARAM_H
