#ifndef WALKING_PARAM_H
#define WALKING_PARAM_H

#include <walker/walker.h>
#include <yaml-cpp/node/node.h>
#include <ros/node_handle.h>

class Walker::Param
{
public:

    typedef std::shared_ptr<Param> Ptr;
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


    double getInitialLowering() const {return _initial_lowering;}
    bool getFirstSideStep() const {return _first_side_step;}
    double getLeanForward() const {return _lean_forward;}

    double getStepClearance() const {return _step_clearance;}
    double getStepDuration() const {return _step_duration;}
    double getDoubleStanceDuration() const {return _double_stance_duration;}
    double getMaxInclination() const {return _max_inclination;}

    double getMpcQ() const {return _mpc_Q;}
    double getMpcR() const {return _mpc_R;}
    double getHorizonDuration() const {return _horizon_duration;}
    double getZmpOffset() const {return _zmp_offset;}

    void log(std::string name, XBot::MatLogger2::Ptr logger);

private:


    bool parseYAML(YAML::Node yaml_node);

    /* init parameters */
    double _initial_lowering;
    bool _first_side_step;
    double _lean_forward;

    /* walking parameters */
    double _step_clearance;
    double _step_duration;
    double _double_stance_duration;
    double _max_inclination;

    /* mpc parameters */
    double _mpc_Q;
    double _mpc_R;
    double _horizon_duration;
    double _zmp_offset;

};

#endif // WALKING_PARAM_H
