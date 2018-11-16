#ifndef RobotInterface_H
#define RobotInterface_H


#include <Eigen/Dense>
// #include <XBotCore/CommandAdvr.h>
// #include "cartesian_interface/CartesianInterface.h"

#include <memory>


class robot_interface
    {
    public:
        
        typedef std::shared_ptr<robot_interface> Ptr;

        Eigen::Vector3d get_com() {return _com_state;};
        Eigen::Vector3d get_l_sole() {return _l_sole_state;};
        Eigen::Vector3d get_r_sole() {return _r_sole_state;};
        Eigen::Vector3d get_distance_ankle_to_com() {return _distance_ankle_to_com;};
        Eigen::Vector3d get_distance_l_to_r_foot() {return _distance_l_to_r_foot;};
        
        
    protected:
        
        Eigen::Vector3d _distance_ankle_to_com, _distance_l_to_r_foot;
        Eigen::Vector3d _com_state;
        Eigen::Vector3d _l_sole_state, _r_sole_state;
        
        bool _check_messages;
    };
    
    
#endif