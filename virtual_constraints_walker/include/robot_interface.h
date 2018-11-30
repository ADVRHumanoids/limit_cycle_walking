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

        Eigen::Vector3d get_com() {return _com_state.translation();};
        Eigen::Vector3d get_distance_ankle_to_com(int n) {return _ankle_to_com[n].translation();};
        Eigen::Vector3d get_distance_l_to_r_foot() {return _l_to_r_foot.translation();};
        Eigen::Vector3d get_sole(int n) {return _sole_state[n].translation();}   /*TODO is this a good implementation?*/
        
    protected:
        
        Eigen::Affine3d _com_state;
        std::vector<Eigen::Affine3d> _sole_state;     /*0 is left, 1 is right*/
        std::vector<Eigen::Affine3d> _ankle_to_com; /*TODO is this a good implementation?*/
        Eigen::Affine3d _l_to_r_foot;
        bool _check_messages;
    };
    
    
#endif