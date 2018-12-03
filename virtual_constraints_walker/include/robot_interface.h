#ifndef RobotInterface_H
#define RobotInterface_H


#include <Eigen/Dense>

#include <memory>


class robot_interface
    {
    public:
        
        enum class Side { Left = 0, Right = 1, Double = -1}; /*Side that is SWINGING*/    
        
        typedef std::shared_ptr<robot_interface> Ptr;

        Eigen::Vector3d get_com() {return _com_state.translation();};
        Eigen::Vector3d get_distance_ankle_to_com(Side desired_side) {return _ankle_to_com[static_cast<int>(desired_side)].translation();};
        Eigen::Vector3d get_distance_l_to_r_foot() {return _l_to_r_foot.translation();};
        Eigen::Vector3d get_sole(Side desired_side) {return _sole_state[static_cast<int>(desired_side)].translation();} 
        
        
    protected:
        
        
    
        friend std::ostream& operator<<(std::ostream& os, Side s)
        {
            if (s == Side::Left)
                return os << "Left";
            else if (s == Side::Right)
               return os << "Right"; 
        };
        
        Eigen::Affine3d _com_state;
        std::vector<Eigen::Affine3d> _sole_state;     /*0 is left, 1 is right*/
        std::vector<Eigen::Affine3d> _ankle_to_com;
        Eigen::Affine3d _l_to_r_foot;
        bool _check_messages;
    };
    
    
#endif