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
        
        Eigen::Vector3d get_distance_ankle_to_com(Side desired_side) 
        {
            if (desired_side == Side::Double) 
            {
                if (comparePoseSimmetry(_ankle_to_com[Side::Left], _ankle_to_com[Side::Right]))
                {
                return _ankle_to_com[Side::Left].translation();
                }
                else std::cout << "double support error (distance)" << std::endl;
            }
            return _ankle_to_com[desired_side].translation();
            
        }
        Eigen::Vector3d get_distance_l_to_r_foot() {return _l_to_r_foot.translation();};
        
        Eigen::Vector3d get_sole(Side desired_side) 
        {
            if (desired_side == Side::Double) 
            {
                if (comparePoseSimmetry(_sole_state[Side::Left], _sole_state[Side::Right]))
                {
                return _sole_state[Side::Left].translation();
                }
                else std::cout << "double support error (sole)" << std::endl;
            }
            else return _sole_state[desired_side].translation();  
        }
        
        bool comparePoseSimmetry(Eigen::Affine3d p1, Eigen::Affine3d p2)
        {
            Eigen::Vector3d p1_T = p1.translation();
            Eigen::Vector3d p2_T = p2.translation();
            
            if ((p1_T.coeff(0) - p2_T.coeff(0)) <= 1e-5 && (p1_T.coeff(2) - p2_T.coeff(2)) <= 1e-5 && (fabs(p1_T.coeff(1)) - fabs(p2_T.coeff(1))) <= 1e-5) 
            {
                return 1;
            }
            else
                return 0;
        }
        
    protected:
        
        
    
        friend std::ostream& operator<<(std::ostream& os, Side s)
        {
            switch (s)
            {
                case Side::Left :  return os << "Left";
                case Side::Right :  return os << "Right";
                case Side::Double :  return os << "Double";
                default : return os << "wrong side";
            }
        };

        Eigen::Affine3d _com_state;
//         std::vector<Eigen::Affine3d> _sole_state;     /*0 is left, 1 is right*/
        std::map<robot_interface::Side, Eigen::Affine3d> _sole_state;
        std::map<robot_interface::Side, Eigen::Affine3d>_ankle_to_com;
        Eigen::Affine3d _l_to_r_foot;
        bool _check_messages;
    };
    
    
#endif