#ifndef RobotInterface_H
#define RobotInterface_H


#include <Eigen/Dense>

#include <memory>


class robot_interface
    {
    public:
        
        enum class Side { Left = 0, Right = 1, Double = -1}; /*Side that is SWINGING*/     /*think a way to put here the values of step_y*/
        typedef std::shared_ptr<robot_interface> Ptr;

        Eigen::Vector3d get_com() {return _com_state.translation();};
        
        Eigen::Affine3d get_waist() {return _waist_state;};
        
        Eigen::Vector3d get_world_to_com() {return _world_to_com.translation();};
        
        Eigen::Vector3d get_distance_ankle_to_com(Side desired_side) 
        {
            if (desired_side == Side::Double) 
            {
                if (comparePoseSimmetry(_ankle_to_com[Side::Left], _ankle_to_com[Side::Right]))
                {
                    return _ankle_to_com[Side::Left].translation();
                }
                else std::cout << "Asked for distance from ankle to com during double stance. However, the distances are different. Left:" << _ankle_to_com[Side::Left].translation().transpose() << std::endl
                                                                                                                              << "Right:"  << _ankle_to_com[Side::Right].translation().transpose() << std::endl;
            }
            return _ankle_to_com[desired_side].translation();
            
        }
        
        Eigen::Vector3d get_distance_l_to_r_foot() {return _l_to_r_foot.translation();};
        
        Eigen::Vector3d get_sole(Side desired_side) 
        {
            if (desired_side == Side::Double) 
            {
                if (comparePoseSimmetry(_sole_state[Side::Left], _sole_state[Side::Right])) /*TODO chemmerda*/
                {
                    return _sole_state[Side::Left].translation();
                }
                else std::cout << "Asked for sole pose during double stance. However, the poses are different. Left: " << _sole_state[Side::Left].translation().transpose() << std::endl
                                                                                                          << "Right: " << _sole_state[Side::Right].translation().transpose() << std::endl;
            }
            else return _sole_state[desired_side].translation();  
        }
        
        Eigen::Affine3d get_sole_tot(Side desired_side)
        {
            if (desired_side == Side::Double) 
            {
                if (comparePoseSimmetry(_sole_state[Side::Left], _sole_state[Side::Right])) /*TODO chemmerda*/
                {
                    return _sole_state[Side::Left];
                }
                else std::cout << "Asked for sole pose during double stance. However, the poses are different. Left: " << _sole_state[Side::Left].matrix() << std::endl
                                                                                                            << "Right: " << _sole_state[Side::Right].matrix() << std::endl;
            }
            else return _sole_state[desired_side];
        }
        
        bool comparePoseSimmetry(Eigen::Affine3d p1, Eigen::Affine3d p2)
        {
            /*return true if the two pose are symmetric on the xz plane (the sagittal plane)*/
            Eigen::Vector3d p1_T = p1.translation();
            Eigen::Vector3d p2_T = p2.translation();
            
//             if ((p1_T.coeff(0) - p2_T.coeff(0)) <= 1e-2 && (p1_T.coeff(2) - p2_T.coeff(2)) <= 1e-2 && (fabs(p1_T.coeff(1)) - fabs(p2_T.coeff(1))) <= 1e-2) 
            if ((p1_T.coeff(2) - p2_T.coeff(2)) <= 1e-2 && (fabs(p1_T.coeff(1)) - fabs(p2_T.coeff(1))) <= 1e-2) 
            {
                return 1;
            }
            else
                return 0;
        }
        
        Eigen::Matrix<double, 6, 1> get_ft_sole(Side desired_side)
        {
            if (desired_side == Side::Double) 
            {
                std::cout << "Asked for sole force/torque during double stance.  Left: " << _sole_ft[Side::Left].matrix() << std::endl
                                                                             << "Right: " << _sole_ft[Side::Right].matrix() << std::endl;
                return _sole_ft[Side::Left];
            }
            else return _sole_ft[desired_side];
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
        Eigen::Affine3d _waist_state;
//         Eigen::Affine3d _zmp_state;
//         std::vector<Eigen::Affine3d> _sole_state;     /*0 is left, 1 is right*/
        std::map<robot_interface::Side, Eigen::Affine3d> _sole_state;
        std::map<robot_interface::Side, Eigen::Affine3d>_ankle_to_com;
        std::map<robot_interface::Side, Eigen::Matrix<double, 6,1> > _sole_ft;
        Eigen::Affine3d _world_to_com;
        Eigen::Affine3d _l_to_r_foot;
        bool _check_messages;
    };
    
    
#endif