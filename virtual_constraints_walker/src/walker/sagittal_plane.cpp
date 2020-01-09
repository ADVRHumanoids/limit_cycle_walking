#include <walker/sagittal_plane.h>

//bool StepMachine::update()
//{
//    _step_state->com_start = _step_state->com_goal;
//    _step_state->foot_start = _step_state->foot_pos;

//    robot_state.com_goal =


//    double q1_max_new = _step_state->

//    q1_max_new = _q1_max;
//    Eigen::Matrix2d R_steer;




//            /* TODO refactor: this is needed for the steering and the length step */

//    double theta = _theta_steer; // change heading
//    R_steer << cos(theta), -sin(theta),
//            sin(theta), cos(theta);

//    _q1_max = 0.05;
//            //                q1_max_new = _q1_max;
//            //                if (_step_counter >= 4 && _step_counter < 5)       /*Left*/
//            //                {
//            //                    _q1_max = 0.001;
//            //                    q1_max_new = _q1_max; // change step length
//            //                    double theta = _theta_steer; // change heading
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                    sin(theta), cos(theta);
//            //                }
//            //                else if (_step_counter >= 5 && _step_counter < 6)   /*Right*/
//            //                {
//            //                    _q1_max = 0.02;
//            //                    q1_max_new = _q1_max; // change step length
//            //                    double theta = _theta_steer; // change heading
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                    sin(theta), cos(theta);
//            //                }
//            //                else if (_step_counter >= 6 && _step_counter < 7)   /*Left*/
//            //                {
//            //                    _q1_max = 0.05;
//            //                    q1_max_new = _q1_max; // change step length
//            //                    double theta = _theta_steer; // change heading
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                    sin(theta), cos(theta);
//            //                }
//            //                else if (_step_counter >= 7 && _step_counter < 8)  /*Right*/
//            //                {
//            //                    _q1_max = 0.05;
//            //                    q1_max_new = _q1_max; // change step length
//            //                    double theta = _theta_steer; // change heading
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                    sin(theta), cos(theta);
//            //                }
//            //                else if (_step_counter >= 8 && _step_counter < 9)   /*Left*/
//            //                {
//            //                    _q1_max = 0.05;
//            //                    q1_max_new = _q1_max; // change step length
//            //                    double theta = _theta_steer; // change heading
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                    sin(theta), cos(theta);
//            //                }
//            //                else if (_step_counter >= 9 && _step_counter < 10) /*Right*/
//            //                {
//            //                    _q1_max = 0.05;
//            //                    q1_max_new = _q1_max; // change step length
//            //                    double theta = _theta_steer; // change heading
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                    sin(theta), cos(theta);
//            //                }
//            //                else if (_step_counter >= 10 && _step_counter < 15)
//            //                {
//            //                    _q1_max = 0.05;
//            //                    q1_max_new = _q1_max; // change step length
//            //                    double theta = _theta_steer; // change heading
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                    sin(theta), cos(theta);
//            //                }
//            //                else
//            //                {
//            //                    _q1_max = - 0.05;
//            //                    q1_max_new = _q1_max;
//            //                    double theta = 0;
//            //                    R_steer << cos(theta), -sin(theta),
//            //                                sin(theta), cos(theta);
//            //                }
//            /*----------------generate q1-------------------------*/
//            double q1 = (q1_max_new - _q1_min);


//            _steep_coeff = (q1_max_new - _q1_min)/_step_duration;
//            /*----------------------------------------------------*/

//            std::cout << "q1_max: " << q1_max_new << std::endl;
//            std::cout << "q1_min: " << _q1_min << std::endl;
//            std::cout << "angle: " << q1 << std::endl;

//            Eigen::Vector2d disp_com; // displacement in the xy plane
//            Eigen::Vector2d disp_com_rot; // displacement in the xy plane
//            disp_com << fabs(_current_pose_ROS.get_distance_ankle_to_com(_current_side).z()) * tan(q1), 0; // displacement of com in x

//            disp_com_rot = R_steer * disp_com; // angle steering
//            std::cout << "disp_com_rot: " << disp_com_rot.transpose() << std::endl;

//            _final_com_position.head(2) = _initial_com_position.head(2) + disp_com_rot;
//            _final_com_position_fake.head(2) = _initial_com_position_fake.head(2) + disp_com;



//            _final_sole_pose.translation() = _current_pose_ROS.get_sole_tot(_other_side).translation() + 2 * (_final_com_position - _current_pose_ROS.get_sole_tot(_other_side).translation());  // get final step given the displacement vector

//            /* TODO refactor: this is needed for the steering (orientation) */
//            double theta_heading;
//            /* orientation */

//            //                if (_step_counter >= 4 && _step_counter < 15)
//            //                {
//            //                    theta_heading = _theta_steer;
//            //                }
//            //                else
//            //                {
//            theta_heading = 0;
//            //                }

//            /* Sole */
//            _final_sole_pose.linear() = (Eigen::AngleAxisd(theta_heading, Eigen::Vector3d::UnitZ())).toRotationMatrix();

//            /* Waist */

//            _final_waist_pose.linear() = (Eigen::AngleAxisd(theta_heading, Eigen::Vector3d::UnitZ())).toRotationMatrix();
//}

//}
